// =============================================================================
// cad_converter — convert a STEP model into simulation-ready meshes for
// MEGAlib/Geant4.
//
//   cad_converter [options] model.step
//
//   --max-deviation <d>   accuracy of the output mesh, in model units
//                         (default: 1/1000 of the model's largest dimension)
//   --angular-deflection <deg>
//                         largest angle between neighbouring triangles along
//                         a curve (default 20: a circle gets at least 18
//                         segments). Larger values give small round features
//                         fewer segments; --max-deviation still holds.
//   --min-volume <v>      remove every solid smaller than <v> (model units
//                         cubed, e.g. mm^3) before anything else is done
//   --obj                 write model.obj (Wavefront OBJ, indexed vertices)
//   --stl                 write model.stl (multi-solid ASCII STL)
//   --geo                 write a MEGAlib/Geomega geometry: model.geo.setup,
//                         including one .geo file per object (in model_geo/)
//                         with a TESSELLATED volume per body
//                         Any combination of --obj, --stl and --geo can be
//                         given; with none, the output is model.obj.
//   --output-dir <dir>    write the output files into <dir> (created if
//                         missing) instead of next to the input
//   --per-body            write every body as its own object (solid_NNNN, ...),
//                         instead of one object per part (all its placements)
//   --no-repair           skip checking and repairing the CAD geometry
//   --no-collisions       don't make the solids conform: they keep their
//                         modeled shape, so overlaps stay and touching meshes
//                         aren't flush
//   --full-accuracy-check measure the deviation of every triangle, even on
//                         very large meshes (default: a sample of 200k)
//
// Debugging big models: --dump-fused <file.brep> saves the conformed model
// (read, repaired, made conformal, after all checks; no mesh), with
// <file.brep>.parts (each body's part) and <file.brep>.meshcut (the
// bodies left out of the fuse). Giving that .brep file as the input instead
// of a STEP file skips straight to meshing.
//
// The pipeline:
//   1. Read the STEP file (step_import.cpp), keeping only its solids: shells
//      and loose faces are removed, since Geant4 and ROOT need watertight
//      solids. Check and repair every unique part, fix faces the import put
//      on the wrong arc (folded_faces.cpp), and put every revolved face's
//      seam at U = 0 (repair.cpp).
//   2. List the bodies: every placed solid. A solid whose shell is still open
//      after repair isn't watertight, and is removed.
//   3. Make the solids conform (conform.cpp): a general fuse makes touching
//      solids share faces and gives every overlap to the smaller solid.
//      Modeled threads (thread_pairs.cpp) and pairs the fuse breaks are left
//      out of it.
//   4. Mesh all bodies with OCCT (meshing.cpp), with a structured mesh on full
//      cylinders, cones and spheres (revolved_mesh.cpp), weld each body into
//      one mesh and measure it against the CAD geometry (weld.cpp).
//   5. Cut every remaining overlap between solid meshes (mesh_cut.cpp), and
//      check every solid's mesh is closed; a defective fused solid sends its
//      neighbourhood back to step 3.
//   6. Write one named mesh per part (or per body) to an OBJ and/or STL file,
//      next to the input or in --output-dir (export.cpp), and/or a Geomega
//      geometry (geomega_export.cpp).
//
// Every stage prints its name when it starts and its duration when it ends.
// Problems are printed as WARNING lines; the exit code is non-zero only if
// the model couldn't be converted at all.
// =============================================================================
#include "body.hpp"
#include "conform.hpp"
#include "export.hpp"
#include "geomega_export.hpp"
#include "folded_faces.hpp"
#include "mesh_cut.hpp"
#include "meshing.hpp"
#include "repair.hpp"
#include "revolved_mesh.hpp"
#include "stage_timer.hpp"
#include "step_import.hpp"
#include "weld.hpp"

#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <GProp_GProps.hxx>
#include <BRep_Builder.hxx>
#include <OSD_Parallel.hxx>
#include <BinTools.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
// Above this many curved triangles, the deviation is measured on an even
// sample of this size (--full-accuracy-check measures all).
constexpr size_t kDeviationSampleLimit = 200000;
// A triangle lower than this fraction of the accuracy limit is a sliver
// (zero area, see CutAndCount).
constexpr double kSliverFraction = 1.0e-9;
// Triangles smaller than this (mm^2) are collapsed: ROOT's TGeoTessellated
// reports a facet under 1e-10 cm^2 (1e-8 mm^2) as degenerate or of zero
// surface area; this keeps a factor 10 margin. Collapsing moves a point by
// at most kMaxCollapseFraction of the accuracy limit.
constexpr double kMinTriangleArea = 1.0e-7;
constexpr double kMaxCollapseFraction = 0.01;

struct Options
{
    std::string input;
    double maxDeviation = 0.0; // 0 = the model's default
    double meshAngleDeg = kDefaultMeshAngleDeg;
    double minVolume = 0.0; // 0 = keep every solid
    // Output formats: any combination; none given means OBJ.
    bool writeObj = false;
    bool writeStl = false;
    bool writeGeo = false;
    bool perBody = false;
    bool repair = true;
    bool collisions = true;
    bool fullAccuracyCheck = false;
    std::string outputDir; // "" = next to the input
    std::string dumpPath;
};

bool ParseArguments(int argc, char** argv, Options& options)
{
    std::vector<std::string> inputs;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--max-deviation" && i + 1 < argc)
        {
            options.maxDeviation = std::stod(argv[++i]);
        }
        else if (arg == "--obj")
        {
            options.writeObj = true;
        }
        else if (arg == "--stl")
        {
            options.writeStl = true;
        }
        else if (arg == "--geo")
        {
            options.writeGeo = true;
        }
        else if (arg == "--per-body")
        {
            options.perBody = true;
        }
        else if (arg == "--no-repair")
        {
            options.repair = false;
        }
        else if (arg == "--no-collisions")
        {
            options.collisions = false;
        }
        else if (arg == "--angular-deflection" && i + 1 < argc)
        {
            options.meshAngleDeg = std::stod(argv[++i]);
            if (!(options.meshAngleDeg > 0.0 && options.meshAngleDeg <= 180.0))
            {
                std::cerr << "--angular-deflection must be between 0 and 180 degrees\n";
                return false;
            }
        }
        else if (arg == "--min-volume" && i + 1 < argc)
        {
            options.minVolume = std::stod(argv[++i]);
            if (!(options.minVolume >= 0.0))
            {
                std::cerr << "--min-volume must not be negative\n";
                return false;
            }
        }
        else if (arg == "--full-accuracy-check")
        {
            options.fullAccuracyCheck = true;
        }
        else if (arg == "--output-dir" && i + 1 < argc)
        {
            options.outputDir = argv[++i];
        }
        else if (arg == "--dump-fused" && i + 1 < argc)
        {
            options.dumpPath = argv[++i];
        }
        else if (!arg.empty() && arg[0] == '-')
        {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
        else
        {
            inputs.push_back(arg);
        }
    }
    if (inputs.size() != 1)
    {
        return false;
    }
    if (!options.writeObj && !options.writeStl && !options.writeGeo)
    {
        options.writeObj = true;
    }
    options.input = inputs.front();
    return true;
}

// Removes every solid smaller than 'minVolume' (--min-volume), to simplify
// the model: screws, washers, small electronics. This happens before the
// fuse, so a removed solid takes nothing from its neighbours (the fuse gives
// an overlap to the smaller solid; removed afterwards, it would leave a
// void). The bodies are numbered again, solid_0001, ... in order.
void RemoveSmallSolids(std::vector<Body>& bodies, double minVolume)
{
    if (minVolume <= 0.0)
    {
        return;
    }
    std::vector<double> volume(bodies.size(), 0.0);
    OSD_Parallel::For(0, static_cast<int>(bodies.size()), [&](int i) {
        GProp_GProps props;
        BRepGProp::VolumeProperties(bodies[i].shape, props);
        volume[i] = std::fabs(props.Mass());
    });
    std::vector<Body> kept;
    std::map<std::string, size_t> removedParts; // part (or body) name -> placements removed
    double removedVolume = 0.0;
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (volume[i] >= minVolume)
        {
            kept.push_back(std::move(bodies[i]));
        }
        else
        {
            ++removedParts[bodies[i].part.empty() ? bodies[i].name : bodies[i].part];
            removedVolume += volume[i];
        }
    }
    const size_t removed = bodies.size() - kept.size();
    std::cout << "  removed " << removed << " solid(s) smaller than " << minVolume << " (--min-volume), "
              << removedVolume << " in all";
    size_t shown = 0;
    for (const auto& [name, count] : removedParts)
    {
        if (shown++ < 10)
        {
            std::cout << (shown == 1 ? ": " : ", ") << name << (count > 1 ? " x" + std::to_string(count) : "");
        }
    }
    std::cout << (removedParts.size() > 10 ? ", ..." : "") << "\n";
    for (size_t i = 0; i < kept.size(); ++i)
    {
        kept[i].name = SolidName(i);
    }
    bodies = std::move(kept);
}

// Removes every body that isn't a watertight solid (IsWatertight), and
// numbers the rest solid_0001, solid_0002, ... in order.
void RemoveOpenSolids(std::vector<Body>& bodies)
{
    std::vector<std::string> removed;
    std::vector<Body> kept;
    for (Body& body : bodies)
    {
        if (IsWatertight(body.shape))
        {
            kept.push_back(std::move(body));
        }
        else
        {
            removed.push_back(body.part.empty() ? body.name : body.name + " (" + body.part + ")");
        }
    }
    if (!removed.empty())
    {
        std::cout << "  WARNING: removed " << removed.size() << " solid(s) that aren't watertight (open shell):";
        for (size_t i = 0; i < removed.size() && i < 10; ++i)
        {
            std::cout << " " << removed[i];
        }
        std::cout << (removed.size() > 10 ? " ..." : "") << "\n";
    }
    for (size_t i = 0; i < kept.size(); ++i)
    {
        kept[i].name = SolidName(i);
    }
    bodies = std::move(kept);
}

// Steps 1-3: read the STEP file, repair it, list the bodies and make the
// solids conform. Sets 'maxDeviation' and fills 'fuse' (for later local
// re-fuses).
std::vector<Body> ReadAndPrepare(const Options& options, double& maxDeviation, FuseContext& fuse)
{
    StepModel step = ReadStep(options.input);
    TopoDS_Shape shape = step.shape;
    // Each placed body's part, in PlacedBodies order (every stage below keeps
    // the bodies in that order, so this maps onto them later).
    std::vector<PartInfo> partOfBody;
    for (const Body& body : PlacedBodies(shape))
    {
        auto found = step.parts.find(body.shape.TShape().get());
        partOfBody.push_back(found != step.parts.end() ? found->second : PartInfo{});
    }

    // Some STEP translators leave edges with only a 2D curve on their surface;
    // BuildCurves3d fills in the missing 3D ones (and harmlessly "fails" on
    // degenerate edges, which have none).
    {
        StageTimer timer("building missing 3D edge curves");
        BRepLib::BuildCurves3d(shape);
    }
    if (options.repair)
    {
        shape = CheckAndRepair(shape);
        StageTimer timer("fixing faces folded onto their neighbour");
        int folded = 0;
        shape = FixFoldedLunes(shape, folded);
        std::cout << "  rebuilt " << folded << " folded face(s) on the other arc\n";
    }
    {
        StageTimer timer("normalizing periodic seams");
        shape = NormalizePeriodicSeams(shape);
    }

    std::vector<Body> bodies = PlacedBodies(shape);
    if (partOfBody.size() == bodies.size())
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            bodies[i].part = partOfBody[i].name;
            bodies[i].partBodies = partOfBody[i].bodies;
        }
    }
    RemoveOpenSolids(bodies);
    RemoveSmallSolids(bodies, options.minVolume);
    std::cout << "  " << bodies.size() << " bodies\n";
    if (bodies.empty())
    {
        throw std::runtime_error("No solids left to convert (watertight, and at least --min-volume): " +
                                 options.input);
    }

    // The accuracy limit drives every meshing step.
    TopoDS_Compound kept;
    BRep_Builder builder;
    builder.MakeCompound(kept);
    for (const Body& body : bodies)
    {
        builder.Add(kept, body.shape);
    }
    maxDeviation = options.maxDeviation > 0.0 ? options.maxDeviation : DefaultMaxDeviation(kept);
    std::cout << "  mesh accuracy limit: " << maxDeviation << " (model units)\n";

    if (options.collisions)
    {
        StageTimer timer("making solids conform (shared contact faces, overlaps resolved)");
        fuse = MakeBodiesConformal(bodies, maxDeviation, /*detectThreadPairs=*/true);
    }
    return bodies;
}

// The debugging shortcut: a model saved with --dump-fused.
std::vector<Body> LoadSavedModel(const Options& options, double& maxDeviation)
{
    StageTimer timer("reading saved model");
    if (options.minVolume > 0.0)
    {
        std::cout << "  note: --min-volume is ignored for a saved model: it is already fused, so removing a solid"
                     " now would leave a void where it overlapped a neighbour\n";
    }
    TopoDS_Shape loaded;
    if (!BinTools::Read(loaded, options.input.c_str()) || loaded.ShapeType() != TopAbs_COMPOUND)
    {
        throw std::runtime_error("Cannot read saved model: " + options.input);
    }
    std::vector<Body> bodies;
    for (TopoDS_Iterator it(loaded); it.More(); it.Next())
    {
        bodies.push_back({SolidName(bodies.size()), it.Value()});
    }
    // Each line: the number of bodies in one placement of the part, a tab,
    // and the part's name.
    std::ifstream partList(options.input + ".parts");
    std::string line;
    for (size_t index = 0; index < bodies.size() && std::getline(partList, line); ++index)
    {
        const size_t tab = line.find('\t');
        bodies[index].partBodies = tab == std::string::npos ? 1 : std::max<size_t>(1, std::stoul(line.substr(0, tab)));
        bodies[index].part = tab == std::string::npos ? line : line.substr(tab + 1);
    }
    std::ifstream meshCutList(options.input + ".meshcut");
    for (size_t index; meshCutList >> index;)
    {
        if (index < bodies.size())
        {
            bodies[index].meshCut = true;
        }
    }
    maxDeviation = options.maxDeviation > 0.0 ? options.maxDeviation : DefaultMaxDeviation(loaded);
    std::cout << "  " << bodies.size() << " bodies; mesh accuracy limit: " << maxDeviation << " (model units)\n";
    return bodies;
}

void SaveModel(const std::vector<Body>& bodies, const std::string& path)
{
    StageTimer timer("saving the conformed model (without its mesh) to " + path);
    TopoDS_Compound model;
    BRep_Builder builder;
    builder.MakeCompound(model);
    for (const Body& body : bodies)
    {
        builder.Add(model, body.shape);
    }
    BinTools::Write(model, path.c_str(), /*theWithTriangles=*/false, /*theWithNormals=*/false,
                    BinTools_FormatVersion_CURRENT);
    std::ofstream partList(path + ".parts");
    for (const Body& body : bodies)
    {
        partList << body.partBodies << '\t' << body.part << "\n";
    }
    std::ofstream meshCutList(path + ".meshcut");
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (bodies[i].meshCut)
        {
            meshCutList << i << "\n";
        }
    }
}

// Step 4: mesh, weld and measure the bodies listed in 'which': all of them at
// first, and after a local re-fuse only the ones that changed (the others
// keep their triangulation). They are meshed together, as one compound, so a
// face shared by two bodies is meshed once for both; the compound's children
// stay in order, so they map back to the bodies. Fills welded[i] for each.
void MeshAndWeld(std::vector<Body>& bodies, const std::vector<size_t>& which, double maxDeviation,
                 const Options& options, std::vector<SurfaceMesh>& welded, MeshQuality& quality)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (size_t i : which)
    {
        builder.Add(compound, bodies[i].shape);
    }
    TopoDS_Shape shape = compound;
    if (options.collisions)
    {
        shape = NormalizePeriodicSeams(shape, /*verbose=*/false); // faces made by the fuse
    }

    // Use the returned shape from here on: it may be a rebuilt copy.
    TopoDS_Shape meshed;
    {
        StageTimer timer("meshing " + std::to_string(which.size()) + " bodies with OCCT (parallel)");
        meshed = MeshWithFallback(shape, maxDeviation, options.meshAngleDeg * kPi / 180.0);
    }
    {
        StageTimer timer("structured meshing of revolved faces");
        const int structured = RetessellateRevolvedFaces(meshed, maxDeviation, options.meshAngleDeg * kPi / 180.0);
        std::cout << "  structured mesh on " << structured << " revolved face(s)\n";
    }
    size_t child = 0;
    for (TopoDS_Iterator it(meshed); it.More() && child < which.size(); it.Next(), ++child)
    {
        bodies[which[child]].shape = it.Value();
    }
    if (child != which.size())
    {
        throw std::runtime_error("internal error: lost track of bodies while meshing");
    }

    // A face still without a mesh would be a hole in its body; name them.
    std::vector<std::string> incomplete;
    for (size_t i : which)
    {
        if (CountUnmeshedFaces(bodies[i].shape) > 0)
        {
            incomplete.push_back(bodies[i].name);
        }
    }
    if (!incomplete.empty())
    {
        std::cout << "  WARNING: " << incomplete.size() << " body(ies) have faces that could not be meshed:";
        for (size_t i = 0; i < incomplete.size() && i < 10; ++i)
        {
            std::cout << " " << incomplete[i];
        }
        std::cout << (incomplete.size() > 10 ? " ..." : "") << "\n";
        DescribeUnmeshedFaces(bodies, 5);
    }

    std::vector<TopoDS_Shape> shapes;
    for (size_t i : which)
    {
        shapes.push_back(bodies[i].shape);
    }
    std::vector<SurfaceMesh> result;
    {
        StageTimer timer("welding and measuring the meshes");
        result = WeldAndMeasure(shapes, options.fullAccuracyCheck ? 0 : kDeviationSampleLimit, quality);
    }
    if (result.size() != which.size())
    {
        throw std::runtime_error("could not build meshes from the triangulation");
    }
    for (size_t k = 0; k < which.size(); ++k)
    {
        welded[which[k]] = std::move(result[k]);
    }
}

// Step 5: Geant4 needs distinct volumes not to overlap at all. Bodies left
// out of the fuse still overlap their neighbours, the fuse can miss an
// interference, and triangles on a concave face bulge slightly past the true
// surface into a neighbour. So every overlap between two solid meshes is cut
// out of the larger body's mesh (mesh_cut.cpp). Then holes and edges used by
// more than two triangles are counted on the final meshes. (The deviation was
// measured before the cut; a cut surface is made of the smaller body's
// triangles.)
//
// Slivers (zero-area triangles, a vertex lying on the opposite edge, left
// where a vertex of one face sits on an edge of the next) are flipped away
// first: ROOT and Geant4 reject degenerate facets. "Zero" is a height below
// kSliverFraction of the accuracy limit; on cosi_test.step the slivers are
// 1e-16 to 1e-13 mm high, and the next-thinnest triangles 1e-8 mm.
//
// Problems go to 'warnings' rather than straight to the output: a later pass
// (a local re-fuse, see main) often fixes them, so only the last pass's are
// printed.
std::vector<SurfaceMesh> CutAndCount(const std::vector<Body>& bodies, const std::vector<SurfaceMesh>& welded,
                                     bool cut, double maxDeviation, MeshQuality& quality,
                                     std::vector<std::string>& warnings)
{
    warnings.clear();
    std::vector<SurfaceMesh> meshes = welded;

    // Slivers and tiny triangles are removed before the cut and after it. A
    // collapse can push a surface a hair (1e-9 to 1e-7 mm^3 on
    // cosi_test.step) into a neighbour whose face passes the point without a
    // vertex there; before the cut that is cut away, after it a collapse is
    // only made if it leaves every body apart (keepApart). (Cutting such a
    // hair instead leaves new tiny triangles, so that doesn't converge.)
    size_t flipped = 0, collapsed = 0, slivers = 0, tiny = 0;
    auto cleanUp = [&](bool keepApart) {
        StageTimer timer("removing sliver and tiny triangles");
        std::vector<size_t> flips(meshes.size(), 0), left(meshes.size(), 0);
        OSD_Parallel::For(0, static_cast<int>(meshes.size()), [&](int i) {
            flips[i] = FlipDegenerateTriangles(meshes[i], kSliverFraction * maxDeviation, kMinTriangleArea, left[i]);
        });
        flipped += std::accumulate(flips.begin(), flips.end(), size_t(0));
        slivers = std::accumulate(left.begin(), left.end(), size_t(0));
        collapsed +=
            CollapseTinyTriangles(meshes, kMinTriangleArea, kMaxCollapseFraction * maxDeviation, keepApart, tiny);
    };
    cleanUp(false);
    if (cut)
    {
        StageTimer timer("cutting every overlap between solid meshes");
        std::vector<int> part;
        std::map<std::string, int> partIds;
        for (const Body& body : bodies)
        {
            part.push_back(body.meshCut && !body.part.empty()
                               ? partIds.emplace(body.part, static_cast<int>(partIds.size())).first->second
                               : -1);
        }
        const MeshCutReport report = CutOverlappingMeshes(meshes, part);
        std::cout << "  " << report.overlappingPairs << " overlapping pair(s); " << report.bodiesCut
                  << " larger body(ies) lost a total volume of " << report.volumeRemoved << "; "
                  << report.mergedCutters << " cut(s) merged several placements of a part into one cutter; "
                  << report.slitsClosed << " zero-thickness slit(s) closed\n";
        warnings.insert(warnings.end(), report.problems.begin(), report.problems.end());
        cleanUp(true);
    }
    if (flipped > 0 || collapsed > 0)
    {
        std::cout << "  " << flipped << " sliver triangle(s) (zero area) removed by edge flips; " << collapsed
                  << " tiny triangle edge(s) (area under " << kMinTriangleArea << " mm^2) collapsed\n";
    }
    if (slivers > 0)
    {
        warnings.push_back(std::to_string(slivers) + " sliver triangle(s) (zero area) could not be removed");
    }
    if (tiny > 0)
    {
        std::ostringstream text;
        text << tiny << " triangle(s) smaller than " << kMinTriangleArea
             << " mm^2 are left; ROOT may report them as degenerate";
        warnings.push_back(text.str());
    }
    quality.openEdges = quality.nonManifoldEdges = 0;
    quality.openEdgesPerBody.assign(meshes.size(), 0);
    for (size_t i = 0; i < meshes.size(); ++i)
    {
        size_t open = 0, nonManifold = 0;
        CountEdgeDefects(meshes[i], open, nonManifold);
        quality.openEdgesPerBody[i] = open;
        quality.openEdges += open;
        quality.nonManifoldEdges += nonManifold;
    }
    return meshes;
}

// Prints mesh quality, and warns about anything that would make the mesh
// unusable for simulation: a solid whose mesh has holes (open edges), edges
// shared by more than two triangles, or a deviation above the limit.
void ReportMeshQuality(const MeshQuality& q, const std::vector<Body>& bodies, double maxDeviation)
{
    std::string sampled;
    if (q.deviationMeasured < q.deviationEligible)
    {
        sampled = " (sampled on " + std::to_string(q.deviationMeasured) + " of " +
                  std::to_string(q.deviationEligible) + " curved triangles; --full-accuracy-check for all)";
    }
    std::cout << "  mesh: " << q.vertices << " vertices, " << q.triangles << " triangles, max deviation "
              << q.maxDeviation << sampled << ", min angle " << q.minAngleDeg << " deg, " << q.trianglesBelow20Deg
              << " triangle(s) below 20 deg\n";
    for (const std::string& note : q.notes)
    {
        std::cout << "    " << note << "\n";
    }

    size_t closedSolids = 0;
    std::vector<std::string> leakySolids;
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        q.openEdgesPerBody[i] > 0 ? leakySolids.push_back(bodies[i].name) : (void)++closedSolids;
    }
    std::cout << "  bodies: " << closedSolids << " closed solid(s), " << leakySolids.size()
              << " solid(s) with holes\n";

    if (!leakySolids.empty())
    {
        std::cout << "  WARNING: " << leakySolids.size() << " solid(s) have holes or cracks in their mesh:";
        for (size_t i = 0; i < leakySolids.size() && i < 10; ++i)
        {
            std::cout << " " << leakySolids[i];
        }
        std::cout << (leakySolids.size() > 10 ? " ..." : "") << "\n";
    }
    if (q.nonManifoldEdges > 0)
    {
        std::cout << "  WARNING: " << q.nonManifoldEdges << " edge(s) shared by more than two triangles\n";
    }
    if (q.maxDeviation > maxDeviation * (1.0 + 1.0e-6))
    {
        std::cout << "  WARNING: max deviation " << q.maxDeviation << " exceeds the limit " << maxDeviation << "\n";
    }
}
} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!ParseArguments(argc, argv, options))
    {
        std::cerr << "Usage: " << argv[0]
                  << " [--max-deviation <d>] [--angular-deflection <deg>] [--min-volume <v>] [--obj] [--stl] [--geo]"
                     " [--per-body] [--output-dir <dir>] [--no-repair]"
                     " [--no-collisions] [--full-accuracy-check] [--dump-fused <file.brep>] model.step\n";
        return 1;
    }

    std::cout << "== " << options.input << " ==\n";
    StageTimer total("total");
    try
    {
        double maxDeviation = 0.0;
        FuseContext fuse; // the solids' original shapes, to re-fuse a neighbourhood later
        std::vector<Body> bodies = fs::path(options.input).extension() == ".brep"
                                       ? LoadSavedModel(options, maxDeviation)
                                       : ReadAndPrepare(options, maxDeviation, fuse);

        std::vector<SurfaceMesh> welded(bodies.size());
        MeshQuality quality;
        std::vector<size_t> all(bodies.size());
        std::iota(all.begin(), all.end(), size_t(0));
        MeshAndWeld(bodies, all, maxDeviation, options, welded, quality);
        std::vector<std::string> cutWarnings; // the last pass's (see CutAndCount)
        std::vector<SurfaceMesh> meshes =
            CutAndCount(bodies, welded, options.collisions, maxDeviation, quality, cutWarnings);

        // A fused solid whose final mesh isn't closed (PartnersOfMeshDefects):
        // leave the smaller body of each defective pair out of the fuse,
        // re-fuse only their neighbourhood, and mesh only what changed.
        for (int pass = 2; fuse.ready && pass <= kMaxMeshPasses; ++pass)
        {
            Bnd_Box box;
            for (const SurfaceMesh& mesh : meshes)
            {
                for (const gp_Pnt& p : mesh.vertices)
                {
                    box.Add(p);
                }
            }
            const std::vector<size_t> more =
                PartnersOfMeshDefects(bodies, meshes, kTouchFraction * std::sqrt(box.SquareExtent()));
            if (more.empty())
            {
                break;
            }
            std::cout << "  leaving " << more.size() << " more solid(s) out of the fuse (their overlaps are cut on the"
                      << " mesh); re-fusing and meshing only their neighbourhood\n";
            std::set<std::string> changed = RefuseLocally(bodies, fuse, more);
            const std::set<std::string> repaired = RepairFuse(bodies, fuse, changed);
            changed.insert(repaired.begin(), repaired.end());
            std::vector<size_t> which;
            for (size_t i = 0; i < bodies.size(); ++i)
            {
                if (changed.count(bodies[i].name))
                {
                    which.push_back(i);
                }
            }
            // The reported quality: the worst of the first measurement and the
            // re-meshed bodies' (an upper bound, since the re-meshed bodies'
            // old meshes were in the first one); totals from the final meshes.
            MeshQuality partQuality;
            MeshAndWeld(bodies, which, maxDeviation, options, welded, partQuality);
            quality.maxDeviation = std::max(quality.maxDeviation, partQuality.maxDeviation);
            quality.minAngleDeg = std::min(quality.minAngleDeg, partQuality.minAngleDeg);
            quality.vertices = quality.triangles = 0;
            for (const SurfaceMesh& mesh : welded)
            {
                quality.vertices += mesh.vertices.size();
                quality.triangles += mesh.triangles.size();
            }
            meshes = CutAndCount(bodies, welded, options.collisions, maxDeviation, quality, cutWarnings);
        }

        // Saved after the checks, so the bodies they took out of the fuse are
        // included.
        if (!options.dumpPath.empty())
        {
            SaveModel(bodies, options.dumpPath);
        }
        for (const std::string& warning : cutWarnings)
        {
            std::cout << "  WARNING: " << warning << "\n";
        }
        ReportMeshQuality(quality, bodies, maxDeviation);

        // Step 6: one named mesh per part (or per body), in every format asked
        // for: OBJ, STL, Geomega.
        const std::vector<OutputObject> objects = GroupBodies(bodies, meshes, options.perBody);
        if (!options.outputDir.empty())
        {
            fs::create_directories(options.outputDir);
        }
        auto outputPath = [&](const char* extension) {
            fs::path path = fs::path(options.input).replace_extension(extension);
            if (!options.outputDir.empty())
            {
                path = fs::path(options.outputDir) / path.filename();
            }
            return path.make_preferred().string();
        };
        const std::string grouping = options.perBody ? "" : " (one per part)";
        if (options.writeObj)
        {
            const std::string path = outputPath(".obj");
            {
                StageTimer timer("writing OBJ");
                ExportObj(objects, meshes, path);
            }
            std::cout << "  wrote " << bodies.size() << " bodies as " << objects.size() << " object(s)" << grouping
                      << " to " << path << "\n";
        }
        if (options.writeStl)
        {
            const std::string path = outputPath(".stl");
            {
                StageTimer timer("writing STL");
                ExportStl(objects, meshes, path);
            }
            std::cout << "  wrote " << bodies.size() << " bodies as " << objects.size() << " object(s)" << grouping
                      << " to " << path << "\n";
        }
        if (options.writeGeo)
        {
            const std::string path = outputPath(".geo.setup");
            StageTimer timer("writing Geomega geometry");
            const GeomegaReport report =
                ExportGeomega(objects, meshes, path, fs::path(options.input).filename().string());
            std::cout << "  wrote " << report.volumes << " TESSELLATED volume(s) in " << objects.size()
                      << " object file(s) to " << report.meshDir << ", included by " << path << "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "  ERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
