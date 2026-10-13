// =============================================================================
// conform.cpp — make the solids conform to each other.
//
// Meshing every body on its own leaves its neighbours' meshes slightly out
// of step: where two parts touch, each side's triangles are within the
// accuracy limit of the true surface, but not of each other, so the meshes
// have tiny gaps and interpenetrations along every contact. And where the
// CAD parts themselves overlap (interference fits, a screw modeled at nominal
// size in a smaller hole, a part placed twice), two volumes claim the same
// space. A particle simulation needs neither.
//
// The general fuse (BRepAlgoAPI_BuilderAlgo) on all solids at once fixes
// both at the B-rep level, before anything is meshed:
//   - Every face that two solids share (a contact) is split so the shared
//     area becomes a single face that both solids reference. Meshing then
//     triangulates it once, and both bodies' meshes use exactly the same
//     triangles there: perfectly flush, with no gap and no overlap.
//   - Where solids overlap, the common region becomes its own piece, listed
//     in the history of every solid it came from. Each such piece is given
//     to the *smallest* of those solids (by original volume) and taken away
//     from the others. That keeps small parts whole: a screw that is too
//     thick widens its hole rather than getting thinner, and a part sitting
//     inside another becomes a cavity in it instead of disappearing.
//   - A body that ends up with (almost) nothing left, typically the same
//     part placed twice, is dropped.
// A body can end up as several pieces (its own part, plus overlaps it was
// given). They share internal faces, which the mesher skips, so the body is
// still one closed surface.
//
// Options: SetNonDestructive keeps the inputs untouched (placements of the
// same part share geometry); SetFuzzyValue lets faces that are *almost*
// coincident (within 1e-6 of the model size, i.e. CAD round-off) count as
// touching; SetUseOBB speeds up the interference search with oriented boxes.
//
// Thread pairs: a fully modeled screw thread in a fully modeled tapped hole
// has two helical surfaces that almost, but not quite, coincide, and the
// fuse turns them into slivers far thinner than the mesh can resolve (holes,
// edges shared by more than two triangles, inside-out triangles). If
// 'detectThreadPairs' is set, FindThreadPairs (thread_pairs.cpp) finds those
// pairs first, and the smaller body of each pair (the one that keeps the overlap
// anyway) is left out of the fuse. It is meshed on its own, and every
// overlap it has is cut on the finished meshes instead (mesh_cut.cpp), with
// the same rule: the larger body loses the overlap. Its contacts are then
// free of overlaps, but not shared triangle for triangle as fused contacts
// are: where its surface and a neighbour's nearly touch, a gap up to the
// mesh accuracy can remain.
//
// The fuse's result is checked, not trusted. It can break on contacts too
// small or too tangent for it, without reporting an error: on
// cosi_test.step, the conical cup point of a 1.5 mm set screw pressing into
// a cap screw came out as a piece with negative volume (inside out) in one
// body and a zero-thickness INTERNAL face in both, which left holes in both
// meshes (Geant4 needs every solid closed). Elsewhere a tiny overlap came
// out as a piece touching the rest of its body only along a circle, so the
// body's surface pinched there (four triangles on one edge). So after each
// fuse, every fused body is checked (BrokenFuseResult): a piece with volume
// <= 0, an INTERNAL or EXTERNAL face, a pinch, or BRepCheck failing. For
// each broken body, the bodies sharing its bad faces are its partners; the
// smaller body of each such pair is left out of the fuse and cut on the mesh
// like a thread pair, and only its neighbourhood is fused again
// (RefuseLocally), up to kMaxFusePasses times.
// =============================================================================
#include "conform.hpp"

#include "occt_types.hpp"
#include "stage_timer.hpp"
#include "thread_pairs.hpp"

#include <BRepAlgoAPI_BuilderAlgo.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <OSD_Parallel.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>

namespace
{
double SolidVolume(const TopoDS_Shape& solid)
{
    GProp_GProps props;
    BRepGProp::VolumeProperties(solid, props);
    return std::abs(props.Mass());
}

constexpr int kMaxFusePasses = 3;

// Why a fused body is broken ("" if it isn't). 'badFaces' gets the faces
// that are wrong: those of pieces with volume <= 0 and INTERNAL/EXTERNAL ones.
// (The volume here is signed: an inside-out piece has a negative one.)
std::string BrokenFuseResult(const TopoDS_Shape& shape, std::vector<TopoDS_Shape>& badFaces)
{
    std::ostringstream why;
    size_t inverted = 0, internal = 0;
    for (TopExp_Explorer piece(shape, TopAbs_SOLID); piece.More(); piece.Next())
    {
        GProp_GProps props;
        BRepGProp::VolumeProperties(piece.Current(), props);
        if (props.Mass() <= 0.0)
        {
            ++inverted;
            for (TopExp_Explorer face(piece.Current(), TopAbs_FACE); face.More(); face.Next())
            {
                badFaces.push_back(face.Current());
            }
        }
    }
    for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next())
    {
        if (face.Current().Orientation() == TopAbs_INTERNAL || face.Current().Orientation() == TopAbs_EXTERNAL)
        {
            ++internal;
            badFaces.push_back(face.Current());
        }
    }
    // Pinches: an edge of the body's surface used by more than two of its
    // faces (two pieces of the body touching only along a line). A face used
    // twice by the body is between two of its pieces, not on its surface.
    // Faces and edges are told apart by TShape *and* location (IsSame), as
    // the mesher does: a part can reuse one face or edge at several places.
    ShapeIndexMap faceIndex;
    std::vector<int> faceUses;
    for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next())
    {
        const int k = faceIndex.Add(face.Current());
        faceUses.resize(faceIndex.Extent(), 0);
        ++faceUses[k - 1];
    }
    ShapeIndexMap edgeIndex;
    std::vector<std::vector<TopoDS_Shape>> edgeFaces;
    for (int f = 1; f <= faceIndex.Extent(); ++f)
    {
        if (faceUses[f - 1] != 1)
        {
            continue;
        }
        for (TopExp_Explorer edge(faceIndex(f), TopAbs_EDGE); edge.More(); edge.Next())
        {
            if (!BRep_Tool::Degenerated(TopoDS::Edge(edge.Current())))
            {
                const int k = edgeIndex.Add(edge.Current());
                edgeFaces.resize(edgeIndex.Extent());
                edgeFaces[k - 1].push_back(faceIndex(f));
            }
        }
    }
    size_t pinches = 0;
    for (const std::vector<TopoDS_Shape>& faces : edgeFaces)
    {
        if (faces.size() > 2)
        {
            ++pinches;
            badFaces.insert(badFaces.end(), faces.begin(), faces.end());
        }
    }
    if (inverted > 0)
    {
        why << inverted << " piece(s) with volume <= 0; ";
    }
    if (pinches > 0)
    {
        why << pinches << " edge(s) where the surface pinches (used by more than two faces); ";
    }
    if (internal > 0)
    {
        why << internal << " zero-thickness (INTERNAL/EXTERNAL) face(s); ";
    }
    if (inverted == 0 && internal == 0 && pinches == 0 && BRepCheck_Analyzer(shape).IsValid())
    {
        return "";
    }
    if (inverted == 0 && internal == 0 && pinches == 0)
    {
        why << "fails BRepCheck; ";
    }
    std::string text = why.str();
    return text.substr(0, text.size() - 2);
}

// One general fuse of 'arguments' (each a body: a solid, or a compound of
// solid pieces from an earlier fuse), and its pieces shared out: each piece
// goes to its only source, or for an overlap to the smallest of its sources
// (by 'volumes', the original volumes; ties go to the earlier argument).
// shapes[k] is argument k's new shape: one solid or a compound of pieces, or
// null if less than 1% of its volume is left (remaining[k] says how much).
// Each solid goes into the fuse on its own (the fuse's history doesn't
// follow solids inside a compound argument).
struct FuseResult
{
    bool ok = false;
    std::vector<TopoDS_Shape> shapes;
    std::vector<double> remaining;
    size_t pieces = 0, overlapPieces = 0;
    double overlapVolume = 0.0;
};

FuseResult FuseSolids(const std::vector<TopoDS_Shape>& arguments, const std::vector<double>& volumes, double fuzzy)
{
    FuseResult result;
    ShapeList list;
    std::vector<std::pair<TopoDS_Shape, size_t>> inputs; // each input solid and its argument
    for (size_t a = 0; a < arguments.size(); ++a)
    {
        for (TopExp_Explorer exp(arguments[a], TopAbs_SOLID); exp.More(); exp.Next())
        {
            list.Append(exp.Current());
            inputs.emplace_back(exp.Current(), a);
        }
    }
    BRepAlgoAPI_BuilderAlgo fuse;
    fuse.SetArguments(list);
    fuse.SetRunParallel(true);
    fuse.SetNonDestructive(true);
    fuse.SetUseOBB(true);
    fuse.SetFuzzyValue(fuzzy);
    {
        StageTimer timer("general fuse of " + std::to_string(arguments.size()) + " solids");
        fuse.Build();
    }
    if (fuse.HasErrors() || !fuse.IsDone())
    {
        return result;
    }

    // Which result pieces came from which argument.
    ShapeIndexMap pieces;
    std::vector<std::vector<size_t>> owners; // per piece: argument indices
    auto addPiece = [&](const TopoDS_Shape& piece, size_t argument) {
        for (TopExp_Explorer exp(piece, TopAbs_SOLID); exp.More(); exp.Next())
        {
            const int index = pieces.Add(exp.Current());
            if (owners.size() < static_cast<size_t>(index))
            {
                owners.resize(index);
            }
            if (std::find(owners[index - 1].begin(), owners[index - 1].end(), argument) == owners[index - 1].end())
            {
                owners[index - 1].push_back(argument);
            }
        }
    };
    for (const auto& [solid, a] : inputs)
    {
        const ShapeList& modified = fuse.Modified(solid);
        if (!modified.IsEmpty())
        {
            for (const TopoDS_Shape& piece : modified)
            {
                addPiece(piece, a);
            }
        }
        else if (!fuse.IsDeleted(solid))
        {
            addPiece(solid, a); // untouched: it is its own (only) piece
        }
    }

    std::vector<double> pieceVolume(pieces.Extent(), 0.0);
    OSD_Parallel::For(0, pieces.Extent(), [&](int k) { pieceVolume[k] = SolidVolume(pieces(k + 1)); });

    std::vector<std::vector<int>> assigned(arguments.size());
    for (int k = 0; k < pieces.Extent(); ++k)
    {
        const std::vector<size_t>& from = owners[k];
        size_t owner = from.front();
        for (size_t a : from)
        {
            if (volumes[a] < volumes[owner] || (volumes[a] == volumes[owner] && a < owner))
            {
                owner = a;
            }
        }
        if (from.size() > 1)
        {
            ++result.overlapPieces;
            result.overlapVolume += pieceVolume[k];
        }
        assigned[owner].push_back(k + 1);
    }

    BRep_Builder builder;
    result.shapes.resize(arguments.size());
    result.remaining.assign(arguments.size(), 0.0);
    for (size_t a = 0; a < arguments.size(); ++a)
    {
        for (int k : assigned[a])
        {
            result.remaining[a] += pieceVolume[k - 1];
        }
        if (assigned[a].empty() || result.remaining[a] < 0.01 * volumes[a])
        {
            continue;
        }
        if (assigned[a].size() == 1)
        {
            result.shapes[a] = pieces(assigned[a].front());
            continue;
        }
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        for (int k : assigned[a])
        {
            builder.Add(compound, pieces(k));
        }
        result.shapes[a] = compound;
    }
    result.pieces = static_cast<size_t>(pieces.Extent());
    result.ok = true;
    return result;
}

// Which fused bodies (solid, not cut on the mesh) use each face, by TShape
// and location: who touches whom after the fuse.
struct FaceUsers
{
    ShapeIndexMap faces;
    std::vector<std::set<size_t>> users;

    explicit FaceUsers(const std::vector<Body>& bodies)
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            if (bodies[i].meshCut || bodies[i].shape.IsNull())
            {
                continue;
            }
            for (TopExp_Explorer face(bodies[i].shape, TopAbs_FACE); face.More(); face.Next())
            {
                const int k = faces.Add(face.Current());
                users.resize(faces.Extent());
                users[k - 1].insert(i);
            }
        }
    }
    std::set<size_t> Of(const TopoDS_Shape& face) const
    {
        const int k = faces.FindIndex(face);
        return k > 0 ? users[k - 1] : std::set<size_t>();
    }
    // Bodies sharing a face with body i.
    std::set<size_t> Neighbours(const std::vector<Body>& bodies, size_t i) const
    {
        std::set<size_t> result;
        for (TopExp_Explorer face(bodies[i].shape, TopAbs_FACE); face.More(); face.Next())
        {
            for (size_t other : Of(face.Current()))
            {
                if (other != i)
                {
                    result.insert(other);
                }
            }
        }
        return result;
    }
};
} // namespace

// =============================================================================
// Re-fusing only a neighbourhood. When more bodies have to be left out of the
// fuse after it ran ('cut': broken by the fuse, or with a defective mesh),
// the fuse isn't run again on the whole model. Instead:
//   - each cut body gets its original shape back (meshed on its own, its
//     overlaps cut on the mesh like a thread pair's);
//   - the bodies it shared faces with restart from their original shapes;
//   - the bodies those share faces with (the ring) take part with their
//     current fused shapes, so their contacts with the restarted bodies are
//     made shared again. A ring body's contacts further out are kept: the
//     fuse only splits faces where something touches them. If it does split
//     a face the ring body shares with a body outside (three bodies meeting
//     on one face), that body joins the ring and the local fuse runs again.
// Everything else keeps its shape (and its mesh). Overlaps go to the
// smaller body by original volume, as in the full fuse, so the result is the
// same as fusing everything again without the cut bodies. Returns the names
// of the bodies whose shapes changed (to be meshed again).
// =============================================================================
std::set<std::string> RefuseLocally(std::vector<Body>& bodies, const FuseContext& context,
                                    const std::vector<size_t>& cut)
{
    std::set<std::string> changed;
    const FaceUsers users(bodies);
    const std::set<size_t> cutSet(cut.begin(), cut.end());
    std::set<size_t> restart, ring;
    for (size_t s : cut)
    {
        for (size_t n : users.Neighbours(bodies, s))
        {
            if (!cutSet.count(n))
            {
                restart.insert(n);
            }
        }
    }
    for (size_t c : restart)
    {
        for (size_t n : users.Neighbours(bodies, c))
        {
            if (!restart.count(n) && !cutSet.count(n))
            {
                ring.insert(n);
            }
        }
    }
    for (size_t s : cut)
    {
        bodies[s].meshCut = true;
        bodies[s].shape = context.original.at(bodies[s].name);
        changed.insert(bodies[s].name);
    }
    if (restart.empty())
    {
        return changed;
    }

    for (int expansion = 0; expansion < 5; ++expansion)
    {
        std::vector<size_t> participants(restart.begin(), restart.end());
        participants.insert(participants.end(), ring.begin(), ring.end());
        std::sort(participants.begin(), participants.end());
        std::vector<TopoDS_Shape> arguments;
        std::vector<double> volumes;
        for (size_t i : participants)
        {
            arguments.push_back(restart.count(i) ? context.original.at(bodies[i].name) : bodies[i].shape);
            volumes.push_back(context.volume.at(bodies[i].name));
        }
        std::cout << "  re-fusing the neighbourhood: " << restart.size() << " solid(s) from their original shape, "
                  << ring.size() << " around them as they are\n";
        const FuseResult result = FuseSolids(arguments, volumes, context.fuzzy);
        if (!result.ok)
        {
            std::cout << "  WARNING: the local fuse failed; the neighbouring bodies keep their shapes\n";
            return changed;
        }

        // A ring body must keep every face it shares with a body outside.
        ShapeIndexMap newFaces;
        for (const TopoDS_Shape& shape : result.shapes)
        {
            if (!shape.IsNull())
            {
                TopExp::MapShapes(shape, TopAbs_FACE, newFaces);
            }
        }
        const std::set<size_t> inside(participants.begin(), participants.end());
        std::set<size_t> outsiders;
        for (size_t r : ring)
        {
            for (TopExp_Explorer face(bodies[r].shape, TopAbs_FACE); face.More(); face.Next())
            {
                if (newFaces.Contains(face.Current()))
                {
                    continue;
                }
                for (size_t other : users.Of(face.Current()))
                {
                    if (!inside.count(other) && !cutSet.count(other))
                    {
                        outsiders.insert(other);
                    }
                }
            }
        }
        if (!outsiders.empty())
        {
            ring.insert(outsiders.begin(), outsiders.end());
            continue;
        }

        for (size_t k = 0; k < participants.size(); ++k)
        {
            const size_t i = participants[k];
            if (result.shapes[k].IsNull())
            {
                std::cout << "  WARNING: " << bodies[i].name << " would have almost nothing left; keeping it as it was\n";
                continue;
            }
            bodies[i].shape = result.shapes[k];
            changed.insert(bodies[i].name);
        }
        return changed;
    }
    std::cout << "  WARNING: the neighbourhood to re-fuse kept growing; the neighbouring bodies keep their shapes\n";
    return changed;
}

// For the fused bodies listed in 'check': the ones the fuse broke
// (BrokenFuseResult), and for each, the smaller body of every pair it forms
// with a body sharing its bad faces (itself, if none does). Those are to be
// left out of the fuse.
static std::vector<size_t> BrokenFusePartners(const std::vector<Body>& bodies, const std::vector<size_t>& check)
{
    std::vector<std::string> broken(check.size());
    std::vector<std::vector<TopoDS_Shape>> badFaces(check.size());
    {
        StageTimer timer("checking " + std::to_string(check.size()) + " fused bodies");
        OSD_Parallel::For(0, static_cast<int>(check.size()), [&](int k) {
            broken[k] = BrokenFuseResult(bodies[check[k]].shape, badFaces[k]);
        });
    }
    if (std::all_of(broken.begin(), broken.end(), [](const std::string& b) { return b.empty(); }))
    {
        return {};
    }
    const FaceUsers users(bodies);
    std::set<size_t> result;
    for (size_t k = 0; k < check.size(); ++k)
    {
        if (broken[k].empty())
        {
            continue;
        }
        const size_t b = check[k];
        std::set<size_t> partners;
        for (const TopoDS_Shape& face : badFaces[k])
        {
            for (size_t other : users.Of(face))
            {
                if (other != b)
                {
                    partners.insert(other);
                }
            }
        }
        std::cout << "  the fuse broke " << bodies[b].name << " (" << broken[k] << ")";
        for (size_t other : partners)
        {
            std::cout << (other == *partners.begin() ? "; with " : ", ") << bodies[other].name;
            result.insert(SolidVolume(bodies[other].shape) < SolidVolume(bodies[b].shape) ? other : b);
        }
        if (partners.empty())
        {
            result.insert(b);
        }
        std::cout << "\n";
    }
    return std::vector<size_t>(result.begin(), result.end());
}

// Checks the fused bodies named in 'check' and, while the fuse broke some,
// leaves the smaller body of each broken pair out of it and re-fuses only
// their neighbourhood (RefuseLocally). Returns the names of the bodies whose
// shapes changed.
std::set<std::string> RepairFuse(std::vector<Body>& bodies, const FuseContext& context, std::set<std::string> check)
{
    std::set<std::string> changed;
    for (int pass = 1; pass <= kMaxFusePasses; ++pass)
    {
        std::vector<size_t> toCheck;
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            if (!bodies[i].meshCut && check.count(bodies[i].name))
            {
                toCheck.push_back(i);
            }
        }
        const std::vector<size_t> cut = BrokenFusePartners(bodies, toCheck);
        if (cut.empty())
        {
            return changed;
        }
        if (pass == kMaxFusePasses)
        {
            std::cout << "  WARNING: fused bodies are still broken after " << pass << " passes; their meshes may have"
                      << " holes\n";
            return changed;
        }
        std::cout << "  leaving " << cut.size() << " more solid(s) out of the fuse (their overlaps are cut on the"
                  << " mesh)\n";
        const std::set<std::string> now = RefuseLocally(bodies, context, cut);
        changed.insert(now.begin(), now.end());
        check = now;
    }
    return changed;
}

FuseContext MakeBodiesConformal(std::vector<Body>& bodies, double maxDeviation, bool detectThreadPairs)
{
    FuseContext context;
    std::vector<size_t> solids;
    Bnd_Box box;
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        solids.push_back(i);
        BRepBndLib::Add(bodies[i].shape, box);
    }
    std::cout << "  " << solids.size() << " solid(s) take part\n";
    if (solids.size() < 2)
    {
        return context;
    }

    std::vector<double> bodyVolume(bodies.size(), 0.0);
    {
        StageTimer timer("measuring volumes");
        OSD_Parallel::For(0, static_cast<int>(solids.size()),
                          [&](int k) { bodyVolume[solids[k]] = SolidVolume(bodies[solids[k]].shape); });
    }
    context.fuzzy = 1.0e-6 * std::sqrt(box.SquareExtent());
    for (size_t i : solids)
    {
        context.original[bodies[i].name] = bodies[i].shape;
        context.volume[bodies[i].name] = bodyVolume[i];
    }

    // Thread pairs: leave the smaller body of each out of the fuse. A thread
    // face that runs inside the other solid, no deeper than the mesh
    // accuracy, is what makes slivers too thin to mesh.
    if (detectThreadPairs)
    {
        std::vector<ThreadPair> pairs;
        {
            StageTimer timer("finding thread pairs (freeform faces just inside another solid)");
            std::vector<TopoDS_Shape> shapes;
            for (size_t i : solids)
            {
                shapes.push_back(bodies[i].shape);
            }
            pairs = FindThreadPairs(shapes, maxDeviation);
        }
        for (const ThreadPair& pair : pairs)
        {
            const size_t a = solids[pair.a], b = solids[pair.b];
            bodies[bodyVolume[b] < bodyVolume[a] ? b : a].meshCut = true;
        }
        std::vector<std::string> names;
        for (const Body& body : bodies)
        {
            if (body.meshCut)
            {
                names.push_back(body.name);
            }
        }
        std::cout << "  " << pairs.size() << " thread pair(s); " << names.size()
                  << " solid(s) left out of the fuse (their overlaps are cut on the mesh):";
        for (size_t i = 0; i < names.size() && i < 10; ++i)
        {
            std::cout << " " << names[i];
        }
        std::cout << (names.size() > 10 ? " ..." : "") << "\n";
    }

    solids.erase(std::remove_if(solids.begin(), solids.end(), [&](size_t i) { return bodies[i].meshCut; }),
                 solids.end());
    if (solids.size() < 2)
    {
        return context;
    }
    std::vector<TopoDS_Shape> arguments;
    std::vector<double> volumes;
    for (size_t i : solids)
    {
        arguments.push_back(bodies[i].shape);
        volumes.push_back(bodyVolume[i]);
    }
    const FuseResult result = FuseSolids(arguments, volumes, context.fuzzy);
    if (!result.ok)
    {
        std::cout << "  WARNING: the general fuse failed; bodies are left as modeled (their meshes won't be flush)\n";
        return context;
    }

    // Use the new shapes; drop bodies with ~nothing left.
    std::vector<char> drop(bodies.size(), 0);
    size_t dropped = 0, reshaped = 0;
    for (size_t k = 0; k < solids.size(); ++k)
    {
        const size_t i = solids[k];
        if (result.shapes[k].IsNull())
        {
            drop[i] = 1;
            ++dropped;
            std::cout << "  dropped " << bodies[i].name << ": " << std::setprecision(3)
                      << 100.0 * (1.0 - result.remaining[k] / bodyVolume[i]) << std::setprecision(6)
                      << "% of it overlaps smaller or equal bodies (e.g. the same part placed twice)\n";
            continue;
        }
        if (result.remaining[k] < bodyVolume[i] * (1.0 - 1.0e-6))
        {
            ++reshaped;
        }
        bodies[i].shape = result.shapes[k];
    }
    std::vector<Body> kept;
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (!drop[i])
        {
            kept.push_back(bodies[i]);
        }
    }
    bodies.swap(kept);
    std::cout << "  " << result.pieces << " piece(s); " << result.overlapPieces << " overlap(s) with total volume "
              << result.overlapVolume << " given to the smaller body (" << reshaped << " larger bodies lost volume); "
              << dropped << " body(ies) dropped\n";
    context.ready = true;

    // Check the result, and repair what the fuse broke (see above).
    std::set<std::string> fused;
    for (const Body& body : bodies)
    {
        if (!body.meshCut)
        {
            fused.insert(body.name);
        }
    }
    RepairFuse(bodies, context, fused);
    return context;
}

// =============================================================================
// Last check: every fused solid's final mesh must be a closed 2-manifold
// (each edge used by exactly two triangles), which Geant4 needs. Some
// defects only show up in the mesh: on cosi_test.step, two set screws meet
// tip to tip and the fuse leaves two B-rep edges less than the weld
// tolerance apart, which welding merges into one edge with four triangles.
// For each fused solid with open or pinched edges, its partners are the
// solids touching it there: with a mesh vertex within 'reach' (a tiny
// fraction of the model size, kTouchFraction) of one of the defective edges'
// vertices. Solids merely nearby don't count. The smaller body of each pair
// is returned (the defective body itself if nothing touches it there):
// main() leaves them out of the fuse (their overlaps are cut on the mesh,
// which always gives a closed 2-manifold), re-fuses only their
// neighbourhood and meshes only what changed.
// =============================================================================
static double MeshVolume(const SurfaceMesh& mesh)
{
    double volume = 0.0;
    for (const auto& tri : mesh.triangles)
    {
        const gp_XYZ& a = mesh.vertices[tri[0]].XYZ();
        const gp_XYZ& b = mesh.vertices[tri[1]].XYZ();
        const gp_XYZ& c = mesh.vertices[tri[2]].XYZ();
        volume += a.Dot(b.Crossed(c)) / 6.0;
    }
    return volume;
}

std::vector<size_t> PartnersOfMeshDefects(const std::vector<Body>& bodies, const std::vector<SurfaceMesh>& meshes,
                                          double reach)
{
    std::vector<Bnd_Box> boxes(meshes.size());
    for (size_t i = 0; i < meshes.size(); ++i)
    {
        for (const gp_Pnt& p : meshes[i].vertices)
        {
            boxes[i].Add(p);
        }
    }
    std::set<size_t> result;
    for (size_t d = 0; d < bodies.size(); ++d)
    {
        if (bodies[d].meshCut || meshes[d].triangles.empty())
        {
            continue;
        }
        std::unordered_map<uint64_t, int> uses;
        for (const auto& tri : meshes[d].triangles)
        {
            for (int k = 0; k < 3; ++k)
            {
                const uint64_t a = static_cast<uint32_t>(tri[k]), b = static_cast<uint32_t>(tri[(k + 1) % 3]);
                ++uses[std::min(a, b) << 32 | std::max(a, b)];
            }
        }
        std::vector<gp_Pnt> defects;
        Bnd_Box defectBox;
        for (const auto& entry : uses)
        {
            if (entry.second != 2)
            {
                for (uint64_t v : {entry.first >> 32, entry.first & 0xffffffffu})
                {
                    defects.push_back(meshes[d].vertices[v]);
                    defectBox.Add(meshes[d].vertices[v]);
                }
            }
        }
        if (defects.empty())
        {
            continue;
        }
        defectBox.Enlarge(reach);
        std::vector<size_t> partners;
        for (size_t j = 0; j < bodies.size(); ++j)
        {
            if (j == d || boxes[j].IsOut(defectBox))
            {
                continue;
            }
            const bool close = std::any_of(meshes[j].vertices.begin(), meshes[j].vertices.end(), [&](const gp_Pnt& p) {
                return !defectBox.IsOut(p) && std::any_of(defects.begin(), defects.end(), [&](const gp_Pnt& q) {
                    return p.SquareDistance(q) <= reach * reach;
                });
            });
            if (close)
            {
                partners.push_back(j);
            }
        }
        if (partners.empty())
        {
            result.insert(d);
        }
        std::cout << "  the mesh of " << bodies[d].name << " has " << defects.size() / 2
                  << " open or pinched edge(s) near (" << defects.front().X() << ", " << defects.front().Y() << ", "
                  << defects.front().Z() << ")";
        for (size_t j : partners)
        {
            std::cout << (j == partners.front() ? "; with " : ", ") << bodies[j].name;
            const size_t smaller = MeshVolume(meshes[j]) < MeshVolume(meshes[d]) ? j : d;
            if (!bodies[smaller].meshCut)
            {
                result.insert(smaller);
            }
        }
        std::cout << "\n";
    }
    return std::vector<size_t>(result.begin(), result.end());
}
