// =============================================================================
// meshing.cpp — triangulate the B-rep with OCCT (BRepMesh_IncrementalMesh),
// with fallbacks for faces the mesher gives up on.
// =============================================================================
#include "meshing.hpp"

#include "boundary_mesh.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeUpgrade_ShapeDivideClosed.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <iostream>
#include <vector>

// =============================================================================
// Mesher fallback 1: restart a failed face's wire at another edge.
//
// Even with its seam at U = 0, a closed cone from Fusion can still come back
// with a null triangulation. OCCT 8.0.1 has cone-specific seam preprocessing
// (BRepMesh_ModelPreProcessor) whose outcome depends on the *order* of the
// edges in the face's boundary wire: the exact same cone meshes perfectly or
// fails depending only on which edge the wire lists first. (Fusion's wire
// starts at the degenerate apex edge; OCCT's own cones start at the base
// circle.) A wire is a closed loop, so starting it at a different edge
// changes nothing about the geometry or topology, only the listing order.
// Rebuilding the face with the loop rotated by one edge and meshing again
// gives exactly the mesh OCCT produces for its own native cone.
// =============================================================================
static TopoDS_Shape RestartWiresOfUnmeshedFaces(const TopoDS_Shape& shape)
{
    Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape;
    BRep_Builder builder;

    for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopLoc_Location location;
        if (!BRep_Tool::Triangulation(face, location).IsNull())
        {
            continue; // this face meshed fine; leave it alone
        }

        // Same surface, no wires yet; then re-add each wire with its edges
        // (and their orientations) listed starting one edge later.
        TopoDS_Face forwardFace = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
        TopoDS_Face newFace = TopoDS::Face(forwardFace.EmptyCopied());
        for (TopoDS_Iterator wireIt(forwardFace); wireIt.More(); wireIt.Next())
        {
            std::vector<TopoDS_Shape> edges;
            for (TopoDS_Iterator edgeIt(wireIt.Value()); edgeIt.More(); edgeIt.Next())
            {
                edges.push_back(edgeIt.Value());
            }
            TopoDS_Wire newWire;
            builder.MakeWire(newWire);
            for (size_t i = 0; i < edges.size(); ++i)
            {
                builder.Add(newWire, edges[(i + 1) % edges.size()]);
            }
            newWire.Orientation(wireIt.Value().Orientation());
            builder.Add(newFace, newWire);
        }
        newFace.Orientation(face.Orientation());
        reshape->Replace(face, newFace);
    }
    return reshape->Apply(shape);
}

// =============================================================================
// Mesher fallback 2 (last resort for a single part): split closed faces.
//
// If no starting edge works, splitting each closed face into two half-faces
// removes its seam altogether, and the fragile seam code path with it.
// Splitting has costs, though, which is why it's the last resort: each cut
// adds a boundary edge where triangles come out more stretched (a visible
// "stitching" line), and on a cone it leaves flat, zero-area triangles
// fanning out from the apex. Because the seam is already at U = 0, the split
// lands at a well-defined 180 degrees.
//
// ShapeUpgrade_ShapeDivideClosed leaves the new edges with stale
// "same parameter"/"same range" flags (BRepCheck_Analyzer then reports the
// shape as invalid), and the mesher can sample a shared edge differently on
// each side, which leaves cracks. BRepLib::SameParameter recomputes the flags
// and tolerances so both half-faces agree on their shared edges again.
// =============================================================================
static TopoDS_Shape SplitClosedFaces(const TopoDS_Shape& shape)
{
    ShapeUpgrade_ShapeDivideClosed divider(shape);
    divider.Perform();
    TopoDS_Shape split = divider.Result();
    BRepLib::SameParameter(split, 1.0e-5, /*forced=*/true);
    return split;
}

// =============================================================================
// Tessellating (meshing) the shape.
//
// BRepMesh_IncrementalMesh computes a triangulation approximating each face's
// exact surface, within the given tolerances, and *attaches* that
// triangulation to the shape (it mutates the TopoDS_Shape's internal data —
// it does not return a new mesh object). Downstream code retrieves the
// triangulation per-face via BRep_Tool::Triangulation().
//
//   deflection        — max distance between the triangle mesh and the true
//                        surface, in model units. Smaller = finer mesh.
//   angularDeflection — max angle between neighbouring triangles along a
//                        curve; keeps small circles from getting too few
//                        points.
//
// The deflection is an absolute length, but it isn't a fixed constant: it is
// the same accuracy limit used for the whole pipeline (by default 1/1000 of
// the part's largest dimension, see DefaultMaxDeviation), so it scales with
// the model whether it's measured in mm or meters. Using the same number
// here as for the structured revolved faces below means OCCT samples the
// circles they share with exactly the accuracy that is asked for.
// =============================================================================
// BRepMesh treats the deflection as a target, not a hard bound: measured
// against the exact surface its triangles overshoot by a few percent (2.9%
// on the test sphere, 1.5% on AssemblyTest.step). Asking for 90% of the
// limit keeps the measured result under it.
constexpr double kOcctDeflectionMargin = 0.9;

static void MeshShape(const TopoDS_Shape& shape, double deflection)
{
    // Constructing BRepMesh_IncrementalMesh with these arguments runs the
    // meshing algorithm immediately (no separate .Perform() call needed).
    // isInParallel: mesh faces on all CPU cores (OSD_Parallel). Each face is
    // triangulated independently, so this is safe and scales with core count.
    BRepMesh_IncrementalMesh mesher(shape, kOcctDeflectionMargin * deflection, /*isRelative=*/false,
                                    kMeshAngleDeg * kPi / 180.0, /*isInParallel=*/true);
}

// Number of faces the mesher gave up on (BRep_Tool::Triangulation is null).
// These would silently vanish from the STL, leaving holes.
int CountUnmeshedFaces(const TopoDS_Shape& shape)
{
    int unmeshed = 0;
    for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        TopLoc_Location location;
        if (BRep_Tool::Triangulation(TopoDS::Face(faceExp.Current()), location).IsNull())
        {
            ++unmeshed;
        }
    }
    return unmeshed;
}

// Mesh only the faces of 'shape' that have no triangulation yet. Running the
// mesher over a whole large model costs time even when it reuses every
// existing triangulation (it rebuilds its data for every face and edge), so
// retries hand it just the faces that still need a mesh. Their boundary
// edges keep the discretization their meshed neighbours already gave them,
// so the new triangles still line up with the neighbours.
static void MeshUnmeshedFaces(const TopoDS_Shape& shape, double deflection)
{
    BRep_Builder builder;
    TopoDS_Compound pending;
    builder.MakeCompound(pending);
    bool any = false;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopLoc_Location location;
        if (BRep_Tool::Triangulation(TopoDS::Face(exp.Current()), location).IsNull())
        {
            builder.Add(pending, exp.Current());
            any = true;
        }
    }
    if (any)
    {
        MeshShape(pending, deflection);
    }
}

// =============================================================================
// Mesh with fallbacks: mesh the shape as-is first (after seam normalization).
// If some faces fail, restart their wires at the next edge and retry (STEP
// 2d), once per edge in the wire; if that never works, split closed faces
// (fallback 2). Returns the shape that actually carries the triangulation,
// which may be a rebuilt version, so later steps (stats, export, collisions)
// must use the returned shape.
// =============================================================================
TopoDS_Shape MeshWithFallback(const TopoDS_Shape& shape, double deflection)
{
    MeshShape(shape, deflection);
    int unmeshed = CountUnmeshedFaces(shape);
    if (unmeshed == 0)
    {
        return shape;
    }
    std::cout << "  " << unmeshed << " face(s) failed to mesh; retrying with their wires restarted at another edge\n";

    // A wire can be restarted at each of its edges; 8 covers typical faces.
    // Only the rebuilt faces lack a triangulation, and only those are meshed
    // again (see MeshUnmeshedFaces).
    TopoDS_Shape best = shape;
    int bestUnmeshed = unmeshed;
    TopoDS_Shape candidate = shape;
    for (int attempt = 1; attempt <= 8; ++attempt)
    {
        candidate = RestartWiresOfUnmeshedFaces(candidate);
        MeshUnmeshedFaces(candidate, deflection);
        const int left = CountUnmeshedFaces(candidate);
        if (left < bestUnmeshed)
        {
            best = candidate;
            bestUnmeshed = left;
        }
        if (left == 0)
        {
            std::cout << "  meshed after restarting the wire " << attempt << " edge(s) later\n";
            return candidate;
        }
    }

    // Last resort, for a single part only: splitting closed faces over a
    // whole assembly would add a cut line to every sphere and cylinder in it.
    int solids = 0;
    for (TopExp_Explorer exp(best, TopAbs_SOLID); exp.More() && solids < 2; exp.Next())
    {
        ++solids;
    }
    if (solids <= 1)
    {
        std::cout << "  still failing; splitting closed faces and retrying\n";
        TopoDS_Shape split = SplitClosedFaces(best);
        MeshUnmeshedFaces(split, deflection);
        if (CountUnmeshedFaces(split) < bestUnmeshed)
        {
            return split;
        }
    }
    // Very last resort: build these faces' triangles from their boundary alone
    // (boundary_mesh.cpp), reusing the points the neighbours' meshes put on the
    // shared edges so everything stays flush.
    const int fromBoundary = TriangulateUnmeshedFacesFromBoundary(best, deflection);
    std::cout << "  triangulated " << fromBoundary << " of " << bestUnmeshed
              << " remaining face(s) from their boundary alone\n";
    return best;
}

// =============================================================================
// The accuracy limit for the whole mesh: the largest allowed distance
// between any triangle and the true surface, in model units.
//
// The default is 1/1000 of the part's largest dimension. For MEGAlib, pick
// it from the detector's needs with --max-deviation. It drives OCCT's mesh
// (MeshShape) and the structured revolved faces (revolved_mesh.cpp); the
// result is then measured against the exact geometry (weld.cpp) and checked
// against it.
// =============================================================================
double DefaultMaxDeviation(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid())
    {
        return 1.0e-3;
    }
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return 1.0e-3 * std::max({xmax - xmin, ymax - ymin, zmax - zmin});
}

// Print a few facts about the first faces without a triangulation, to see
// what kind of face the mesher gives up on.
void DescribeUnmeshedFaces(const std::vector<Body>& bodies, int limit)
{
    int shown = 0;
    for (const Body& body : bodies)
    {
        for (TopExp_Explorer exp(body.shape, TopAbs_FACE); exp.More() && shown < limit; exp.Next())
        {
            const TopoDS_Face& face = TopoDS::Face(exp.Current());
            TopLoc_Location location;
            if (!BRep_Tool::Triangulation(face, location).IsNull())
            {
                continue;
            }
            GProp_GProps props;
            BRepGProp::SurfaceProperties(face, props);
            int wires = 0, edges = 0;
            double maxTolerance = BRep_Tool::Tolerance(face);
            for (TopExp_Explorer w(face, TopAbs_WIRE); w.More(); w.Next())
            {
                ++wires;
            }
            for (TopExp_Explorer e(face, TopAbs_EDGE); e.More(); e.Next())
            {
                ++edges;
                maxTolerance = std::max(maxTolerance, BRep_Tool::Tolerance(TopoDS::Edge(e.Current())));
            }
            BRepAdaptor_Surface surface(face);
            static const char* kTypes[] = {"plane", "cylinder", "cone", "sphere", "torus", "Bezier", "B-spline",
                                           "revolution", "extrusion", "offset", "other"};
            const int type = std::min(static_cast<int>(surface.GetType()), 10);
            std::cout << "    unmeshed face in " << body.name << ": " << kTypes[type] << ", area " << props.Mass()
                      << ", " << wires << " wire(s), " << edges << " edge(s), max tolerance " << maxTolerance << "\n";
            ++shown;
        }
        if (shown >= limit)
        {
            break;
        }
    }
}
