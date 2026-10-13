// =============================================================================
// repair.cpp — check and repair the CAD geometry, and normalize the seams of
// revolved faces.
//
// Checking and repairing:
//
// BRepCheck_Analyzer walks a shape's topology and geometry and checks it
// against B-rep's structural rules (closed wires, consistent orientations,
// curves/surfaces agreeing with their tolerances, etc.): exactly the kind of
// inconsistencies STEP round-trips introduce. ShapeFix_Shape is OCCT's
// general-purpose repair ("shape healing"): it reorders and closes wires,
// closes small gaps, and removes degenerate geometry. It won't sew separate
// faces into a solid (that's BRepBuilderAPI_Sewing's job).
//
// Two things keep this fast on large assemblies:
//
//   1. Every *unique* body is checked once. An assembly places the same part
//      many times (cosi_test.step: 157 parts, 611 placements). Each placement
//      shares the same underlying geometry (TopoDS_TShape) at a different
//      location, so checking the whole assembly would check every copy.
//   2. Bodies are checked in parallel (OSD_Parallel), and ShapeFix only runs
//      on bodies that fail. STEPControl_Reader already runs a round of shape
//      healing during import, so most bodies arrive valid.
//
// A "body" is a solid (step_import.cpp has already removed everything else).
// =============================================================================
#include "repair.hpp"

#include "stage_timer.hpp"
#include "step_import.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepTools.hxx>
#include <BRepTools_Modification.hxx>
#include <BRepTools_Modifier.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_ElementarySurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <OSD_Parallel.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Vec2d.hxx>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

static std::vector<char> CheckBodies(const std::vector<TopoDS_Shape>& bodies)
{
    std::vector<char> valid(bodies.size(), 1);
    OSD_Parallel::For(0, static_cast<int>(bodies.size()),
                      [&](int i) { valid[i] = BRepCheck_Analyzer(bodies[i]).IsValid() ? 1 : 0; });
    return valid;
}

TopoDS_Shape CheckAndRepair(const TopoDS_Shape& shape)
{
    size_t placements = 0;
    std::vector<TopoDS_Shape> bodies = UniqueBodies(shape, &placements);

    std::cout << "  " << bodies.size() << " unique solids (" << placements << " placed)\n";

    std::vector<char> valid;
    {
        StageTimer timer("checking " + std::to_string(bodies.size()) + " unique bodies");
        valid = CheckBodies(bodies);
    }
    const size_t invalid = static_cast<size_t>(std::count(valid.begin(), valid.end(), 0));
    std::cout << "  " << invalid << " of " << bodies.size() << " unique bodies fail the check\n";
    if (invalid == 0)
    {
        return shape;
    }

    StageTimer timer("repairing " + std::to_string(invalid) + " bodies");
    // Replacing the unplaced part updates every placement of it at once.
    // ModeConsiderLocation matches each placement by its part (TShape) and
    // keeps the placement's location; by default ReShape only matches a
    // shape at exactly the location it was registered with.
    Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape;
    reshape->ModeConsiderLocation() = true;
    size_t nowValid = 0;
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (valid[i])
        {
            continue;
        }
        Handle(ShapeFix_Shape) fixer = new ShapeFix_Shape(bodies[i]);
        // SetPrecision/SetMaxTolerance control how aggressively gaps get
        // closed. Defaults are usually fine for mechanical CAD; tune these
        // for very small or very large parts.
        fixer->SetPrecision(1.0e-4);
        fixer->SetMaxTolerance(1.0e-2);
        fixer->Perform();
        if (BRepCheck_Analyzer(fixer->Shape()).IsValid())
        {
            ++nowValid;
        }
        reshape->Replace(bodies[i], fixer->Shape());
    }
    std::cout << "  repaired " << invalid << " bodies: " << nowValid << " now valid, " << (invalid - nowValid)
              << " still invalid\n";
    return reshape->Apply(shape);
}

// =============================================================================
// Move the seam of every revolved face to parameter U = 0.
//
// A face that sweeps a full 360 degrees around an axis — the lateral surface
// of a complete cone, cylinder, or sphere — is "periodic": its parametric
// domain wraps around on itself, joined by a single seam edge. Each such
// surface also has its *own* parametric origin (U = 0), fixed by the
// reference direction stored with the surface in the STEP file.
//
// OCCT's own primitives always put the seam edge exactly at U = 0, so the
// face's U range is [0, 2*pi]. CAD tools don't have to: Fusion 360 writes the
// seam where the revolve's sketch profile was (e.g. the -X side), but gives
// the surface a reference direction pointing the other way (+X). The seam
// then sits at U = pi and the face's range becomes [pi, 3*pi] or [-pi, pi].
// That's perfectly legal B-rep, and BRepCheck_Analyzer calls it valid, but
// it puts the surface's parametric origin in the middle of the face, 180
// degrees away from the modeled seam. Anything that assumes the seam is at
// U = 0 then goes wrong on *both* sides:
//   - at the seam edge itself (0/360 degrees in the CAD model), and
//   - at the surface's parametric origin, on the opposite side (180 degrees).
//
// The fix below rotates each such surface about its own axis so its
// parametric origin lines up with the seam, and shifts every edge's 2D
// curve on that face by the same amount. The 3D geometry doesn't change at
// all; only the numbering of the angle does. BRepTools_Modifier is OCCT's
// standard mechanism for rewriting geometry while keeping topology (shared
// edges, seam edges with their two curves) consistent.
// =============================================================================
namespace
{
class SeamToZeroModification : public BRepTools_Modification
{
public:
    // Faces to rewrite: the rotated surface to use, and how far to shift U.
    struct Rewrite
    {
        Handle(Geom_Surface) surface;
        double uShift;
    };
    std::map<const TopoDS_TShape*, Rewrite> faces;

    bool NewSurface(const TopoDS_Face& face, Handle(Geom_Surface)& surface, TopLoc_Location& location,
                    double& tolerance, bool& reverseWires, bool& reverseFace) override
    {
        auto it = faces.find(face.TShape().get());
        if (it == faces.end())
        {
            return false;
        }
        surface = it->second.surface;
        BRep_Tool::Surface(face, location); // keep the face's placement unchanged
        tolerance = BRep_Tool::Tolerance(face);
        reverseWires = false;
        reverseFace = false;
        return true;
    }

    bool NewCurve2d(const TopoDS_Edge& edge, const TopoDS_Face& face, const TopoDS_Edge&, const TopoDS_Face&,
                    Handle(Geom2d_Curve)& curve, double& tolerance) override
    {
        auto it = faces.find(face.TShape().get());
        if (it == faces.end())
        {
            return false;
        }
        double first, last;
        Handle(Geom2d_Curve) original = BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (original.IsNull())
        {
            return false;
        }
        curve = Handle(Geom2d_Curve)::DownCast(original->Copy());
        curve->Translate(gp_Vec2d(-it->second.uShift, 0.0));
        tolerance = BRep_Tool::Tolerance(edge);
        return true;
    }

    // 3D curves, vertices and edge parameters are untouched.
    bool NewCurve(const TopoDS_Edge&, Handle(Geom_Curve)&, TopLoc_Location&, double&) override { return false; }
    bool NewPoint(const TopoDS_Vertex&, gp_Pnt&, double&) override { return false; }
    bool NewParameter(const TopoDS_Vertex&, const TopoDS_Edge&, double&, double&) override { return false; }

    GeomAbs_Shape Continuity(const TopoDS_Edge& edge, const TopoDS_Face& f1, const TopoDS_Face& f2,
                             const TopoDS_Edge&, const TopoDS_Face&, const TopoDS_Face&) override
    {
        return BRep_Tool::Continuity(edge, f1, f2);
    }
};
} // namespace

TopoDS_Shape NormalizePeriodicSeams(const TopoDS_Shape& shape, bool verbose)
{
    Handle(SeamToZeroModification) modification = new SeamToZeroModification;

    for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopLoc_Location location;
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face, location);
        if (surface.IsNull() || !surface->IsUPeriodic())
        {
            continue;
        }

        // Only surfaces of revolution have a single axis whose rotation shifts
        // U cleanly: cylinders, cones, spheres, tori (all "elementary") and
        // general revolved profiles. A torus's V direction is periodic too;
        // that isn't handled here.
        gp_Ax1 axis;
        if (auto elementary = Handle(Geom_ElementarySurface)::DownCast(surface))
        {
            axis = elementary->Axis();
        }
        else if (auto revolution = Handle(Geom_SurfaceOfRevolution)::DownCast(surface))
        {
            axis = revolution->Axis();
        }
        else
        {
            continue;
        }

        double uMin, uMax, vMin, vMax;
        BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
        if (std::abs(uMin) < 1.0e-9)
        {
            continue; // already starts at U = 0
        }

        // Rotating the surface by uMin makes its new U = 0 land where the old
        // U = uMin was. Whether that's +uMin or -uMin depends on the
        // handedness of the surface's coordinate system, so just try both and
        // keep the one that verifiably maps new(u) onto old(u + uMin).
        const double vMid = 0.5 * (vMin + vMax);
        const double tol = 1.0e-7 * std::max(1.0, surface->Value(uMin, vMid).XYZ().Modulus());
        Handle(Geom_Surface) rotated;
        for (double sign : {1.0, -1.0})
        {
            Handle(Geom_Surface) candidate = Handle(Geom_Surface)::DownCast(surface->Copy());
            candidate->Rotate(axis, sign * uMin);
            if (candidate->Value(0.0, vMid).Distance(surface->Value(uMin, vMid)) < tol
                && candidate->Value(1.0, vMid).Distance(surface->Value(uMin + 1.0, vMid)) < tol)
            {
                rotated = candidate;
                break;
            }
        }
        if (!rotated.IsNull())
        {
            modification->faces[face.TShape().get()] = {rotated, uMin};
        }
    }

    if (modification->faces.empty())
    {
        return shape;
    }

    // Modify each unique part once, unplaced, and put it back at all of its
    // placements. Running the modifier on the assembly itself would copy
    // every placement (it treats each located occurrence as its own shape),
    // so repeated parts would stop sharing geometry and every later stage
    // would mesh and measure each placement separately. One modifier for all
    // parts keeps a face shared by two bodies (a fused contact) shared.
    const std::vector<TopoDS_Shape> parts = UniqueBodies(shape);
    TopoDS_Compound allParts;
    BRep_Builder builder;
    builder.MakeCompound(allParts);
    for (const TopoDS_Shape& part : parts)
    {
        builder.Add(allParts, part);
    }
    BRepTools_Modifier modifier(allParts, modification);
    if (!modifier.IsDone())
    {
        std::cout << "  WARNING: could not normalize periodic seams; meshing the shape as-is\n";
        return shape;
    }
    if (verbose)
    {
        std::cout << "  moved the seam to U=0 on " << modification->faces.size() << " revolved face(s)\n";
    }
    Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape;
    reshape->ModeConsiderLocation() = true; // every placement of the part (see CheckAndRepair)
    for (const TopoDS_Shape& part : parts)
    {
        const TopoDS_Shape& modified = modifier.ModifiedShape(part);
        if (!modified.IsSame(part))
        {
            reshape->Replace(part, modified);
        }
    }
    return reshape->Apply(shape);
}
