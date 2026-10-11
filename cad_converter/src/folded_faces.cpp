// =============================================================================
// folded_faces.cpp — rebuild "folded lunes" on the correct arc.
//
// A lune here is a face on a surface of revolution whose boundary is two
// meridians (iso-U curves, at U = a and U = b) joined only by degenerate
// edges, i.e. points on the axis: a sphere or cone segment between two
// meridians, or a torus elbow whose tube touches the axis (a horn or
// spindle torus, minor radius >= major radius). Its 3D boundary is the same
// for the arc from a to b and for the complementary arc the other way round
// the axis, so only the face's parametric domain says which one is meant.
//
// STEP files usually store no parametric (pcurve) data; the importer projects
// each edge onto the surface, and on a periodic surface a meridian at U = 0
// can land at U = 2*pi just as well. cosi_test.step has four identical 90°
// handle elbows (horn tori, R = r = 0.25 in, written identically in the file);
// the importer builds one of them on the 270° complement with a reversed
// normal. Every edge is still used once in each direction, so BRepCheck
// passes and the mesh is closed, but the handle bends inward: the 270° tube
// folds back over the straight parts of the handle and encloses its outside.
//
// The fold gives it away. Where the elbow meets its neighbour along a
// meridian, a correct face continues away from the neighbour (smoothly, for
// a handle), while the wrong arc leaves the shared edge in the *same*
// direction as the neighbour: the two faces lie on top of each other there,
// which no valid solid can have. So for each lune, this compares the
// direction pointing into the face from the middle of each meridian edge
// with the direction pointing into the neighbour across that edge. If they
// agree (dot product > kFoldDot), the face is rebuilt on the complementary
// arc with the same edges: the meridian at U = b moves to U = b - 2*pi, the
// degenerate edges are rebuilt to span the new arc, the loop is reversed and
// the face orientation flipped (so every shared edge is still used in the
// same direction by the solid). The fix is kept only if the fold is gone.
// =============================================================================
#include "folded_faces.hpp"

#include "occt_types.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepTools.hxx>
#include <BRepTools_ReShape.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>

#include <cmath>
#include <iostream>
#include <set>
#include <vector>

namespace
{
constexpr double kFoldDot = 0.5;     // both faces leave the edge within 60 degrees of each other
constexpr double kParamTol = 1.0e-7; // radians: "this edge lies on U = a"

// Unit vector at the middle of 'edge', tangent to 'face' and perpendicular to
// the edge, pointing into the face. False if it can't be computed.
bool InwardDirection(const TopoDS_Face& face, const TopoDS_Edge& edge, gp_Vec& inward)
{
    const TopoDS_Face forward = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
    for (TopExp_Explorer exp(forward, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& candidate = TopoDS::Edge(exp.Current());
        if (!candidate.IsSame(edge) || BRep_Tool::IsClosed(candidate, forward))
        {
            continue;
        }
        double first, last;
        Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(candidate, forward, first, last);
        if (pcurve.IsNull())
        {
            return false;
        }
        gp_Pnt2d uv;
        gp_Vec2d tangent;
        pcurve->D1(0.5 * (first + last), uv, tangent);
        if (candidate.Orientation() == TopAbs_REVERSED)
        {
            tangent.Reverse();
        }
        // In (u, v), a forward face lies to the left of its edges.
        const gp_Vec2d left(-tangent.Y(), tangent.X());
        BRepAdaptor_Surface surface(forward);
        gp_Pnt p;
        gp_Vec du, dv;
        surface.D1(uv.X(), uv.Y(), p, du, dv);
        inward = du * left.X() + dv * left.Y();
        if (inward.Magnitude() < 1.0e-12)
        {
            return false;
        }
        inward.Normalize();
        return true;
    }
    return false;
}

// One edge of a lune's loop, in loop order on the forward face.
struct LoopEdge
{
    TopoDS_Edge edge; // as oriented in the loop
    bool degenerate = false;
    bool atB = false; // a meridian at U = b (else at U = a)
    Handle(Geom2d_Curve) pcurve;
    double first = 0.0, last = 0.0;
    gp_Pnt2d start, end; // in loop direction
};

// The loop of 'face' if it is a lune (see the top of the file), else empty.
std::vector<LoopEdge> LuneLoop(const TopoDS_Face& face, double& a, double& b, double& period)
{
    const TopoDS_Face forward = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
    Handle(Geom_Surface) surface = BRep_Tool::Surface(forward);
    if (surface.IsNull() || !surface->IsUPeriodic())
    {
        return {};
    }
    period = surface->UPeriod();
    double v0, v1;
    BRepTools::UVBounds(forward, a, b, v0, v1);
    if (b - a > period - kParamTol)
    {
        return {}; // goes all the way round: nothing to choose
    }
    int wires = 0;
    for (TopExp_Explorer exp(forward, TopAbs_WIRE); exp.More(); exp.Next())
    {
        ++wires;
    }
    if (wires != 1)
    {
        return {};
    }

    std::vector<LoopEdge> loop;
    bool seenA = false, seenB = false;
    for (BRepTools_WireExplorer exp(BRepTools::OuterWire(forward), forward); exp.More(); exp.Next())
    {
        LoopEdge e;
        e.edge = exp.Current();
        e.degenerate = BRep_Tool::Degenerated(e.edge);
        e.pcurve = BRep_Tool::CurveOnSurface(e.edge, forward, e.first, e.last);
        if (e.pcurve.IsNull() || BRep_Tool::IsClosed(e.edge, forward))
        {
            return {};
        }
        const bool reversed = e.edge.Orientation() == TopAbs_REVERSED;
        e.start = e.pcurve->Value(reversed ? e.last : e.first);
        e.end = e.pcurve->Value(reversed ? e.first : e.last);
        if (!e.degenerate)
        {
            const double u = e.start.X();
            const double mid = e.pcurve->Value(0.5 * (e.first + e.last)).X();
            if (std::abs(e.end.X() - u) > kParamTol || std::abs(mid - u) > kParamTol)
            {
                return {}; // not a meridian
            }
            if (std::abs(u - a) < kParamTol)
            {
                seenA = true;
            }
            else if (std::abs(u - b) < kParamTol)
            {
                e.atB = true;
                seenB = true;
            }
            else
            {
                return {};
            }
        }
        loop.push_back(e);
    }
    return seenA && seenB ? loop : std::vector<LoopEdge>();
}

// True if 'face' folds back onto a neighbour along one of its meridians.
bool IsFolded(const TopoDS_Face& face, const ShapeAncestorsMap& edgeFaces)
{
    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next())
    {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        if (BRep_Tool::Degenerated(edge) || !edgeFaces.Contains(edge))
        {
            continue;
        }
        for (const TopoDS_Shape& other : edgeFaces.FindFromKey(edge))
        {
            if (other.IsSame(face))
            {
                continue;
            }
            gp_Vec here, there;
            if (InwardDirection(face, edge, here) && InwardDirection(TopoDS::Face(other), edge, there)
                && here.Dot(there) > kFoldDot)
            {
                return true;
            }
        }
    }
    return false;
}

// The same face on the complementary arc, built with the same meridian edges.
// Built in the face's own frame (no location) and FORWARD; the caller flips
// its orientation relative to the original.
TopoDS_Face ComplementaryFace(const TopoDS_Face& face, const std::vector<LoopEdge>& loop, double period)
{
    const TopoDS_Face forward = TopoDS::Face(face.Oriented(TopAbs_FORWARD));
    TopLoc_Location surfaceLocation;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(forward, surfaceLocation);
    const double tolerance = BRep_Tool::Tolerance(forward);
    BRep_Builder builder;
    TopoDS_Face result;
    builder.MakeFace(result, surface, surfaceLocation, tolerance);

    // The new loop runs backwards through the old one, with the meridian at
    // U = b moved to U = b - period, so it encloses [b - period, a].
    const size_t n = loop.size();
    auto shifted = [&](const LoopEdge& e, const gp_Pnt2d& p) {
        return e.atB ? gp_Pnt2d(p.X() - period, p.Y()) : p;
    };
    std::vector<gp_Pnt2d> starts(n), ends(n); // in new loop order
    for (size_t j = 0; j < n; ++j)
    {
        const LoopEdge& e = loop[n - 1 - j];
        if (!e.degenerate)
        {
            starts[j] = shifted(e, e.end);
            ends[j] = shifted(e, e.start);
        }
    }
    for (size_t j = 0; j < n; ++j)
    {
        if (loop[n - 1 - j].degenerate)
        {
            starts[j] = ends[(j + n - 1) % n];
            ends[j] = starts[(j + 1) % n];
        }
    }

    TopoDS_Wire wire;
    builder.MakeWire(wire);
    for (size_t j = 0; j < n; ++j)
    {
        const LoopEdge& e = loop[n - 1 - j];
        if (!e.degenerate)
        {
            Handle(Geom2d_Curve) pcurve = Handle(Geom2d_Curve)::DownCast(e.pcurve->Copy());
            if (e.atB)
            {
                pcurve->Translate(gp_Vec2d(-period, 0.0));
            }
            builder.UpdateEdge(e.edge, pcurve, result, BRep_Tool::Tolerance(e.edge));
            builder.Range(e.edge, result, e.first, e.last);
            builder.Add(wire, e.edge.Reversed());
            continue;
        }
        // A degenerate edge: a new one spanning the new arc in (u, v).
        const gp_Vec2d span(starts[j], ends[j]);
        const double length = span.Magnitude();
        if (length < kParamTol)
        {
            continue;
        }
        TopoDS_Edge degenerate;
        builder.MakeEdge(degenerate);
        builder.UpdateEdge(degenerate, new Geom2d_Line(starts[j], gp_Dir2d(span)), result, tolerance);
        builder.Range(degenerate, 0.0, length);
        builder.Degenerated(degenerate, true);
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(TopoDS::Edge(e.edge.Oriented(TopAbs_FORWARD)), v1, v2);
        builder.Add(degenerate, v1.Oriented(TopAbs_FORWARD));
        builder.Add(degenerate, v2.Oriented(TopAbs_REVERSED));
        builder.Add(wire, degenerate);
    }
    wire.Closed(true);
    builder.Add(result, wire);
    return result;
}
} // namespace

TopoDS_Shape FixFoldedLunes(const TopoDS_Shape& shape, int& fixedFaces)
{
    fixedFaces = 0;
    // Every unique part, unplaced; fixing it fixes all of its placements.
    std::vector<TopoDS_Shape> parts;
    std::set<const TopoDS_TShape*> seen;
    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next())
    {
        if (seen.insert(exp.Current().TShape().get()).second)
        {
            parts.push_back(exp.Current().Located(TopLoc_Location()));
        }
    }
    for (TopExp_Explorer exp(shape, TopAbs_SHELL, TopAbs_SOLID); exp.More(); exp.Next())
    {
        if (seen.insert(exp.Current().TShape().get()).second)
        {
            parts.push_back(exp.Current().Located(TopLoc_Location()));
        }
    }

    Handle(BRepTools_ReShape) reshape = new BRepTools_ReShape;
    reshape->ModeConsiderLocation() = true; // every placement, keeping its location
    for (const TopoDS_Shape& part : parts)
    {
        ShapeAncestorsMap edgeFaces;
        TopExp::MapShapesAndAncestors(part, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        std::set<const TopoDS_TShape*> done;
        for (TopExp_Explorer exp(part, TopAbs_FACE); exp.More(); exp.Next())
        {
            const TopoDS_Face& face = TopoDS::Face(exp.Current());
            if (!done.insert(face.TShape().get()).second)
            {
                continue;
            }
            double a, b, period;
            const std::vector<LoopEdge> loop = LuneLoop(face, a, b, period);
            if (loop.empty() || !IsFolded(face, edgeFaces))
            {
                continue;
            }
            // Rebuild in the face's own frame; flipped orientation keeps every
            // shared edge used in the same direction (the loop is reversed).
            const TopoDS_Face fixedFace = TopoDS::Face(
                ComplementaryFace(TopoDS::Face(face.Located(TopLoc_Location())), loop, period)
                    .Oriented(TopAbs::Reverse(face.Orientation())));

            // Keep it only if it is valid and no longer folded.
            Handle(BRepTools_ReShape) trial = new BRepTools_ReShape;
            trial->ModeConsiderLocation() = true;
            trial->Replace(face, fixedFace.Located(face.Location()));
            const TopoDS_Shape trialPart = trial->Apply(part);
            ShapeAncestorsMap trialEdgeFaces;
            TopExp::MapShapesAndAncestors(trialPart, TopAbs_EDGE, TopAbs_FACE, trialEdgeFaces);
            const TopoDS_Face trialFace = TopoDS::Face(trial->Value(face));
            if (!BRepCheck_Analyzer(fixedFace).IsValid() || IsFolded(trialFace, trialEdgeFaces))
            {
                std::cout << "  WARNING: a face folds back onto its neighbour (wrong arc of a surface of"
                             " revolution), and rebuilding it on the other arc didn't help; left as is\n";
                continue;
            }
            reshape->Replace(face, fixedFace.Located(face.Location()));
            ++fixedFaces;
        }
    }
    return fixedFaces > 0 ? reshape->Apply(shape) : shape;
}
