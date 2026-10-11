// =============================================================================
// boundary_mesh.cpp — last-resort triangulation for faces BRepMesh can't mesh.
//
// WHY
//
// The general fuse (conform.cpp) splits faces wherever solids touch. On
// real assemblies that produces some awkward faces, for example a thin helical
// strip of a screw thread only 0.03 mm wide that winds half a turn. Their
// boundary can't even be drawn as a proper polygon in the face's (u, v)
// parameter space, so BRepMesh gives up on them whatever its settings (on
// cosi_test.step: 86 faces, no setting fixes more than 7). Leaving them out
// would leave a hole in their body.
//
// HOW
//
// Such faces are small, so their boundary alone is a good enough mesh:
//   1. Boundary points: for every edge of the face, take the points the
//      neighbouring face's mesh already put on that edge
//      (Poly_PolygonOnTriangulation), so the new triangles meet the
//      neighbour's exactly and the result stays flush. An edge with no meshed
//      neighbour is sampled directly (GCPnts_TangentialDeflection), once, and
//      reused for every face that shares it.
//   2. A flat 2D picture of the boundary: the face's own (u, v) coordinates if
//      the boundary is a proper polygon there, otherwise the best-fit plane
//      through the boundary points.
//   3. Ear clipping: repeatedly cut off a convex corner ("ear") that contains
//      no other boundary point, until only triangles are left. Holes are first
//      joined to the outer boundary by a bridge edge so it's all one polygon.
//   4. If neither 2D picture is a proper polygon, a fan of triangles from the
//      middle of the boundary still closes the hole.
// The result is stored as the face's triangulation (with boundary polygons
// for its edges), exactly as if BRepMesh had produced it.
// =============================================================================
#include "boundary_mesh.hpp"

#include "occt_types.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

// ---- 2D polygon helpers --------------------------------------------------

double Cross(const gp_XY& o, const gp_XY& a, const gp_XY& b)
{
    return (a.X() - o.X()) * (b.Y() - o.Y()) - (a.Y() - o.Y()) * (b.X() - o.X());
}

double SignedArea(const std::vector<gp_XY>& pts, const std::vector<int>& loop)
{
    double area = 0.0;
    for (size_t i = 0; i < loop.size(); ++i)
    {
        const gp_XY& a = pts[loop[i]];
        const gp_XY& b = pts[loop[(i + 1) % loop.size()]];
        area += a.X() * b.Y() - b.X() * a.Y();
    }
    return 0.5 * area;
}

// Do segments ab and cd cross (properly, or by touching in their interiors)?
bool SegmentsCross(const gp_XY& a, const gp_XY& b, const gp_XY& c, const gp_XY& d)
{
    const double d1 = Cross(c, d, a), d2 = Cross(c, d, b), d3 = Cross(a, b, c), d4 = Cross(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

// True if no two non-adjacent boundary segments cross: the loops form a
// proper polygon (with holes) that ear clipping can work on.
bool IsSimple(const std::vector<gp_XY>& pts, const std::vector<std::vector<int>>& loops)
{
    std::vector<std::pair<int, int>> segments;
    for (const auto& loop : loops)
    {
        if (loop.size() < 3 || std::abs(SignedArea(pts, loop)) <= 0.0)
        {
            return false;
        }
        for (size_t i = 0; i < loop.size(); ++i)
        {
            segments.push_back({loop[i], loop[(i + 1) % loop.size()]});
        }
    }
    for (size_t i = 0; i < segments.size(); ++i)
    {
        for (size_t j = i + 1; j < segments.size(); ++j)
        {
            const auto& s = segments[i];
            const auto& t = segments[j];
            if (s.first == t.first || s.first == t.second || s.second == t.first || s.second == t.second)
            {
                continue; // adjacent
            }
            if (SegmentsCross(pts[s.first], pts[s.second], pts[t.first], pts[t.second]))
            {
                return false;
            }
        }
    }
    return true;
}

bool PointInTriangle(const gp_XY& p, const gp_XY& a, const gp_XY& b, const gp_XY& c)
{
    return Cross(a, b, p) > 0 && Cross(b, c, p) > 0 && Cross(c, a, p) > 0;
}

// Join each hole to the outer loop with a bridge edge (from the hole's
// right-most point to the nearest outer point it can see), giving one
// polygon. The outer loop must be counter-clockwise and the holes clockwise.
std::vector<int> BridgeHoles(const std::vector<gp_XY>& pts, std::vector<int> outer, std::vector<std::vector<int>> holes)
{
    std::sort(holes.begin(), holes.end(), [&](const std::vector<int>& a, const std::vector<int>& b) {
        auto maxX = [&](const std::vector<int>& l) {
            double m = -1e300;
            for (int i : l) m = std::max(m, pts[i].X());
            return m;
        };
        return maxX(a) > maxX(b);
    });
    for (const auto& hole : holes)
    {
        size_t h = 0;
        for (size_t i = 1; i < hole.size(); ++i)
        {
            if (pts[hole[i]].X() > pts[hole[h]].X())
            {
                h = i;
            }
        }
        const gp_XY& H = pts[hole[h]];

        // Candidate outer points, nearest first; take the first visible one.
        std::vector<size_t> order(outer.size());
        for (size_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
            return (pts[outer[a]] - H).SquareModulus() < (pts[outer[b]] - H).SquareModulus();
        });
        size_t chosen = order.front();
        for (size_t candidate : order)
        {
            const gp_XY& V = pts[outer[candidate]];
            bool blocked = false;
            auto check = [&](const std::vector<int>& loop) {
                for (size_t i = 0; i < loop.size() && !blocked; ++i)
                {
                    int a = loop[i], b = loop[(i + 1) % loop.size()];
                    if (a == outer[candidate] || b == outer[candidate] || a == hole[h] || b == hole[h]) continue;
                    blocked = SegmentsCross(H, V, pts[a], pts[b]);
                }
            };
            check(outer);
            for (const auto& other : holes) check(other);
            if (!blocked)
            {
                chosen = candidate;
                break;
            }
        }

        // outer[0..chosen], hole from h all the way round back to h, outer[chosen..]
        std::vector<int> merged(outer.begin(), outer.begin() + chosen + 1);
        for (size_t k = 0; k <= hole.size(); ++k)
        {
            merged.push_back(hole[(h + k) % hole.size()]);
        }
        merged.insert(merged.end(), outer.begin() + chosen, outer.end());
        outer.swap(merged);
    }
    return outer;
}

// Ear clipping of one counter-clockwise polygon (which may pass through the
// same point twice, at a bridge). Returns triangles as indices into pts.
std::vector<std::array<int, 3>> EarClip(const std::vector<gp_XY>& pts, std::vector<int> poly)
{
    std::vector<std::array<int, 3>> tris;
    int guard = static_cast<int>(poly.size()) * static_cast<int>(poly.size()) + 10;
    while (poly.size() > 3 && guard-- > 0)
    {
        const size_t n = poly.size();
        int best = -1;
        double bestCross = -1.0;
        for (size_t i = 0; i < n; ++i)
        {
            const int a = poly[(i + n - 1) % n], b = poly[i], c = poly[(i + 1) % n];
            const double cross = Cross(pts[a], pts[b], pts[c]);
            if (cross <= 0.0)
            {
                continue; // reflex or flat: not an ear
            }
            bool empty = true;
            for (size_t j = 0; j < n && empty; ++j)
            {
                const int p = poly[j];
                if (p == a || p == b || p == c) continue;
                const gp_XY& q = pts[p];
                if ((q - pts[a]).SquareModulus() == 0 || (q - pts[b]).SquareModulus() == 0 ||
                    (q - pts[c]).SquareModulus() == 0)
                    continue; // bridge duplicate of a corner
                empty = !PointInTriangle(q, pts[a], pts[b], pts[c]);
            }
            if (empty)
            {
                best = static_cast<int>(i);
                break;
            }
            if (cross > bestCross)
            {
                bestCross = cross;
            }
        }
        if (best < 0)
        {
            // No clean ear (numerically awkward polygon): cut the most convex
            // corner anyway rather than give up.
            for (size_t i = 0; i < n; ++i)
            {
                const int a = poly[(i + n - 1) % n], b = poly[i], c = poly[(i + 1) % n];
                if (Cross(pts[a], pts[b], pts[c]) == bestCross)
                {
                    best = static_cast<int>(i);
                    break;
                }
            }
            if (best < 0) best = 0;
        }
        const size_t i = static_cast<size_t>(best);
        tris.push_back({poly[(i + n - 1) % n], poly[i], poly[(i + 1) % n]});
        poly.erase(poly.begin() + i);
    }
    if (poly.size() == 3)
    {
        tris.push_back({poly[0], poly[1], poly[2]});
    }
    return tris;
}

// ---- the face ------------------------------------------------------------

class FaceTriangulator
{
public:
    FaceTriangulator(const TopoDS_Shape& model, double deflection) : deflection_(deflection)
    {
        TopExp::MapShapesAndAncestors(model, TopAbs_EDGE, TopAbs_FACE, edgeFaces_);
    }

    bool Triangulate(const TopoDS_Face& face);

private:
    // Points on an edge (global coordinates), in the edge's forward direction.
    const std::vector<gp_Pnt>& EdgePoints(const TopoDS_Edge& edge);

    double deflection_;
    ShapeAncestorsMap edgeFaces_;
    ShapeIndexMap sampledEdges_;
    std::vector<std::vector<gp_Pnt>> samples_;
    std::vector<gp_Pnt> scratch_;
};

const std::vector<gp_Pnt>& FaceTriangulator::EdgePoints(const TopoDS_Edge& anyEdge)
{
    const TopoDS_Edge edge = TopoDS::Edge(anyEdge.Oriented(TopAbs_FORWARD));

    // 1. A neighbouring face that is meshed already sampled this edge.
    if (edgeFaces_.Contains(edge))
    {
        for (const TopoDS_Shape& f : edgeFaces_.FindFromKey(edge))
        {
            TopLoc_Location location;
            const TopoDS_Face& neighbour = TopoDS::Face(f);
            Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(neighbour, location);
            if (tri.IsNull()) continue;
            const Handle(Poly_PolygonOnTriangulation)& polygon = BRep_Tool::PolygonOnTriangulation(edge, tri, location);
            if (polygon.IsNull()) continue;
            scratch_.clear();
            for (int k = 1; k <= polygon->NbNodes(); ++k)
            {
                scratch_.push_back(tri->Node(polygon->Node(k)).Transformed(location));
            }
            return scratch_;
        }
    }

    // 2. Otherwise sample it ourselves, once, so every face sharing it agrees.
    int index = sampledEdges_.FindIndex(edge);
    if (index == 0)
    {
        std::vector<gp_Pnt> points;
        if (BRep_Tool::Degenerated(edge))
        {
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(edge, v1, v2);
            points = {BRep_Tool::Pnt(v1), BRep_Tool::Pnt(v2)};
        }
        else
        {
            BRepAdaptor_Curve curve(edge);
            GCPnts_TangentialDeflection sampler(curve, 20.0 * kPi / 180.0, deflection_);
            for (int k = 1; k <= sampler.NbPoints(); ++k)
            {
                points.push_back(sampler.Value(k));
            }
            if (points.size() < 2)
            {
                points = {curve.Value(curve.FirstParameter()), curve.Value(curve.LastParameter())};
            }
        }
        index = sampledEdges_.Add(edge);
        samples_.push_back(points);
    }
    return samples_[index - 1];
}

bool FaceTriangulator::Triangulate(const TopoDS_Face& originalFace)
{
    const TopoDS_Face face = TopoDS::Face(originalFace.Oriented(TopAbs_FORWARD));
    const TopLoc_Location faceLocation = face.Location();
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face); // global coordinates

    // ---- 1. Boundary loops (global 3D points) and each edge's node indices.
    std::vector<gp_Pnt> points;
    std::vector<std::vector<int>> loops;
    struct EdgeNodes
    {
        TopoDS_Edge edge;
        std::vector<int> nodes; // in the edge's forward direction
    };
    std::vector<EdgeNodes> edgeNodes;
    for (TopExp_Explorer we(face, TopAbs_WIRE); we.More(); we.Next())
    {
        std::vector<int> loop;
        for (BRepTools_WireExplorer ex(TopoDS::Wire(we.Current()), face); ex.More(); ex.Next())
        {
            const TopoDS_Edge& edge = ex.Current();
            std::vector<gp_Pnt> pts = EdgePoints(edge);
            const bool reversed = edge.Orientation() == TopAbs_REVERSED;
            if (reversed)
            {
                std::reverse(pts.begin(), pts.end());
            }
            std::vector<int> ids;
            for (size_t k = 0; k < pts.size(); ++k)
            {
                // Consecutive edges share their end point: reuse it.
                if (k == 0 && !loop.empty() && points[loop.back()].Distance(pts[0]) <= 1.0e-9 * (1.0 + pts[0].XYZ().Modulus()))
                {
                    ids.push_back(loop.back());
                    continue;
                }
                points.push_back(pts[k]);
                loop.push_back(static_cast<int>(points.size()) - 1);
                ids.push_back(loop.back());
            }
            if (!BRep_Tool::Degenerated(edge) && !BRep_Tool::IsClosed(edge, face))
            {
                if (reversed)
                {
                    std::reverse(ids.begin(), ids.end());
                }
                edgeNodes.push_back({TopoDS::Edge(edge.Oriented(TopAbs_FORWARD)), ids});
            }
        }
        // The loop's last point is its first point again.
        if (loop.size() > 1 && points[loop.back()].Distance(points[loop.front()]) <= 1.0e-9 * (1.0 + points[loop.front()].XYZ().Modulus()))
        {
            const int last = loop.back();
            loop.pop_back();
            for (auto& en : edgeNodes)
                for (int& id : en.nodes)
                    if (id == last) id = loop.front();
        }
        if (loop.size() >= 3)
        {
            loops.push_back(loop);
        }
    }
    if (loops.empty())
    {
        return false;
    }

    // ---- 2. A flat picture of the boundary.
    Handle(ShapeAnalysis_Surface) projector = new ShapeAnalysis_Surface(surface);
    std::vector<gp_XY> uv(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        uv[i] = projector->ValueOfUV(points[i], 1.0e-7).XY();
    }

    // Best-fit plane (through the centroid, normal = direction of least
    // spread), used if the (u, v) picture isn't a proper polygon.
    gp_XYZ centroid(0, 0, 0);
    for (const gp_Pnt& p : points) centroid += p.XYZ();
    centroid /= static_cast<double>(points.size());
    gp_Vec normal(0, 0, 0);
    for (const auto& loop : loops) // Newell's method: robust polygon normal
    {
        for (size_t i = 0; i < loop.size(); ++i)
        {
            const gp_XYZ& a = points[loop[i]].XYZ();
            const gp_XYZ& b = points[loop[(i + 1) % loop.size()]].XYZ();
            normal += gp_Vec((a.Y() - b.Y()) * (a.Z() + b.Z()), (a.Z() - b.Z()) * (a.X() + b.X()),
                             (a.X() - b.X()) * (a.Y() + b.Y()));
        }
    }
    std::vector<gp_XY> planar(points.size());
    bool havePlane = normal.Magnitude() > 0.0;
    if (havePlane)
    {
        normal.Normalize();
        gp_Vec x = std::abs(normal.X()) < 0.9 ? gp_Vec(1, 0, 0).Crossed(normal) : gp_Vec(0, 1, 0).Crossed(normal);
        x.Normalize();
        gp_Vec y = normal.Crossed(x);
        for (size_t i = 0; i < points.size(); ++i)
        {
            gp_Vec d(centroid, points[i].XYZ());
            planar[i] = gp_XY(d.Dot(x), d.Dot(y));
        }
    }

    // The face's normal direction (forward orientation) at the boundary, to
    // wind the triangles the right way whichever picture is used.
    gp_Vec faceNormal(0, 0, 0);
    {
        GeomLProp_SLProps props(surface, uv[loops[0][0]].X(), uv[loops[0][0]].Y(), 1, 1.0e-9);
        if (props.IsNormalDefined()) faceNormal = gp_Vec(props.Normal());
    }

    std::vector<std::array<int, 3>> tris;
    bool usedUV = false;
    auto triangulateIn = [&](const std::vector<gp_XY>& chart) -> bool {
        if (!IsSimple(chart, loops)) return false;
        // Outer loop = largest area; make it counter-clockwise, holes clockwise.
        size_t outerIndex = 0;
        for (size_t i = 1; i < loops.size(); ++i)
            if (std::abs(SignedArea(chart, loops[i])) > std::abs(SignedArea(chart, loops[outerIndex]))) outerIndex = i;
        std::vector<int> outer = loops[outerIndex];
        if (SignedArea(chart, outer) < 0) std::reverse(outer.begin(), outer.end());
        std::vector<std::vector<int>> holes;
        for (size_t i = 0; i < loops.size(); ++i)
        {
            if (i == outerIndex) continue;
            std::vector<int> hole = loops[i];
            if (SignedArea(chart, hole) > 0) std::reverse(hole.begin(), hole.end());
            holes.push_back(hole);
        }
        tris = EarClip(chart, BridgeHoles(chart, outer, holes));
        return !tris.empty();
    };
    if (triangulateIn(uv))
    {
        usedUV = true;
    }
    else if (!(havePlane && triangulateIn(planar)))
    {
        // ---- 4. Fan from the middle of the (outer) boundary.
        if (loops.size() != 1) return false;
        gp_Pnt2d mid = projector->ValueOfUV(gp_Pnt(centroid), 1.0e-7);
        points.push_back(surface->Value(mid.X(), mid.Y()));
        uv.push_back(mid.XY());
        const int m = static_cast<int>(points.size()) - 1;
        const auto& loop = loops[0];
        for (size_t i = 0; i < loop.size(); ++i) tris.push_back({m, loop[i], loop[(i + 1) % loop.size()]});
    }

    // Wind the triangles so they face the same way as the face's surface.
    if (faceNormal.Magnitude() > 0.0)
    {
        double agreement = 0.0;
        for (const auto& t : tris)
        {
            agreement += gp_Vec(points[t[0]], points[t[1]]).Crossed(gp_Vec(points[t[0]], points[t[2]])).Dot(faceNormal);
        }
        if (agreement < 0.0)
        {
            for (auto& t : tris) std::swap(t[1], t[2]);
        }
    }

    // ---- Store: nodes in the face's own coordinate frame.
    Handle(Poly_Triangulation) triangulation =
        new Poly_Triangulation(static_cast<int>(points.size()), static_cast<int>(tris.size()), usedUV);
    const gp_Trsf toFace = faceLocation.Inverted().Transformation();
    for (size_t i = 0; i < points.size(); ++i)
    {
        triangulation->SetNode(static_cast<int>(i) + 1, points[i].Transformed(toFace));
        if (usedUV) triangulation->SetUVNode(static_cast<int>(i) + 1, gp_Pnt2d(uv[i]));
    }
    for (size_t i = 0; i < tris.size(); ++i)
    {
        triangulation->SetTriangle(static_cast<int>(i) + 1, Poly_Triangle(tris[i][0] + 1, tris[i][1] + 1, tris[i][2] + 1));
    }
    BRep_Builder builder;
    builder.UpdateFace(face, triangulation);
    for (const auto& en : edgeNodes)
    {
        NCollection_Array1<int> nodes(1, static_cast<int>(en.nodes.size()));
        for (size_t k = 0; k < en.nodes.size(); ++k) nodes(static_cast<int>(k) + 1) = en.nodes[k] + 1;
        builder.UpdateEdge(en.edge, new Poly_PolygonOnTriangulation(nodes), triangulation, faceLocation);
    }
    return true;
}

} // namespace

int TriangulateUnmeshedFacesFromBoundary(const TopoDS_Shape& model, double deflection)
{
    FaceTriangulator triangulator(model, deflection);
    ShapeIndexMap done;
    int count = 0;
    for (TopExp_Explorer exp(model, TopAbs_FACE); exp.More(); exp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location location;
        if (!BRep_Tool::Triangulation(face, location).IsNull() || done.Contains(face))
        {
            continue;
        }
        done.Add(face);
        if (triangulator.Triangulate(face))
        {
            ++count;
        }
    }
    return count;
}
