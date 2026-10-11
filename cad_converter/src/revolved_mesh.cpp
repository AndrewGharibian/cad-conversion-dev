// =============================================================================
// revolved_mesh.cpp — structured triangulation of full-revolution faces.
//
// WHY
//
// Cylinders, cones and spheres have a simple exact structure: every point is
// (angle around the axis, position along the profile). The simplest mesh
// that stays within a given distance of such a surface follows that
// structure:
//   - a cylinder or cone is straight along its profile, so no points are
//     needed between its boundary circles: one ring of rectangles (a cone
//     ending in an apex becomes a fan of triangles meeting at the apex);
//   - a sphere curves both ways, so it gets evenly spaced rows of latitude
//     between its boundaries, each with the same evenly spaced columns.
// OCCT's mesher doesn't do this. It treats every face as a generic patch in
// (u, v) parameter space, samples the seam edge separately from the
// interior, and so leaves a strip of mismatched triangles along the seam
// (on the test sphere: 51 points on the seam vs 15 in the next column),
// plus interior rows on cones that add nothing.
//
// HOW
//
// For each face that is a complete revolution (U range = 2*pi) of a
// cylinder, cone or sphere, and whose boundary consists only of full circles
// (edges at constant v) and poles (degenerate edges):
//   1. Take each boundary circle's points *from OCCT's existing mesh*
//      (Poly_PolygonOnTriangulation). The neighbouring faces use exactly
//      these points, so the new triangulation joins them without cracks.
//   2. Sort circles and poles by v. For a sphere, add rows of latitude in
//      between, spaced so that the chord error stays under maxDeviation.
//   3. Stitch consecutive rows: matching columns become rectangles (two
//      triangles), a pole becomes a fan, and rows with different point
//      counts are zipped together by angle.
//   4. Store it as the face's new Poly_Triangulation, with new polygons for
//      the boundary edges, so everything downstream reads it as if OCCT had
//      produced it.
// The seam edge is ignored: columns simply wrap from 360 back to 0 degrees.
//
// Triangles are wound counter-clockwise in (u, v), the same convention as
// OCCT's own triangulations (consumers flip them for REVERSED faces).
// =============================================================================
#include "revolved_mesh.hpp"

#include "occt_types.hpp"

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace
{
constexpr double kTwoPi = 6.28318530717958647692;

// One row of the structured mesh: either a ring of points at constant v, or
// a single pole point.
struct Row
{
    double v = 0.0;
    bool pole = false;
    std::vector<double> u;      // ring: angle of each point, ascending in [u0, u0 + 2*pi)
    std::vector<int> nodes;     // ring: new node index of each column
    std::vector<gp_Pnt> points; // ring: 3D points (triangulation frame)
    std::vector<int> oldNodes;  // while collecting: old OCCT node index of each point
    std::vector<std::pair<int, int>> oldNodeColumns; // boundary ring: (old node index, column)
    int poleNode = -1;
    gp_Pnt polePoint;
};

bool RetessellateFace(const TopoDS_Face& originalFace, double maxDeviation, double maxAngle)
{
    // Work on the FORWARD face: the triangulation belongs to the underlying
    // face, independent of how the solid uses it.
    const TopoDS_Face face = TopoDS::Face(originalFace.Oriented(TopAbs_FORWARD));

    TopLoc_Location triLocation;
    Handle(Poly_Triangulation) oldTri = BRep_Tool::Triangulation(face, triLocation);
    if (oldTri.IsNull() || !oldTri->HasUVNodes())
    {
        return false;
    }

    // Surface, in the triangulation's coordinate frame.
    TopLoc_Location surfaceLocation;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face, surfaceLocation);
    const gp_Trsf toTriFrame = (triLocation.Inverted() * surfaceLocation).Transformation();
    auto evaluate = [&](double u, double v) { return surface->Value(u, v).Transformed(toTriFrame); };

    GeomAdaptor_Surface adaptor(surface);
    const GeomAbs_SurfaceType type = adaptor.GetType();
    if (type != GeomAbs_Cylinder && type != GeomAbs_Cone && type != GeomAbs_Sphere)
    {
        return false;
    }

    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!surface->IsUPeriodic() || std::abs((u1 - u0) - kTwoPi) > 1.0e-6)
    {
        return false; // not a full revolution
    }
    const double vTol = 1.0e-7 * std::max(1.0, v1 - v0);
    auto wrapU = [&](double u) {
        double w = std::fmod(u - u0, kTwoPi);
        if (w < 0.0)
        {
            w += kTwoPi;
        }
        if (w > kTwoPi - 1.0e-9)
        {
            w = 0.0; // the seam copy at u0 + 2*pi is the same point as u0
        }
        return u0 + w;
    };

    // ---- 1. Boundary rows from the face's edges ---------------------------
    std::vector<Row> rows;
    std::vector<std::pair<TopoDS_Edge, Handle(Poly_PolygonOnTriangulation)>> boundaryEdges;
    for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
    {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
        if (BRep_Tool::IsClosed(edge, face))
        {
            continue; // the seam: columns just wrap around instead
        }

        if (BRep_Tool::Degenerated(edge))
        {
            double first, last;
            Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
            if (pcurve.IsNull())
            {
                return false;
            }
            Row pole;
            pole.pole = true;
            pole.v = pcurve->Value(0.5 * (first + last)).Y();
            TopoDS_Vertex vertex = TopExp::FirstVertex(edge);
            pole.polePoint = BRep_Tool::Pnt(vertex).Transformed(triLocation.Inverted().Transformation());
            rows.push_back(pole);
            continue;
        }

        const Handle(Poly_PolygonOnTriangulation)& polygon = BRep_Tool::PolygonOnTriangulation(edge, oldTri, triLocation);
        if (polygon.IsNull())
        {
            return false;
        }
        boundaryEdges.push_back({edge, polygon});

        // Every point of a usable boundary edge sits at the same v (a circle
        // of latitude). Anything else (an oblique cut, a hole) -> keep OCCT's mesh.
        const double v = oldTri->UVNode(polygon->Node(1)).Y();
        Row* row = nullptr;
        for (Row& r : rows)
        {
            if (!r.pole && std::abs(r.v - v) <= vTol)
            {
                row = &r;
            }
        }
        if (!row)
        {
            rows.push_back(Row());
            row = &rows.back();
            row->v = v;
        }
        for (int k = 1; k <= polygon->NbNodes(); ++k)
        {
            const int node = polygon->Node(k);
            const gp_Pnt2d uv = oldTri->UVNode(node);
            if (std::abs(uv.Y() - v) > vTol)
            {
                return false;
            }
            row->u.push_back(wrapU(uv.X()));
            row->points.push_back(oldTri->Node(node));
            row->oldNodes.push_back(node);
        }
    }

    // Sort each ring by angle and merge duplicates (a closed circle lists its
    // start point twice; the seam point appears at u0 and u0 + 2*pi).
    for (Row& r : rows)
    {
        if (r.pole)
        {
            continue;
        }
        std::vector<size_t> order(r.u.size());
        for (size_t i = 0; i < order.size(); ++i)
        {
            order[i] = i;
        }
        std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return r.u[a] < r.u[b]; });
        Row sorted;
        sorted.v = r.v;
        for (size_t i : order)
        {
            const bool duplicate = !sorted.u.empty() && std::abs(r.u[i] - sorted.u.back()) < 1.0e-9;
            if (!duplicate)
            {
                sorted.u.push_back(r.u[i]);
                sorted.points.push_back(r.points[i]);
            }
            sorted.oldNodeColumns.push_back({r.oldNodes[i], static_cast<int>(sorted.u.size()) - 1});
        }
        // The last point may be the seam copy of the first (u0 + 2*pi wrapped to u0
        // sorts first, but floating point can leave it last): merge it too.
        if (sorted.u.size() > 1 && sorted.points.back().Distance(sorted.points.front()) < 1.0e-9)
        {
            const int last = static_cast<int>(sorted.u.size()) - 1;
            for (auto& entry : sorted.oldNodeColumns)
            {
                if (entry.second == last)
                {
                    entry.second = 0;
                }
            }
            sorted.u.pop_back();
            sorted.points.pop_back();
        }
        r = sorted;
        if (r.u.size() < 3)
        {
            return false;
        }
    }

    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.v < b.v; });
    if (rows.size() < 2 || std::abs(rows.front().v - v0) > vTol || std::abs(rows.back().v - v1) > vTol)
    {
        return false; // the boundary doesn't close off the face at both ends
    }
    // ---- 2. Rows of latitude in between (spheres only) --------------------
    // A cylinder or cone is a straight line along v: extra rows would add
    // triangles without improving accuracy.
    if (type == GeomAbs_Sphere)
    {
        // How far can grid points be apart? A flat triangle through three
        // points of a sphere (radius R) whose circumscribed circle has radius
        // r dips below the surface by at most R - sqrt(R^2 - r^2). Keeping
        // that under d requires r^2 <= 2Rd - d^2. A grid cell's triangles are
        // roughly right-angled, so r is half the cell's diagonal chord c:
        //   c^2 = a^2 + b^2 <= 8Rd - 4d^2
        // where a and b are the chords between neighbouring columns (widest
        // at the equator) and rows. (Sizing by the edges alone, a^2 <= 8Rd,
        // would let the triangle centres stray about twice as far.)
        const double radius = adaptor.Sphere().Radius();
        const double diagonalSq = std::max(0.0, 8.0 * radius * maxDeviation - 4.0 * maxDeviation * maxDeviation);
        auto chordToAngle = [&](double chord) {
            return std::min(maxAngle, 2.0 * std::asin(std::min(1.0, chord / (2.0 * radius))));
        };

        // Columns: reuse the largest boundary circle's count (so rows line up
        // with it); for a closed sphere, use square cells (a = b).
        size_t columns = 0;
        for (const Row& r : rows)
        {
            columns = std::max(columns, r.u.size());
        }
        double step;
        if (columns == 0)
        {
            step = chordToAngle(std::sqrt(0.5 * diagonalSq));
            columns = std::max<size_t>(3, static_cast<size_t>(std::ceil(kTwoPi / step - 1.0e-9)));
        }
        else
        {
            // The circle fixes a; spend what's left of the budget on b.
            const double a = 2.0 * radius * std::sin(0.5 * kTwoPi / columns);
            const double bSq = diagonalSq - a * a;
            step = chordToAngle(bSq > 0.25 * a * a ? std::sqrt(bSq) : 0.5 * a);
        }

        std::vector<Row> withLatitudes;
        for (size_t i = 0; i < rows.size(); ++i)
        {
            withLatitudes.push_back(rows[i]);
            if (i + 1 == rows.size())
            {
                break;
            }
            const double va = rows[i].v, vb = rows[i + 1].v;
            const int intervals = std::max(1, static_cast<int>(std::ceil((vb - va) / step - 1.0e-9)));
            for (int j = 1; j < intervals; ++j)
            {
                Row latitude;
                latitude.v = va + (vb - va) * j / intervals;
                for (size_t c = 0; c < columns; ++c)
                {
                    double u = u0 + kTwoPi * c / columns;
                    latitude.u.push_back(u);
                    latitude.points.push_back(evaluate(u, latitude.v));
                }
                withLatitudes.push_back(latitude);
            }
        }
        rows.swap(withLatitudes);
    }

    // Two poles in a row would mean a face with no area between them.
    for (size_t i = 0; i + 1 < rows.size(); ++i)
    {
        if (rows[i].pole && rows[i + 1].pole)
        {
            return false;
        }
    }

    // ---- 3. Nodes and triangles -------------------------------------------
    std::vector<gp_Pnt> nodes;
    std::vector<gp_Pnt2d> uvNodes;
    std::map<int, int> oldToNew; // boundary points: old node index -> new node index
    for (Row& r : rows)
    {
        if (r.pole)
        {
            r.poleNode = static_cast<int>(nodes.size());
            nodes.push_back(r.polePoint);
            uvNodes.push_back(gp_Pnt2d(u0 + 0.5 * kTwoPi, r.v));
            continue;
        }
        r.nodes.clear();
        for (size_t c = 0; c < r.u.size(); ++c)
        {
            r.nodes.push_back(static_cast<int>(nodes.size()));
            nodes.push_back(r.points[c]);
            uvNodes.push_back(gp_Pnt2d(r.u[c], r.v));
        }
        for (const auto& entry : r.oldNodeColumns)
        {
            oldToNew[entry.first] = r.nodes[entry.second];
        }
    }

    std::vector<std::array<int, 3>> triangles;
    auto stitch = [&](const Row& lower, const Row& upper) {
        if (lower.pole)
        {
            const size_t n = upper.nodes.size();
            for (size_t i = 0; i < n; ++i)
            {
                triangles.push_back({lower.poleNode, upper.nodes[(i + 1) % n], upper.nodes[i]});
            }
            return;
        }
        if (upper.pole)
        {
            const size_t n = lower.nodes.size();
            for (size_t i = 0; i < n; ++i)
            {
                triangles.push_back({lower.nodes[i], lower.nodes[(i + 1) % n], upper.poleNode});
            }
            return;
        }

        const size_t na = lower.nodes.size(), nb = upper.nodes.size();
        bool aligned = (na == nb);
        for (size_t i = 0; aligned && i < na; ++i)
        {
            aligned = std::abs(lower.u[i] - upper.u[i]) < 1.0e-7;
        }
        if (aligned)
        {
            // Matching columns: each rectangle becomes two triangles.
            for (size_t i = 0; i < na; ++i)
            {
                int a0 = lower.nodes[i], a1 = lower.nodes[(i + 1) % na];
                int b0 = upper.nodes[i], b1 = upper.nodes[(i + 1) % na];
                triangles.push_back({a0, a1, b1});
                triangles.push_back({a0, b1, b0});
            }
            return;
        }

        // Different columns: walk around both rings together by angle.
        auto angle = [](const Row& r, size_t i) {
            const size_t n = r.u.size();
            return r.u[i % n] + kTwoPi * static_cast<double>(i / n);
        };
        size_t i = 0, j = 0;
        while (i < na || j < nb)
        {
            bool advanceLower = (j == nb) || (i < na && angle(lower, i + 1) <= angle(upper, j + 1));
            if (advanceLower)
            {
                triangles.push_back({lower.nodes[i % na], lower.nodes[(i + 1) % na], upper.nodes[j % nb]});
                ++i;
            }
            else
            {
                triangles.push_back({lower.nodes[i % na], upper.nodes[(j + 1) % nb], upper.nodes[j % nb]});
                ++j;
            }
        }
    };
    for (size_t i = 0; i + 1 < rows.size(); ++i)
    {
        stitch(rows[i], rows[i + 1]);
    }

    // ---- 4. Store the new triangulation and boundary polygons -------------
    Handle(Poly_Triangulation) newTri =
        new Poly_Triangulation(static_cast<int>(nodes.size()), static_cast<int>(triangles.size()), true);
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        newTri->SetNode(static_cast<int>(i) + 1, nodes[i]);
        newTri->SetUVNode(static_cast<int>(i) + 1, uvNodes[i]);
    }
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        newTri->SetTriangle(static_cast<int>(i) + 1,
                            Poly_Triangle(triangles[i][0] + 1, triangles[i][1] + 1, triangles[i][2] + 1));
    }

    BRep_Builder builder;
    builder.UpdateFace(face, newTri);
    for (const auto& entry : boundaryEdges)
    {
        const Handle(Poly_PolygonOnTriangulation)& oldPolygon = entry.second;
        NCollection_Array1<int> newNodes(1, oldPolygon->NbNodes());
        for (int k = 1; k <= oldPolygon->NbNodes(); ++k)
        {
            newNodes(k) = oldToNew.at(oldPolygon->Node(k)) + 1;
        }
        Handle(Poly_PolygonOnTriangulation) newPolygon =
            oldPolygon->HasParameters() ? new Poly_PolygonOnTriangulation(newNodes, oldPolygon->Parameters()->Array1())
                                        : new Poly_PolygonOnTriangulation(newNodes);
        builder.UpdateEdge(entry.first, newPolygon, newTri, triLocation);
    }
    return true;
}

} // namespace

int RetessellateRevolvedFaces(const TopoDS_Shape& meshedShape, double maxDeviation, double maxAngleRad)
{
    ShapeIndexMap faces;
    TopExp::MapShapes(meshedShape, TopAbs_FACE, faces);
    int replaced = 0;
    for (int i = 1; i <= faces.Extent(); ++i)
    {
        if (RetessellateFace(TopoDS::Face(faces(i)), maxDeviation, maxAngleRad))
        {
            ++replaced;
        }
    }
    return replaced;
}
