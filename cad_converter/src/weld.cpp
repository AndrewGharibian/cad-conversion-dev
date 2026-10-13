// =============================================================================
// weld.cpp — weld OCCT's per-face triangulations into one mesh per body, and
// measure it against the exact CAD geometry.
//
// BRepMesh_IncrementalMesh triangulates every B-rep face on its own: each face
// has its own copy of the points on its boundary. Welding merges those copies
// (by rounding positions onto a grid far finer than the mesh), body by body,
// so each body becomes one closed, indexed mesh. Triangles that collapse when
// welded (a pole or apex has a whole row of nodes at one point) are dropped;
// that can't open a hole, because their edges coincide with their neighbours'.
//
// A face can be used by two bodies: after the general fuse (conform.cpp) two
// touching bodies reference one shared face. It is emitted once for each, with
// each body's own orientation, so both meshes contain exactly the same
// triangles there. A face used twice within *one* body separates two of its
// own pieces (a body can be a compound of fused pieces); it is internal, and
// skipped.
//
// Measuring: the largest distance between each triangle and its exact face
// surface, the smallest triangle angle, and open edges (holes) and edges used
// by more than two triangles. Planar faces are skipped (flat triangles on a
// plane are exact), and so are repeated placements of a part (the same
// triangles, moved).
// =============================================================================
#include "weld.hpp"

#include "occt_types.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Extrema_GenLocateExtPS.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom_Surface.hxx>
#include <OSD_Parallel.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Vertex
{
    gp_Pnt p;
    int body = -1; // which input body this vertex belongs to
};

struct Triangle
{
    std::array<int, 3> v; // counter-clockwise seen from outside
    int face = -1;        // B-rep face this triangle lies on
    int body = -1;        // body this triangle belongs to
    bool measure = false; // include in the deviation measurement
    // Surface (u, v) of the corners, from the mesher, unwrapped across the
    // seam of periodic surfaces: the starting guess when measuring.
    std::array<gp_Pnt2d, 3> uv;
    bool hasUV = false;
};

// An undirected mesh edge (a, b), packed into one integer for hashing.
uint64_t EdgeKey(int a, int b)
{
    if (a > b)
    {
        std::swap(a, b);
    }
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
}

std::pair<int, int> EdgeVertices(uint64_t key)
{
    return {static_cast<int>(key >> 32), static_cast<int>(key & 0xffffffffu)};
}

double MinAngleDeg(const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c)
{
    const gp_Pnt* p[3] = {&a, &b, &c};
    double smallest = 180.0;
    for (int k = 0; k < 3; ++k)
    {
        gp_Vec u(*p[k], *p[(k + 1) % 3]);
        gp_Vec w(*p[k], *p[(k + 2) % 3]);
        if (u.Magnitude() <= 0.0 || w.Magnitude() <= 0.0)
        {
            return 0.0;
        }
        smallest = std::min(smallest, u.Angle(w) * 180.0 / kPi);
    }
    return smallest;
}

class Welder
{
public:
    Welder(const std::vector<TopoDS_Shape>& bodies, size_t deviationSampleLimit)
        : bodies_(bodies), sampleLimit_(deviationSampleLimit)
    {
        BRep_Builder builder;
        TopoDS_Compound all;
        builder.MakeCompound(all);
        for (const TopoDS_Shape& body : bodies_)
        {
            builder.Add(all, body);
        }
        shape_ = all;
    }

    bool Build();
    MeshQuality Measure();
    std::vector<SurfaceMesh> Result() const;

private:
    double TriangleDeviation(const Triangle& t, ShapeAnalysis_Surface& surface) const;
    std::unordered_map<uint64_t, std::array<int, 2>> EdgeTriangles() const;
    std::string DescribeFace(int face) const;

    std::vector<TopoDS_Shape> bodies_;
    TopoDS_Shape shape_; // compound of all bodies
    double weldTol_ = 1.0e-9;
    size_t sampleLimit_ = 0;

    ShapeIndexMap faces_;
    std::vector<Handle(ShapeAnalysis_Surface)> surfaces_; // per face
    // Faces whose triangles need their deviation measured: not planar, and
    // the first placement of their part.
    std::vector<char> measureFace_;

    std::vector<Vertex> verts_;
    std::vector<Triangle> tris_;
};

// Largest distance between a triangle and its exact face surface.
//
// The vertices lie on the surface, so the gap is largest somewhere inside
// the triangle. For a sphere or cylinder it peaks at the circumcentre (the
// point equidistant from all three corners), so that is always sampled when
// it lies inside the triangle; a barycentric grid (every 1/6 of the way
// along each side, 28 points including the edge midpoints, which catch
// chords of curved sharp edges) covers other surface shapes.
double Welder::TriangleDeviation(const Triangle& t, ShapeAnalysis_Surface& surface) const
{
    const gp_XYZ& a = verts_[t.v[0]].p.XYZ();
    const gp_XYZ& b = verts_[t.v[1]].p.XYZ();
    const gp_XYZ& c = verts_[t.v[2]].p.XYZ();
    double worst = 0.0;
    // With the corners' (u, v) known, the closest surface point to a sample is
    // found by a few local Newton steps from the interpolated (u, v)
    // (NextValueOfUV); that is far cheaper than ValueOfUV's search over the
    // whole surface, which was the bottleneck on large B-spline models.
    // NextValueOfUV falls back to the full search if the local one fails.
    //
    // On freeform surfaces (B-splines: modeled threads, say) NextValueOfUV's
    // own Newton gives up almost always (on skewed parametrizations, and next
    // to C0 knots), so nearly every sample took the full search: on
    // cosi_test.step, 470 s of CPU for 37k triangles. Extrema_GenLocateExtPS,
    // a local search for the closest point from the same start, succeeds
    // there; it finds the same deviations at 1/60 of the cost. The full
    // search remains the fallback. Analytic surfaces keep NextValueOfUV,
    // which is fast and exact on them.
    const GeomAbs_SurfaceType type = surface.Adaptor3d()->GetType();
    const bool freeform = type != GeomAbs_Plane && type != GeomAbs_Cylinder && type != GeomAbs_Cone &&
                          type != GeomAbs_Sphere && type != GeomAbs_Torus;
    auto sample = [&](const gp_XYZ& point, double wb, double wc) {
        gp_Pnt p(point);
        gp_Pnt2d uv;
        if (t.hasUV)
        {
            gp_XY guess = t.uv[0].XY() * (1.0 - wb - wc) + t.uv[1].XY() * wb + t.uv[2].XY() * wc;
            if (freeform)
            {
                Extrema_GenLocateExtPS locate(*surface.Adaptor3d());
                locate.Perform(p, guess.X(), guess.Y());
                if (locate.IsDone())
                {
                    worst = std::max(worst, std::sqrt(locate.SquareDistance()));
                    return;
                }
            }
            uv = surface.NextValueOfUV(gp_Pnt2d(guess), p, weldTol_);
        }
        else
        {
            uv = surface.ValueOfUV(p, weldTol_);
        }
        worst = std::max(worst, p.Distance(surface.Value(uv)));
    };

    constexpr int kSteps = 6;
    for (int i = 0; i <= kSteps; ++i)
    {
        for (int j = 0; j <= kSteps - i; ++j)
        {
            if ((i == 0 && j == 0) || (i == kSteps) || (j == kSteps))
            {
                continue; // the corners themselves lie on the surface
            }
            double wb = double(i) / kSteps, wc = double(j) / kSteps;
            sample(a * (1.0 - wb - wc) + b * wb + c * wc, wb, wc);
        }
    }

    gp_XYZ ab = b - a, ac = c - a, n = ab.Crossed(ac);
    double n2 = n.SquareModulus();
    if (n2 > 0.0)
    {
        gp_XYZ toCenter = (n.Crossed(ab) * ac.SquareModulus() + ac.Crossed(n) * ab.SquareModulus()) / (2.0 * n2);
        gp_XYZ center = a + toCenter;
        // Barycentric coordinates of the circumcentre: sample it only if inside.
        double wb = (center - a).Crossed(ac).Dot(n) / n2;
        double wc = ab.Crossed(center - a).Dot(n) / n2;
        if (wb >= 0.0 && wc >= 0.0 && wb + wc <= 1.0)
        {
            sample(center, wb, wc);
        }
    }
    return worst;
}

bool Welder::Build()
{
    Bnd_Box box;
    BRepBndLib::Add(shape_, box);
    if (box.IsVoid())
    {
        return false;
    }
    weldTol_ = std::max(1.0e-9, 1.0e-6 * std::sqrt(box.SquareExtent()));

    TopExp::MapShapes(shape_, TopAbs_FACE, faces_);
    for (int i = 1; i <= faces_.Extent(); ++i)
    {
        surfaces_.push_back(new ShapeAnalysis_Surface(BRep_Tool::Surface(TopoDS::Face(faces_(i)))));
    }

    // The body index is part of the weld key, so each body is welded on its
    // own: two bodies that touch keep separate vertices.
    std::map<std::array<long long, 4>, int> weldMap;
    auto weld = [&](int body, const gp_Pnt& p) {
        const std::array<long long, 4> key{body, std::llround(p.X() / weldTol_), std::llround(p.Y() / weldTol_),
                                           std::llround(p.Z() / weldTol_)};
        auto inserted = weldMap.emplace(key, static_cast<int>(verts_.size()));
        if (inserted.second)
        {
            verts_.push_back({p, body});
        }
        return inserted.first->second;
    };

    std::vector<bool> faceSeen(faces_.Extent(), false);
    measureFace_.assign(faces_.Extent(), 0);
    std::set<const TopoDS_TShape*> measuredParts;
    for (size_t b = 0; b < bodies_.size(); ++b)
    {
        std::map<int, int> usesInBody;
        for (TopExp_Explorer faceExp(bodies_[b], TopAbs_FACE); faceExp.More(); faceExp.Next())
        {
            ++usesInBody[faces_.FindIndex(faceExp.Current()) - 1];
        }
        std::set<int> doneInBody;
        for (TopExp_Explorer faceExp(bodies_[b], TopAbs_FACE); faceExp.More(); faceExp.Next())
        {
            const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
            const int f = faces_.FindIndex(face) - 1;
            if (usesInBody[f] > 1 || !doneInBody.insert(f).second)
            {
                continue;
            }
            const bool firstUse = !faceSeen[f];
            faceSeen[f] = true;
            if (firstUse && GeomAdaptor_Surface(BRep_Tool::Surface(face)).GetType() != GeomAbs_Plane
                && measuredParts.insert(face.TShape().get()).second)
            {
                measureFace_[f] = 1;
            }

            TopLoc_Location location;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull())
            {
                continue; // unmeshed face: its body shows up with open edges
            }

            std::vector<int> nodeToVertex(triangulation->NbNodes() + 1, -1);
            for (int n = 1; n <= triangulation->NbNodes(); ++n)
            {
                nodeToVertex[n] = weld(static_cast<int>(b), triangulation->Node(n).Transformed(location));
            }

            const bool reversed = (face.Orientation() == TopAbs_REVERSED);
            const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            for (int t = 1; t <= triangulation->NbTriangles(); ++t)
            {
                int n1, n2, n3;
                triangulation->Triangle(t).Get(n1, n2, n3);
                if (reversed)
                {
                    std::swap(n2, n3);
                }
                Triangle tri;
                tri.v = {nodeToVertex[n1], nodeToVertex[n2], nodeToVertex[n3]};
                tri.face = f;
                tri.body = static_cast<int>(b);
                tri.measure = firstUse && measureFace_[f]; // a shared face is measured once
                if (triangulation->HasUVNodes())
                {
                    tri.uv = {triangulation->UVNode(n1), triangulation->UVNode(n2), triangulation->UVNode(n3)};
                    tri.hasUV = true;
                    for (int k = 1; k < 3; ++k)
                    {
                        if (surface->IsUPeriodic())
                        {
                            const double period = surface->UPeriod();
                            tri.uv[k].SetX(tri.uv[k].X()
                                           - period * std::round((tri.uv[k].X() - tri.uv[0].X()) / period));
                        }
                        if (surface->IsVPeriodic())
                        {
                            const double period = surface->VPeriod();
                            tri.uv[k].SetY(tri.uv[k].Y()
                                           - period * std::round((tri.uv[k].Y() - tri.uv[0].Y()) / period));
                        }
                    }
                }
                if (tri.v[0] != tri.v[1] && tri.v[1] != tri.v[2] && tri.v[0] != tri.v[2])
                {
                    tris_.push_back(tri);
                }
            }
        }
    }
    return !tris_.empty();
}

std::unordered_map<uint64_t, std::array<int, 2>> Welder::EdgeTriangles() const
{
    std::unordered_map<uint64_t, std::array<int, 2>> result;
    result.reserve(tris_.size() * 2);
    for (size_t t = 0; t < tris_.size(); ++t)
    {
        for (int k = 0; k < 3; ++k)
        {
            auto it = result.emplace(EdgeKey(tris_[t].v[k], tris_[t].v[(k + 1) % 3]), std::array<int, 2>{-1, -1}).first;
            if (it->second[0] == -2)
            {
                continue; // already non-manifold (a 4th, 5th... triangle must not reset it)
            }
            if (it->second[0] < 0)
            {
                it->second[0] = static_cast<int>(t);
            }
            else if (it->second[1] < 0)
            {
                it->second[1] = static_cast<int>(t);
            }
            else
            {
                it->second = {-2, -2}; // used by more than two triangles
            }
        }
    }
    return result;
}

MeshQuality Welder::Measure()
{
    MeshQuality q;
    std::vector<char> used(verts_.size(), 0);
    for (const Triangle& t : tris_)
    {
        ++q.triangles;
        for (int v : t.v)
        {
            used[v] = 1;
        }
        double angle = MinAngleDeg(verts_[t.v[0]].p, verts_[t.v[1]].p, verts_[t.v[2]].p);
        q.minAngleDeg = std::min(q.minAngleDeg, angle);
        if (angle < 20.0)
        {
            ++q.trianglesBelow20Deg;
        }
    }
    q.vertices = static_cast<size_t>(std::count(used.begin(), used.end(), 1));

    // Deviation: only triangles on faces that need it (see measureFace_),
    // sampled evenly if there are too many.
    std::vector<std::vector<int>> byFace(faces_.Extent());
    for (const Triangle& t : tris_)
    {
        q.deviationEligible += t.measure ? 1 : 0;
    }
    const size_t stride = (sampleLimit_ > 0 && q.deviationEligible > sampleLimit_)
                              ? (q.deviationEligible + sampleLimit_ - 1) / sampleLimit_
                              : 1;
    size_t eligibleIndex = 0;
    for (size_t t = 0; t < tris_.size(); ++t)
    {
        if (tris_[t].measure && eligibleIndex++ % stride == 0)
        {
            byFace[tris_[t].face].push_back(static_cast<int>(t));
            ++q.deviationMeasured;
        }
    }
    // Measured in parallel, in chunks of ~1000 triangles so that one huge face
    // doesn't end up on a single core. Each chunk makes its own projector for
    // its face: ShapeAnalysis_Surface caches state, so it can't be shared
    // between threads, but it's cheap to create.
    struct Chunk
    {
        int face;
        size_t begin, end;
    };
    std::vector<Chunk> chunks;
    constexpr size_t kChunk = 1000;
    for (size_t f = 0; f < byFace.size(); ++f)
    {
        for (size_t i = 0; i < byFace[f].size(); i += kChunk)
        {
            chunks.push_back({static_cast<int>(f), i, std::min(byFace[f].size(), i + kChunk)});
        }
    }
    std::vector<double> faceMax(chunks.size(), 0.0);
    std::vector<int> worstTri(chunks.size(), -1);
    OSD_Parallel::For(0, static_cast<int>(chunks.size()), [&](int c) {
        const Chunk& chunk = chunks[c];
        Handle(ShapeAnalysis_Surface) projector = new ShapeAnalysis_Surface(surfaces_[chunk.face]->Surface());
        for (size_t i = chunk.begin; i < chunk.end; ++i)
        {
            const double d = TriangleDeviation(tris_[byFace[chunk.face][i]], *projector);
            if (d > faceMax[c])
            {
                faceMax[c] = d;
                worstTri[c] = byFace[chunk.face][i];
            }
        }
    });
    int worst = -1;
    for (size_t c = 0; c < faceMax.size(); ++c)
    {
        if (faceMax[c] > q.maxDeviation)
        {
            q.maxDeviation = faceMax[c];
            worst = worstTri[c];
        }
    }
    if (worst >= 0)
    {
        std::ostringstream note;
        note << "worst deviation " << q.maxDeviation << " in body #" << tris_[worst].body << ", on "
             << DescribeFace(tris_[worst].face);
        q.notes.push_back(note.str());
    }

    q.openEdgesPerBody.assign(bodies_.size(), 0);
    for (const auto& entry : EdgeTriangles())
    {
        if (entry.second[0] == -2)
        {
            ++q.nonManifoldEdges;
        }
        else if (entry.second[1] < 0)
        {
            ++q.openEdges;
            const int body = verts_[EdgeVertices(entry.first).first].body;
            if (q.openEdgesPerBody[body]++ == 0 && q.notes.size() < 8)
            {
                const gp_Pnt& p = verts_[EdgeVertices(entry.first).first].p;
                std::ostringstream note;
                note << "open edge in body #" << body << " at (" << p.X() << ", " << p.Y() << ", " << p.Z()
                     << "), on " << DescribeFace(tris_[entry.second[0]].face);
                q.notes.push_back(note.str());
            }
        }
    }

    // Examples of edges used by more than two triangles, with the faces involved.
    if (q.nonManifoldEdges > 0)
    {
        std::unordered_map<uint64_t, std::vector<int>> uses;
        for (size_t t = 0; t < tris_.size(); ++t)
        {
            for (int k = 0; k < 3; ++k)
            {
                uses[EdgeKey(tris_[t].v[k], tris_[t].v[(k + 1) % 3])].push_back(static_cast<int>(t));
            }
        }
        int shown = 0;
        for (const auto& entry : uses)
        {
            if (entry.second.size() <= 2 || shown >= 3)
            {
                continue;
            }
            ++shown;
            std::ostringstream note;
            note << "edge used by " << entry.second.size() << " triangles in body #" << tris_[entry.second[0]].body
                 << "; faces:";
            std::set<int> faces;
            for (int t : entry.second)
            {
                faces.insert(tris_[t].face);
            }
            for (int f : faces)
            {
                note << " [" << DescribeFace(f) << "]";
            }
            q.notes.push_back(note.str());
        }
    }
    return q;
}

std::string Welder::DescribeFace(int face) const
{
    static const char* kTypes[] = {"plane", "cylinder", "cone", "sphere", "torus", "Bezier", "B-spline",
                                   "revolution", "extrusion", "offset", "other"};
    const TopoDS_Face& f = TopoDS::Face(faces_(face + 1));
    GProp_GProps props;
    BRepGProp::SurfaceProperties(f, props);
    TopLoc_Location location;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(f, location);
    const int type = std::min(static_cast<int>(GeomAdaptor_Surface(BRep_Tool::Surface(f)).GetType()), 10);
    std::ostringstream out;
    out << "face #" << face << " " << kTypes[type] << ", area " << props.Mass() << ", "
        << (tri.IsNull() ? 0 : tri->NbTriangles()) << " tris"
        << (tri.IsNull() || tri->HasUVNodes() ? "" : " (no UV: boundary/plane triangulation)");
    return out.str();
}

std::vector<SurfaceMesh> Welder::Result() const
{
    std::vector<SurfaceMesh> meshes(bodies_.size());
    std::vector<int> newIndex(verts_.size(), -1);
    for (const Triangle& t : tris_)
    {
        SurfaceMesh& mesh = meshes[t.body];
        std::array<int, 3> tri;
        for (int k = 0; k < 3; ++k)
        {
            int v = t.v[k];
            if (newIndex[v] < 0)
            {
                newIndex[v] = static_cast<int>(mesh.vertices.size());
                mesh.vertices.push_back(verts_[v].p);
            }
            tri[k] = newIndex[v];
        }
        mesh.triangles.push_back(tri);
    }
    return meshes;
}
} // namespace

std::vector<SurfaceMesh> WeldAndMeasure(const std::vector<TopoDS_Shape>& bodies, size_t deviationSampleLimit,
                                        MeshQuality& quality)
{
    using Clock = std::chrono::steady_clock;
    auto seconds = [](Clock::time_point since) { return std::chrono::duration<double>(Clock::now() - since).count(); };

    Welder welder(bodies, deviationSampleLimit);
    Clock::time_point start = Clock::now();
    if (!welder.Build())
    {
        return {};
    }
    std::cout << "     weld: " << seconds(start) << " s" << std::endl;

    start = Clock::now();
    quality = welder.Measure();
    std::cout << "     measure: " << seconds(start) << " s" << std::endl;
    return welder.Result();
}
