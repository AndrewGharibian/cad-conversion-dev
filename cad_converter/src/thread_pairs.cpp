// =============================================================================
// thread_pairs.cpp â€” find the solid pairs the general fuse turns into slivers.
//
// Why: a fully modeled screw thread in a fully modeled tapped hole has two
// helical surfaces that are meant to lie on top of each other. They don't
// quite: the screw and the hole are modeled separately (clearance, profile,
// phase), and CAD stores a helicoid as a B-spline that only approximates the
// true surface, so each side wobbles around it independently. The surfaces
// cross each other again and again, a few hundredths of a millimetre apart.
// The general fuse computes all of those crossings exactly, which gives
// helical sliver faces far thinner than any triangle the mesher makes, and
// near-tangent intersection curves the neighbouring faces disagree on. The
// result is holes, edges shared by more than two triangles and inside-out
// triangles in the mesh.
//
// What is detected: a pair of solids where a *freeform* face (B-spline,
// Bezier, offset, swept, ...) of one solid runs inside the other solid, no
// deeper than 'depthLimit', over a substantial part of the face. That is
// what "two surfaces meant to coincide, slightly overlapping" looks like.
// Analytic faces (planes, cylinders, cones, spheres, tori) are not tested
// as the face that runs inside: the fuse handles them exactly, including
// coincident ones (e.g. a shaft in a bore).
//
// How, per pair of solids whose bounding boxes overlap:
//   1. Sample each freeform face of one solid that lies in the other's box on
//      a regular (u, v) grid, keeping the points inside the face's trimming.
//   2. For each point, find the distance to the other solid's faces nearby
//      (only faces whose box, grown by depthLimit, contains the point).
//   3. If that distance is at most depthLimit, classify the point against
//      the other solid; points strictly inside count as "near".
//   4. The pair is a thread pair if some face has at least kMinNear near
//      points and at least kMinNearFraction of its samples are near. A face
//      that merely cuts through the other solid only has near points along
//      the intersection line, a small fraction of its samples.
// Both directions are tested (a's faces in b, b's faces in a).
// =============================================================================
#include "thread_pairs.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_ExtPF.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <Bnd_Box.hxx>
#include <OSD_Parallel.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <iterator>
#include <limits>
#include <memory>

namespace
{
constexpr int kGrid = 8;                 // samples per face: kGrid x kGrid in (u, v)
constexpr size_t kMinNear = 4;           // at least this many near points on one face...
constexpr double kMinNearFraction = 0.2; // ...and at least this fraction of its samples

struct FaceInfo
{
    TopoDS_Face face;
    Bnd_Box box;
    bool freeform = false;
};

struct SolidInfo
{
    Bnd_Box box;
    std::vector<FaceInfo> faces;
};

bool IsFreeform(const TopoDS_Face& face)
{
    switch (BRepAdaptor_Surface(face, false).GetType())
    {
    case GeomAbs_Plane:
    case GeomAbs_Cylinder:
    case GeomAbs_Cone:
    case GeomAbs_Sphere:
    case GeomAbs_Torus: return false;
    default: return true;
    }
}

// Points of 'face' on a kGrid x kGrid (u, v) grid that lie inside its trimming.
std::vector<gp_Pnt> SampleFace(const TopoDS_Face& face)
{
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    BRepAdaptor_Surface surface(face);
    BRepTopAdaptor_FClass2d inside(face, Precision::PConfusion());
    std::vector<gp_Pnt> points;
    for (int i = 0; i < kGrid; ++i)
    {
        const double u = u0 + (u1 - u0) * (i + 0.5) / kGrid;
        for (int j = 0; j < kGrid; ++j)
        {
            const double v = v0 + (v1 - v0) * (j + 0.5) / kGrid;
            if (inside.Perform(gp_Pnt2d(u, v)) == TopAbs_IN)
            {
                points.push_back(surface.Value(u, v));
            }
        }
    }
    return points;
}

// Tests the freeform faces of 'from' against the solid 'into'. Returns the
// best face's (near, samples) counts, or (0, 0) if no face qualifies.
std::pair<size_t, size_t> NearSamples(const SolidInfo& from, const SolidInfo& intoInfo, const TopoDS_Shape& into,
                                      double depthLimit)
{
    std::unique_ptr<BRepClass3d_SolidClassifier> classifier;
    std::vector<std::unique_ptr<BRepExtrema_ExtPF>> distance(intoInfo.faces.size());
    std::pair<size_t, size_t> best(0, 0);
    for (const FaceInfo& f : from.faces)
    {
        if (!f.freeform || f.box.IsOut(intoInfo.box))
        {
            continue;
        }
        const std::vector<gp_Pnt> points = SampleFace(f.face);
        size_t inside = 0;
        for (const gp_Pnt& p : points)
        {
            if (intoInfo.box.IsOut(p))
            {
                continue;
            }
            const TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(p).Vertex();
            double nearest = std::numeric_limits<double>::max();
            for (size_t g = 0; g < intoInfo.faces.size(); ++g)
            {
                if (intoInfo.faces[g].box.IsOut(p))
                {
                    continue;
                }
                if (!distance[g])
                {
                    distance[g] = std::make_unique<BRepExtrema_ExtPF>();
                    distance[g]->Initialize(intoInfo.faces[g].face, Extrema_ExtFlag_MIN);
                }
                distance[g]->Perform(vertex, intoInfo.faces[g].face);
                if (distance[g]->IsDone())
                {
                    for (int k = 1; k <= distance[g]->NbExt(); ++k)
                    {
                        nearest = std::min(nearest, distance[g]->SquareDistance(k));
                    }
                }
            }
            if (nearest > depthLimit * depthLimit)
            {
                continue;
            }
            if (!classifier)
            {
                classifier = std::make_unique<BRepClass3d_SolidClassifier>(into);
            }
            classifier->Perform(p, Precision::Confusion());
            if (classifier->State() == TopAbs_IN)
            {
                ++inside;
            }
        }
        if (inside >= kMinNear && inside >= kMinNearFraction * points.size() && inside > best.first)
        {
            best = {inside, points.size()};
        }
    }
    return best;
}
} // namespace

std::vector<ThreadPair> FindThreadPairs(const std::vector<TopoDS_Shape>& solids, double depthLimit)
{
    // Boxes of every solid and face. Face boxes are grown by depthLimit, so
    // "the point's box test passes" means "the face may be within depthLimit".
    std::vector<SolidInfo> info(solids.size());
    OSD_Parallel::For(0, static_cast<int>(solids.size()), [&](int i) {
        BRepBndLib::Add(solids[i], info[i].box, false);
        info[i].box.Enlarge(depthLimit);
        for (TopExp_Explorer exp(solids[i], TopAbs_FACE); exp.More(); exp.Next())
        {
            FaceInfo f;
            f.face = TopoDS::Face(exp.Current());
            BRepBndLib::Add(f.face, f.box, false);
            f.box.Enlarge(depthLimit);
            f.freeform = IsFreeform(f.face);
            info[i].faces.push_back(f);
        }
    });

    // Candidate pairs: overlapping boxes, and a freeform face of one in the
    // other's box.
    auto hasFreeformIn = [&](const SolidInfo& from, const SolidInfo& into) {
        return std::any_of(from.faces.begin(), from.faces.end(),
                           [&](const FaceInfo& f) { return f.freeform && !f.box.IsOut(into.box); });
    };
    std::vector<ThreadPair> candidates;
    for (size_t a = 0; a < solids.size(); ++a)
    {
        for (size_t b = a + 1; b < solids.size(); ++b)
        {
            if (!info[a].box.IsOut(info[b].box) &&
                (hasFreeformIn(info[a], info[b]) || hasFreeformIn(info[b], info[a])))
            {
                candidates.push_back({a, b, 0, 0});
            }
        }
    }

    OSD_Parallel::For(0, static_cast<int>(candidates.size()), [&](int k) {
        ThreadPair& pair = candidates[k];
        std::pair<size_t, size_t> ab = NearSamples(info[pair.a], info[pair.b], solids[pair.b], depthLimit);
        std::pair<size_t, size_t> ba = NearSamples(info[pair.b], info[pair.a], solids[pair.a], depthLimit);
        const std::pair<size_t, size_t>& best = ab.first >= ba.first ? ab : ba;
        pair.nearSamples = best.first;
        pair.samples = best.second;
    });

    std::vector<ThreadPair> pairs;
    std::copy_if(candidates.begin(), candidates.end(), std::back_inserter(pairs),
                 [](const ThreadPair& p) { return p.nearSamples > 0; });
    return pairs;
}
