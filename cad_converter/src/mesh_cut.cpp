// =============================================================================
// mesh_cut.cpp — cut every overlap between body meshes.
//
// Geant4 needs distinct volumes not to overlap at all. The general fuse
// resolves most overlaps before meshing, but not all:
//   - bodies left out of the fuse (thread pairs, see thread_pairs.cpp, and
//     pairs the fuse broke) still overlap their neighbours;
//   - the fuse can miss an interference (on cosi_test.step, a centering ring
//     and two feedthroughs, 0.37 mm^3 each, in the full fuse only);
//   - even a correctly fused pair can overlap once meshed: on a concave
//     surface the flat triangles bulge outward, up to the mesh accuracy, into
//     a neighbour touching there.
// So every pair of solid meshes is checked, and every overlap is removed here,
// on the finished meshes, with the Manifold
// library (https://github.com/elalish/manifold, Apache-2.0). Manifold's
// booleans are guaranteed to return a closed, oriented 2-manifold for any
// closed input, however close or tangent the surfaces are, which is exactly
// what the B-rep fuse can't promise for near-coincident thread surfaces.
//
// The rule is the fuse's: an overlap goes to the smaller body, and the
// larger body loses it. So each body that overlaps smaller ones becomes
//     body - (every smaller overlapping body, as originally meshed).
// Using the original meshes of the smaller bodies keeps it consistent when
// three bodies overlap in one place: the smallest keeps the region, and both
// larger ones lose it.
//
// Where the larger body was cut, its new surface is made of the smaller
// body's triangles, so the two are flush *inside the overlap*. Where the two
// meshes only nearly touch (the thread flanks alternate between overlapping
// and gapping), a gap remains, no wider than the original surfaces' gap. A
// gap is harmless for particle tracking (it is a sliver of the surrounding
// material); an overlap is not.
//
// Triangles away from the overlap are kept as they are, so a cut body's
// faces shared with fused neighbours stay flush with them.
//
// Repeated parts: the bodies left out of the fuse are mostly screws, and an
// assembly places the same screw many times (cosi_test.step: 12 identical
// screws in one plate). Each placement is cut with its own mesh, so the cut
// surfaces of the larger bodies are made of exactly the vertices that
// placement exports. Where several placements of one part cut the same
// larger body, they are merged into one cutter (a single mesh), which is
// subtracted once.
// =============================================================================
#include "mesh_cut.hpp"

#include <manifold/manifold.h>

#include <OSD_Parallel.hxx>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_map>

namespace
{
// An overlap smaller than this fraction of the smaller body's volume is
// numerical noise (e.g. two coplanar faces in contact), not an overlap. It is
// tiny on purpose: Geant4 needs distinct volumes not to overlap at all.
constexpr double kMinOverlapFraction = 1.0e-12;
// A body left with less than this fraction of its volume is dropped, like a
// duplicate placement in the fuse.
constexpr double kMinRemainingFraction = 0.01;
// Rounds of checking and cutting (see CutOverlappingMeshes); the last one
// only checks.
constexpr int kMaxRounds = 4;
// 'originalID' (from Manifold::ReserveIDs) names the mesh. Manifold sorts a
// result's triangles by the IDs of the meshes they came from, so IDs given
// in a fixed order keep the output the same however the work is scheduled.
manifold::MeshGL64 ToManifoldMesh(const SurfaceMesh& mesh, uint32_t originalID)
{
    manifold::MeshGL64 gl;
    gl.numProp = 3;
    gl.runOriginalID = {originalID};
    gl.vertProperties.reserve(3 * mesh.vertices.size());
    for (const gp_Pnt& p : mesh.vertices)
    {
        gl.vertProperties.insert(gl.vertProperties.end(), {p.X(), p.Y(), p.Z()});
    }
    gl.triVerts.reserve(3 * mesh.triangles.size());
    for (const auto& tri : mesh.triangles)
    {
        for (int corner : tri)
        {
            gl.triVerts.push_back(static_cast<uint64_t>(corner));
        }
    }
    return gl;
}

SurfaceMesh FromManifoldMesh(const manifold::MeshGL64& gl)
{
    SurfaceMesh mesh;
    const size_t stride = gl.numProp;
    mesh.vertices.reserve(gl.vertProperties.size() / stride);
    for (size_t i = 0; i + 2 < gl.vertProperties.size(); i += stride)
    {
        mesh.vertices.emplace_back(gl.vertProperties[i], gl.vertProperties[i + 1], gl.vertProperties[i + 2]);
    }
    mesh.triangles.reserve(gl.triVerts.size() / 3);
    for (size_t i = 0; i + 2 < gl.triVerts.size(); i += 3)
    {
        mesh.triangles.push_back({static_cast<int>(gl.triVerts[i]), static_cast<int>(gl.triVerts[i + 1]),
                                  static_cast<int>(gl.triVerts[i + 2])});
    }
    return mesh;
}

struct Box
{
    double lo[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max()};
    double hi[3] = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::lowest()};

    void Add(const gp_Pnt& p)
    {
        for (int k = 0; k < 3; ++k)
        {
            lo[k] = std::min(lo[k], p.Coord(k + 1));
            hi[k] = std::max(hi[k], p.Coord(k + 1));
        }
    }
    bool Near(const gp_Pnt& p, double reach) const
    {
        for (int k = 0; k < 3; ++k)
        {
            if (p.Coord(k + 1) < lo[k] - reach || p.Coord(k + 1) > hi[k] + reach)
            {
                return false;
            }
        }
        return true;
    }
    bool Overlaps(const Box& other) const
    {
        for (int k = 0; k < 3; ++k)
        {
            if (hi[k] < other.lo[k] || other.hi[k] < lo[k])
            {
                return false;
            }
        }
        return true;
    }
};

// A cut can leave a zero-thickness slit: where the removed overlap was a
// sliver thinner than Manifold's precision, the result has the same triangle
// twice, with opposite orientations, on vertices duplicated at one position.
// That is topologically fine but geometrically a crack of no width, lying on
// the neighbour's surface. This merges vertices at exactly the same position,
// removes such opposite pairs, and drops triangles that became degenerate.
// The mesh is only changed if the result is still a closed 2-manifold (every
// edge used by exactly two triangles); returns whether it was changed.
bool CloseZeroThicknessSlits(SurfaceMesh& mesh)
{
    std::map<std::array<double, 3>, int> byPosition;
    std::vector<int> remap(mesh.vertices.size());
    std::vector<gp_Pnt> vertices;
    for (size_t v = 0; v < mesh.vertices.size(); ++v)
    {
        const gp_Pnt& p = mesh.vertices[v];
        const auto inserted =
            byPosition.emplace(std::array<double, 3>{p.X(), p.Y(), p.Z()}, static_cast<int>(vertices.size()));
        if (inserted.second)
        {
            vertices.push_back(p);
        }
        remap[v] = inserted.first->second;
    }
    if (vertices.size() == mesh.vertices.size())
    {
        return false; // no two vertices at one position: no slit
    }
    std::vector<std::array<int, 3>> triangles;
    for (const auto& tri : mesh.triangles)
    {
        const std::array<int, 3> t{remap[tri[0]], remap[tri[1]], remap[tri[2]]};
        if (t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
        {
            triangles.push_back(t);
        }
    }
    // Pair up coincident triangles with opposite orientations.
    std::map<std::array<int, 3>, std::vector<size_t>> byCorners;
    for (size_t t = 0; t < triangles.size(); ++t)
    {
        std::array<int, 3> key = triangles[t];
        std::sort(key.begin(), key.end());
        byCorners[key].push_back(t);
    }
    auto orientation = [&](size_t t) { // +1 or -1: the cyclic order of its corners
        const std::array<int, 3>& c = triangles[t];
        const int first = static_cast<int>(std::min_element(c.begin(), c.end()) - c.begin());
        return c[(first + 1) % 3] < c[(first + 2) % 3] ? 1 : -1;
    };
    std::vector<char> removed(triangles.size(), 0);
    for (const auto& entry : byCorners)
    {
        std::vector<size_t> up, down;
        for (size_t t : entry.second)
        {
            (orientation(t) > 0 ? up : down).push_back(t);
        }
        for (size_t k = 0; k < std::min(up.size(), down.size()); ++k)
        {
            removed[up[k]] = removed[down[k]] = 1;
        }
    }
    SurfaceMesh result;
    result.vertices = vertices;
    for (size_t t = 0; t < triangles.size(); ++t)
    {
        if (!removed[t])
        {
            result.triangles.push_back(triangles[t]);
        }
    }
    size_t open = 0, nonManifold = 0;
    CountEdgeDefects(result, open, nonManifold);
    if (open > 0 || nonManifold > 0 || result.triangles.empty())
    {
        return false;
    }
    mesh = std::move(result);
    return true;
}
} // namespace

MeshCutReport CutOverlappingMeshes(std::vector<SurfaceMesh>& meshes, const std::vector<int>& part)
{
    MeshCutReport report;
    const size_t n = meshes.size();
    auto takesPart = [&](size_t i) { return !meshes[i].triangles.empty(); };

    // Each body's mesh as a Manifold, built when a pair first needs it.
    std::vector<std::unique_ptr<manifold::Manifold>> solids(n);
    std::vector<double> volume(n, 0.0);

    std::vector<Box> boxes(n);
    for (size_t i = 0; i < n; ++i)
    {
        for (const gp_Pnt& p : meshes[i].vertices)
        {
            boxes[i].Add(p);
        }
    }

    // A mesh Manifold rejects (not closed, or not consistently oriented)
    // can't be cut.
    std::vector<char> rejected(n, 0);

    // The work is spread over all cores: the Manifold objects are built, the
    // pairs tested and the bodies cut in parallel (each task only reads the
    // shared Manifolds). Everything that depends on order (the problems
    // reported, the operands of each cut, the sums) is collected afterwards
    // in index order, so the result is the same as a serial run.
    //
    // Every overlapping pair of solids: the larger body (ties: the later one)
    // loses the overlap to the smaller. Cutting moves no vertex by more than
    // Manifold's precision, but two bodies cut separately (by the same small
    // body, say) can end up overlapping by that much. So after the first
    // round, the pairs involving a body that was just cut are checked again,
    // up to kMaxRounds rounds.
    std::vector<char> active(n, 1);
    for (int round = 0; round < kMaxRounds; ++round)
    {
        // The candidate pairs, and the Manifolds they need, in first-use order.
        std::vector<std::pair<size_t, size_t>> candidates;
        std::vector<size_t> toBuild;
        std::vector<char> queued(n, 0);
        auto need = [&](size_t i) {
            if (!solids[i] && !rejected[i] && !queued[i])
            {
                queued[i] = 1;
                toBuild.push_back(i);
            }
        };
        for (size_t i = 0; i < n; ++i)
        {
            if (!takesPart(i))
            {
                continue;
            }
            for (size_t j = i + 1; j < n; ++j)
            {
                if (!takesPart(j) || !(active[i] || active[j]) || !boxes[i].Overlaps(boxes[j]))
                {
                    continue;
                }
                need(i);
                need(j);
                candidates.emplace_back(i, j);
            }
        }
        std::vector<std::unique_ptr<manifold::Manifold>> built(toBuild.size());
        const uint32_t firstID = manifold::Manifold::ReserveIDs(static_cast<uint32_t>(toBuild.size()));
        OSD_Parallel::For(0, static_cast<int>(toBuild.size()), [&](int k) {
            built[k] = std::make_unique<manifold::Manifold>(
                ToManifoldMesh(meshes[toBuild[k]], firstID + static_cast<uint32_t>(k)));
            built[k]->Status(); // evaluate it here, in parallel
        });
        for (size_t k = 0; k < toBuild.size(); ++k)
        {
            const size_t i = toBuild[k];
            if (built[k]->Status() != manifold::Manifold::Error::NoError)
            {
                rejected[i] = 1;
                std::ostringstream problem;
                problem << "body #" << i << ": its mesh is not a closed surface, so overlaps with it can't be cut"
                        << " (Manifold error " << static_cast<int>(built[k]->Status()) << ")";
                report.problems.push_back(problem.str());
                continue;
            }
            volume[i] = built[k]->Volume();
            solids[i] = std::move(built[k]);
        }

        std::vector<double> common(candidates.size(), 0.0);
        OSD_Parallel::For(0, static_cast<int>(candidates.size()), [&](int k) {
            const auto [i, j] = candidates[k];
            if (solids[i] && solids[j])
            {
                common[k] = (*solids[i] ^ *solids[j]).Volume();
            }
        });
        std::vector<std::vector<size_t>> smallerOverlaps(n);
        size_t pairs = 0;
        for (size_t k = 0; k < candidates.size(); ++k)
        {
            const auto [i, j] = candidates[k];
            if (!solids[i] || !solids[j] || common[k] <= kMinOverlapFraction * std::min(volume[i], volume[j]))
            {
                continue;
            }
            ++pairs;
            const bool iLoses = volume[i] > volume[j];
            smallerOverlaps[iLoses ? i : j].push_back(iLoses ? j : i);
        }
        if (pairs == 0)
        {
            break;
        }
        report.overlappingPairs += pairs;
        if (round == kMaxRounds - 1)
        {
            std::ostringstream problem;
            problem << pairs << " pair(s) still overlap after " << kMaxRounds - 1 << " rounds of cutting";
            report.problems.push_back(problem.str());
            break;
        }

        std::vector<size_t> losers;
        for (size_t i = 0; i < n; ++i)
        {
            if (!smallerOverlaps[i].empty())
            {
                losers.push_back(i);
            }
        }
        struct Cut
        {
            manifold::Manifold result;
            size_t mergedCutters = 0;
        };
        std::vector<Cut> cuts(losers.size());
        OSD_Parallel::For(0, static_cast<int>(losers.size()), [&](int k) {
            const size_t i = losers[k];
            // One cutter per repeated part (its placements that cut this body,
            // merged into one mesh), and one per other smaller body.
            std::map<int, std::vector<manifold::Manifold>> byPart;
            std::vector<manifold::Manifold> operands{*solids[i]};
            for (size_t j : smallerOverlaps[i])
            {
                if (part[j] >= 0)
                {
                    byPart[part[j]].push_back(*solids[j]);
                }
                else
                {
                    operands.push_back(*solids[j]);
                }
            }
            for (const auto& entry : byPart)
            {
                if (entry.second.size() == 1)
                {
                    operands.push_back(entry.second.front());
                    continue;
                }
                operands.push_back(manifold::Manifold::BatchBoolean(entry.second, manifold::OpType::Add));
                ++cuts[k].mergedCutters;
            }
            cuts[k].result = manifold::Manifold::BatchBoolean(operands, manifold::OpType::Subtract);
            cuts[k].result.Status(); // evaluate it here, in parallel
        });

        std::vector<char> cutNow(n, 0);
        std::vector<SurfaceMesh> cutMeshes(n);
        std::vector<char> keep(losers.size(), 0);
        for (size_t k = 0; k < losers.size(); ++k)
        {
            const size_t i = losers[k];
            const manifold::Manifold& result = cuts[k].result;
            report.mergedCutters += cuts[k].mergedCutters;
            if (result.Status() != manifold::Manifold::Error::NoError)
            {
                std::ostringstream problem;
                problem << "body #" << i << ": the mesh cut failed (Manifold error "
                        << static_cast<int>(result.Status()) << "); its overlaps are left as they are";
                report.problems.push_back(problem.str());
                continue;
            }
            const double remaining = result.Volume();
            report.bodiesCut += round == 0 ? 1 : 0;
            report.volumeRemoved += volume[i] - remaining;
            cutNow[i] = 1;
            if (remaining < kMinRemainingFraction * volume[i])
            {
                std::ostringstream problem;
                problem << "body #" << i << ": dropped, " << 100.0 * (1.0 - remaining / volume[i])
                        << "% of it overlaps smaller bodies";
                report.problems.push_back(problem.str());
                continue; // cutMeshes[i] stays empty
            }
            keep[k] = 1;
        }
        std::vector<char> slitClosed(losers.size(), 0);
        OSD_Parallel::For(0, static_cast<int>(losers.size()), [&](int k) {
            if (keep[k])
            {
                const size_t i = losers[k];
                cutMeshes[i] = FromManifoldMesh(cuts[k].result.GetMeshGL64());
                slitClosed[k] = CloseZeroThicknessSlits(cutMeshes[i]) ? 1 : 0;
            }
        });
        for (char closed : slitClosed)
        {
            report.slitsClosed += closed;
        }
        // Use the cut meshes only now, so every body of this round was cut by
        // the others' meshes as they were before it.
        for (size_t i = 0; i < n; ++i)
        {
            if (!cutNow[i])
            {
                continue;
            }
            meshes[i] = std::move(cutMeshes[i]);
            solids[i].reset();
            boxes[i] = Box();
            for (const gp_Pnt& p : meshes[i].vertices)
            {
                boxes[i].Add(p);
            }
        }
        active = cutNow;
    }
    return report;
}

void CountEdgeDefects(const SurfaceMesh& mesh, size_t& openEdges, size_t& nonManifoldEdges)
{
    std::unordered_map<uint64_t, int> uses;
    uses.reserve(3 * mesh.triangles.size());
    for (const auto& tri : mesh.triangles)
    {
        for (int k = 0; k < 3; ++k)
        {
            const uint64_t a = static_cast<uint32_t>(tri[k]), b = static_cast<uint32_t>(tri[(k + 1) % 3]);
            ++uses[std::min(a, b) << 32 | std::max(a, b)];
        }
    }
    openEdges = nonManifoldEdges = 0;
    for (const auto& entry : uses)
    {
        openEdges += entry.second == 1;
        nonManifoldEdges += entry.second > 2;
    }
}

size_t FlipDegenerateTriangles(SurfaceMesh& mesh, double minHeight, double minArea, size_t& remaining)
{
    auto& tris = mesh.triangles;
    // The sliver's longest edge, as k (the edge tri[k] -> tri[k + 1]), or -1
    // if the triangle isn't a sliver: lower than minHeight, or smaller than
    // minArea with its height under kCapFraction of its shortest edge (a
    // vertex just off the opposite edge; a tiny triangle with a short edge is
    // left to CollapseTinyTriangles).
    constexpr double kCapFraction = 0.01;
    auto sliverEdge = [&](const std::array<int, 3>& tri) {
        const gp_Pnt* p[3] = {&mesh.vertices[tri[0]], &mesh.vertices[tri[1]], &mesh.vertices[tri[2]]};
        double longest = 0.0, shortest = std::numeric_limits<double>::max();
        int edge = -1;
        for (int k = 0; k < 3; ++k)
        {
            const double length = p[k]->SquareDistance(*p[(k + 1) % 3]);
            shortest = std::min(shortest, length);
            if (length > longest)
            {
                longest = length;
                edge = k;
            }
        }
        if (edge < 0)
        {
            return -1;
        }
        const double twiceArea = gp_Vec(*p[0], *p[1]).Crossed(gp_Vec(*p[0], *p[2])).Magnitude();
        const double height = twiceArea / std::sqrt(longest);
        const bool cap = 0.5 * twiceArea < minArea && height < kCapFraction * std::sqrt(shortest);
        return height < minHeight || cap ? edge : -1;
    };
    remaining = 0;
    if (std::none_of(tris.begin(), tris.end(), [&](const std::array<int, 3>& tri) { return sliverEdge(tri) >= 0; }))
    {
        return 0;
    }

    // Directed edge a -> b to its triangle (each is used once in a closed,
    // oriented mesh).
    auto key = [](int a, int b) {
        return static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32 | static_cast<uint32_t>(b);
    };
    std::unordered_map<uint64_t, int> edgeTriangle;
    auto link = [&](int t, bool add) {
        for (int k = 0; k < 3; ++k)
        {
            const uint64_t edge = key(tris[t][k], tris[t][(k + 1) % 3]);
            add ? (void)(edgeTriangle[edge] = t) : (void)edgeTriangle.erase(edge);
        }
    };
    for (int t = 0; t < static_cast<int>(tris.size()); ++t)
    {
        link(t, true);
    }

    // Sliver (a, b, c) with c on its longest edge ab, and the neighbour
    // (b, a, d) across ab: the pair becomes (c, a, d) and (d, b, c). Their
    // outline (b, c, a, d) is the same, so the surface stays put, and the
    // new edge cd is used once in each direction. A flip can leave another
    // sliver (d on the same line), hence the passes.
    size_t flips = 0;
    for (int pass = 0; pass < 8; ++pass)
    {
        bool changed = false;
        for (int t = 0; t < static_cast<int>(tris.size()); ++t)
        {
            const int k = sliverEdge(tris[t]);
            if (k < 0)
            {
                continue;
            }
            const int a = tris[t][k], b = tris[t][(k + 1) % 3], c = tris[t][(k + 2) % 3];
            const auto across = edgeTriangle.find(key(b, a));
            if (across == edgeTriangle.end())
            {
                continue;
            }
            const int u = across->second;
            int d = -1;
            for (int v : tris[u])
            {
                d = v != a && v != b ? v : d;
            }
            if (d < 0 || d == c || edgeTriangle.count(key(c, d)) || edgeTriangle.count(key(d, c)))
            {
                continue;
            }
            link(t, false);
            link(u, false);
            tris[t] = {c, a, d};
            tris[u] = {d, b, c};
            link(t, true);
            link(u, true);
            ++flips;
            changed = true;
        }
        if (!changed)
        {
            break;
        }
    }
    remaining = static_cast<size_t>(
        std::count_if(tris.begin(), tris.end(), [&](const std::array<int, 3>& tri) { return sliverEdge(tri) >= 0; }));
    return flips;
}

namespace
{
using PointKey = std::array<double, 3>;

PointKey KeyOf(const gp_Pnt& p)
{
    return {p.X(), p.Y(), p.Z()};
}

double TriangleArea(const SurfaceMesh& mesh, const std::array<int, 3>& tri)
{
    const gp_Pnt& a = mesh.vertices[tri[0]];
    return 0.5 * gp_Vec(a, mesh.vertices[tri[1]]).Crossed(gp_Vec(a, mesh.vertices[tri[2]])).Magnitude();
}

// True if every directed edge is used exactly once and its reverse too: a
// closed, consistently oriented surface without doubled triangles.
bool IsClosedOriented(const SurfaceMesh& mesh)
{
    std::unordered_map<uint64_t, int> uses;
    uses.reserve(3 * mesh.triangles.size());
    auto key = [](int a, int b) {
        return static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32 | static_cast<uint32_t>(b);
    };
    for (const auto& tri : mesh.triangles)
    {
        for (int k = 0; k < 3; ++k)
        {
            if (++uses[key(tri[k], tri[(k + 1) % 3])] > 1)
            {
                return false;
            }
        }
    }
    for (const auto& entry : uses)
    {
        const int a = static_cast<int>(entry.first >> 32), b = static_cast<int>(entry.first & 0xffffffffu);
        if (!uses.count(key(b, a)))
        {
            return false;
        }
    }
    return true;
}

// True if a triangle's bounding box comes within 'reach' of 'point'.
bool HasTriangleNear(const SurfaceMesh& mesh, const gp_Pnt& point, double reach)
{
    for (const auto& tri : mesh.triangles)
    {
        bool inReach = true;
        for (int k = 1; k <= 3 && inReach; ++k)
        {
            const double a = mesh.vertices[tri[0]].Coord(k), b = mesh.vertices[tri[1]].Coord(k),
                         c = mesh.vertices[tri[2]].Coord(k);
            inReach = point.Coord(k) >= std::min({a, b, c}) - reach && point.Coord(k) <= std::max({a, b, c}) + reach;
        }
        if (inReach)
        {
            return true;
        }
    }
    return false;
}

// Moves the vertices 'from' (all at one point) of 'mesh' to 'to': merged into
// the vertex already there, if any, or moved. The triangles that collapse
// are dropped, and unused vertices removed. Returns false, leaving 'mesh'
// as it was, if the result isn't a closed, oriented surface or a remaining
// triangle around the point would turn over.
bool MovePoint(SurfaceMesh& mesh, const std::vector<int>& from, const gp_Pnt& to)
{
    SurfaceMesh result = mesh;
    int target = -1;
    for (int v = 0; v < static_cast<int>(result.vertices.size()); ++v)
    {
        if (KeyOf(result.vertices[v]) == KeyOf(to))
        {
            target = v;
            break;
        }
    }
    if (target < 0)
    {
        target = from.front();
        result.vertices[target] = to;
    }
    auto moved = [&](int v) { return std::find(from.begin(), from.end(), v) != from.end(); };
    std::vector<std::array<int, 3>> triangles;
    for (const auto& tri : mesh.triangles)
    {
        std::array<int, 3> next = tri;
        bool touches = false;
        for (int& v : next)
        {
            if (moved(v))
            {
                v = target;
                touches = true;
            }
        }
        if (next[0] == next[1] || next[1] == next[2] || next[2] == next[0])
        {
            continue; // collapsed
        }
        if (touches)
        {
            const gp_Pnt& a = mesh.vertices[tri[0]];
            const gp_Vec before = gp_Vec(a, mesh.vertices[tri[1]]).Crossed(gp_Vec(a, mesh.vertices[tri[2]]));
            const gp_Pnt& b = result.vertices[next[0]];
            const gp_Vec after =
                gp_Vec(b, result.vertices[next[1]]).Crossed(gp_Vec(b, result.vertices[next[2]]));
            if (before.Dot(after) <= 0.0)
            {
                return false; // turned over, or became flat
            }
        }
        triangles.push_back(next);
    }
    result.triangles = std::move(triangles);
    if (result.triangles.size() < 4 || !IsClosedOriented(result))
    {
        return false;
    }
    // Drop the vertices no triangle uses any more.
    std::vector<int> index(result.vertices.size(), -1);
    std::vector<gp_Pnt> vertices;
    for (auto& tri : result.triangles)
    {
        for (int& v : tri)
        {
            if (index[v] < 0)
            {
                index[v] = static_cast<int>(vertices.size());
                vertices.push_back(result.vertices[v]);
            }
            v = index[v];
        }
    }
    result.vertices = std::move(vertices);
    mesh = std::move(result);
    return true;
}
} // namespace

size_t CollapseTinyTriangles(std::vector<SurfaceMesh>& meshes, double minArea, double maxMove, bool keepApart,
                             size_t& remaining)
{
    // For keepApart: each body's box, and its mesh as a Manifold (built when
    // first needed, dropped when the mesh changes).
    std::vector<Box> boxes(meshes.size());
    std::vector<std::unique_ptr<manifold::Manifold>> solids(meshes.size());
    auto boxOf = [&](size_t i) {
        boxes[i] = Box();
        for (const gp_Pnt& p : meshes[i].vertices)
        {
            boxes[i].Add(p);
        }
    };
    if (keepApart)
    {
        for (size_t i = 0; i < meshes.size(); ++i)
        {
            boxOf(i);
        }
    }
    auto solidOf = [&](size_t i) -> const manifold::Manifold& {
        if (!solids[i])
        {
            solids[i] = std::make_unique<manifold::Manifold>(
                ToManifoldMesh(meshes[i], manifold::Manifold::ReserveIDs(1)));
        }
        return *solids[i];
    };
    // A collapse moves a point by up to 'reach'. A body that has no vertex
    // there but whose face passes nearby can end up overlapping the moved
    // surface by a hair, so every changed mesh is tested against every body
    // within reach of the point; the collapse is made only if none overlaps.
    auto staysApart = [&](const std::vector<std::pair<size_t, SurfaceMesh>>& changed, const gp_Pnt& point,
                          double reach) {
        std::map<size_t, manifold::Manifold> after;
        for (const auto& [i, mesh] : changed)
        {
            after.emplace(i, manifold::Manifold(ToManifoldMesh(mesh, manifold::Manifold::ReserveIDs(1))));
            if (after.at(i).Status() != manifold::Manifold::Error::NoError)
            {
                return false;
            }
        }
        // The pairs to test, and the unchanged bodies' Manifolds built first,
        // so the tests below only read them, in parallel.
        std::vector<std::pair<size_t, size_t>> pairs;
        for (const auto& entry : after)
        {
            for (size_t j = 0; j < meshes.size(); ++j)
            {
                if (j != entry.first && !meshes[j].triangles.empty() && boxes[j].Near(point, reach) &&
                    HasTriangleNear(meshes[j], point, reach))
                {
                    pairs.emplace_back(entry.first, j);
                    if (!after.count(j))
                    {
                        solidOf(j);
                    }
                }
            }
        }
        std::vector<char> apart(pairs.size(), 1);
        OSD_Parallel::For(0, static_cast<int>(pairs.size()), [&](int k) {
            const manifold::Manifold& solid = after.at(pairs[k].first);
            const auto changedJ = after.find(pairs[k].second);
            const manifold::Manifold& other = changedJ != after.end() ? changedJ->second : *solids[pairs[k].second];
            apart[k] = other.Status() == manifold::Manifold::Error::NoError &&
                       (solid ^ other).Volume() <= kMinOverlapFraction * std::min(solid.Volume(), other.Volume());
        });
        return std::all_of(apart.begin(), apart.end(), [](char a) { return a != 0; });
    };

    // Moves tried and rejected (not tried again).
    std::set<std::pair<PointKey, PointKey>> rejected;
    size_t collapsed = 0;
    for (int pass = 0; pass < 32; ++pass)
    {
        // The ways to remove each tiny triangle: collapsing one of its edges
        // no longer than maxMove, either end onto the other, as moves
        // (from, to). Shortest edges first; the same move found in several
        // bodies (a shared edge) is one move.
        std::map<std::pair<PointKey, PointKey>, double> options;
        for (const SurfaceMesh& mesh : meshes)
        {
            for (const auto& tri : mesh.triangles)
            {
                if (TriangleArea(mesh, tri) >= minArea)
                {
                    continue;
                }
                for (int k = 0; k < 3; ++k)
                {
                    const gp_Pnt& a = mesh.vertices[tri[k]];
                    const gp_Pnt& b = mesh.vertices[tri[(k + 1) % 3]];
                    const double length = a.Distance(b);
                    if (length <= maxMove)
                    {
                        options[{KeyOf(a), KeyOf(b)}] = length;
                        options[{KeyOf(b), KeyOf(a)}] = length;
                    }
                }
            }
        }
        std::vector<std::pair<double, std::pair<PointKey, PointKey>>> candidates;
        for (const auto& [move, length] : options)
        {
            if (!rejected.count(move))
            {
                candidates.emplace_back(length, move);
            }
        }
        std::sort(candidates.begin(), candidates.end());
        if (candidates.empty())
        {
            break;
        }

        // A move touches two points; after it, the triangles around them have
        // changed, so each point is moved or kept at most once per pass. (Every
        // option for a tiny triangle involves two of its corners, so once one
        // removes it, the others are skipped.)
        std::set<PointKey> touched;
        size_t done = 0;
        for (const auto& candidate : candidates)
        {
            const auto& [from, to] = candidate.second;
            if (touched.count(from) || touched.count(to))
            {
                continue;
            }
            // Every body with a vertex at the moving point moves it too, so
            // touching bodies stay flush.
            std::vector<std::pair<size_t, SurfaceMesh>> results;
            bool ok = true;
            for (size_t i = 0; i < meshes.size() && ok; ++i)
            {
                std::vector<int> vertices;
                for (int v = 0; v < static_cast<int>(meshes[i].vertices.size()); ++v)
                {
                    if (KeyOf(meshes[i].vertices[v]) == from)
                    {
                        vertices.push_back(v);
                    }
                }
                if (vertices.empty())
                {
                    continue;
                }
                SurfaceMesh mesh = meshes[i];
                ok = MovePoint(mesh, vertices, gp_Pnt(to[0], to[1], to[2]));
                results.emplace_back(i, std::move(mesh));
            }
            if (results.empty())
            {
                continue; // the point is gone
            }
            const gp_Pnt point(to[0], to[1], to[2]);
            if (!ok || results.empty() ||
                (keepApart && !staysApart(results, point, 2.0 * point.Distance(gp_Pnt(from[0], from[1], from[2])))))
            {
                rejected.insert(candidate.second);
                continue;
            }
            for (auto& [i, mesh] : results)
            {
                meshes[i] = std::move(mesh);
                if (keepApart)
                {
                    boxOf(i);
                    solids[i].reset();
                }
            }
            touched.insert(from);
            touched.insert(to);
            ++collapsed;
            ++done;
        }
        if (done == 0)
        {
            break;
        }
    }
    remaining = CountTinyTriangles(meshes, minArea);
    return collapsed;
}

size_t CountTinyTriangles(const std::vector<SurfaceMesh>& meshes, double minArea)
{
    size_t count = 0;
    for (const SurfaceMesh& mesh : meshes)
    {
        for (const auto& tri : mesh.triangles)
        {
            count += TriangleArea(mesh, tri) < minArea ? 1 : 0;
        }
    }
    return count;
}
