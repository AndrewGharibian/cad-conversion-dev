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
