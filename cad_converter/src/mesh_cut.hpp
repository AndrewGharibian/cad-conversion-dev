// =============================================================================
// mesh_cut.hpp — resolve overlaps between body meshes with mesh booleans.
//
// Bodies left out of the general fuse (thread pairs, see thread_pairs.hpp)
// still overlap their neighbours after meshing. CutOverlappingMeshes removes
// each overlap from the larger body, using the Manifold library's robust
// mesh booleans, so the smaller body keeps its volume as in the fuse.
// See mesh_cut.cpp for details.
// =============================================================================
#pragma once

#include "weld.hpp"

#include <cstddef>
#include <string>
#include <vector>

struct MeshCutReport
{
    size_t overlappingPairs = 0; // overlapping pairs found (and cut), over all rounds
    size_t bodiesCut = 0;        // bodies that lost volume
    double volumeRemoved = 0.0;  // total volume taken from the larger bodies
    size_t mergedCutters = 0;    // cuts that subtracted several placements of a part at once
    size_t slitsClosed = 0;      // cut meshes cleaned of zero-thickness slits
    std::vector<std::string> problems; // pairs or bodies that couldn't be handled
};

// meshes[i] is body i's closed mesh; isSolid[i] says whether it encloses a
// volume (only solids take part). Every pair of solids whose meshes overlap
// is resolved: the larger body (by mesh volume) loses the overlap. Meshes
// are replaced in place.
//
// part[i] >= 0 says which part body i is a placement of (bodies with the
// same value are copies of one part); -1 if unknown. Placements of one part
// that cut the same body are merged into one cutter.
MeshCutReport CutOverlappingMeshes(std::vector<SurfaceMesh>& meshes, const std::vector<char>& isSolid,
                                   const std::vector<int>& part);

// Open edges (used by one triangle) and edges used by more than two
// triangles, counted on a welded mesh by vertex index.
void CountEdgeDefects(const SurfaceMesh& mesh, size_t& openEdges, size_t& nonManifoldEdges);
