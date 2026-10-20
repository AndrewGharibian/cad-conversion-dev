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

// meshes[i] is body i's closed mesh (empty if it was dropped). Every pair of
// solids whose meshes overlap is resolved: the larger body (by mesh volume) loses the overlap. Meshes
// are replaced in place.
//
// part[i] >= 0 says which part body i is a placement of (bodies with the
// same value are copies of one part); -1 if unknown. Placements of one part
// that cut the same body are merged into one cutter.
MeshCutReport CutOverlappingMeshes(std::vector<SurfaceMesh>& meshes, const std::vector<int>& part);

// Open edges (used by one triangle) and edges used by more than two
// triangles, counted on a welded mesh by vertex index.
void CountEdgeDefects(const SurfaceMesh& mesh, size_t& openEdges, size_t& nonManifoldEdges);

// Removes slivers: triangles whose height (twice the area over the longest
// edge) is below 'minHeight', i.e. a vertex lying on the opposite edge, and
// triangles under 'minArea' whose height is tiny next to their shortest edge
// (a vertex just off the opposite edge). Each is fixed by flipping that edge
// with the neighbouring triangle, which moves the surface by at most that
// height and leaves the mesh closed and oriented. Returns the number of
// flips; 'remaining' gets the slivers that couldn't be fixed.
size_t FlipDegenerateTriangles(SurfaceMesh& mesh, double minHeight, double minArea, size_t& remaining);

// Removes tiny triangles (area below 'minArea'), which ROOT reports as
// degenerate or of zero surface area, by collapsing one of their edges at
// most 'maxMove' long (shortest first, either end onto the other): its two
// ends become one point. Every body
// with a vertex at the moving point moves it too, so touching bodies stay
// flush, and a collapse is only made if every mesh it changes stays a
// closed, oriented surface with no triangle turned over. With 'keepApart',
// also only if no changed mesh then overlaps a nearby body (tested with
// Manifold), for meshes that must stay overlap-free. Returns the number of
// collapses; 'remaining' gets the tiny triangles left.
size_t CollapseTinyTriangles(std::vector<SurfaceMesh>& meshes, double minArea, double maxMove, bool keepApart,
                             size_t& remaining);

// Triangles with an area below 'minArea', over all meshes.
size_t CountTinyTriangles(const std::vector<SurfaceMesh>& meshes, double minArea);
