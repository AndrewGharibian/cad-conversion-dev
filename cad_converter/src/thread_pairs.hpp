// =============================================================================
// thread_pairs.hpp — find pairs of solids that the general fuse can't handle:
// modeled screw threads sitting in modeled tapped holes, and anything else
// where a freeform face runs just inside another solid.
//
// See thread_pairs.cpp for the full explanation. In short: two thread
// surfaces that are meant to coincide but differ by CAD round-off cross each
// other over and over, and the general fuse turns that into helical slivers
// far thinner than the mesh can resolve. Such pairs are left out of the fuse
// and their overlap is cut on the mesh instead (mesh_cut.hpp).
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <vector>

struct ThreadPair
{
    size_t a = 0, b = 0;   // indices into the 'solids' argument, a < b
    size_t nearSamples = 0; // sample points found just inside the other solid
    size_t samples = 0;     // sample points tested on the freeform faces involved
};

// Every pair of solids where a freeform (non-analytic) face of one lies inside
// the other, no deeper than 'depthLimit', over a substantial part of the face.
// Runs in parallel; pairs come back sorted by (a, b).
std::vector<ThreadPair> FindThreadPairs(const std::vector<TopoDS_Shape>& solids, double depthLimit);
