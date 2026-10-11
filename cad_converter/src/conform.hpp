// =============================================================================
// conform.hpp — make touching solids share faces and give every overlap to
// one solid (the general fuse), and repair what the fuse breaks.
// =============================================================================
#pragma once

#include "body.hpp"
#include "weld.hpp"

#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <vector>

// What a later local re-fuse needs: every solid's original (placed, unfused)
// shape and volume, by body name, and the fuse's fuzzy value.
struct FuseContext
{
    bool ready = false; // false: no fuse ran (or it failed), nothing to re-fuse
    double fuzzy = 0.0;
    std::map<std::string, TopoDS_Shape> original;
    std::map<std::string, double> volume;
};

// Fuses all solids (except thread pairs, found first if 'detectThreadPairs'),
// checks the result and repairs what the fuse broke (RepairFuse). Bodies left
// out of the fuse get Body::meshCut; bodies left with almost nothing (the same
// part placed twice) are dropped.
FuseContext MakeBodiesConformal(std::vector<Body>& bodies, double maxDeviation, bool detectThreadPairs);

// Leaves the bodies 'cut' out of the fuse and fuses again only their
// neighbourhood. Returns the names of the bodies whose shapes changed.
std::set<std::string> RefuseLocally(std::vector<Body>& bodies, const FuseContext& context,
                                    const std::vector<size_t>& cut);

// Checks the fused bodies named in 'check' and, while the fuse broke some,
// leaves the smaller body of each broken pair out of it and re-fuses locally.
// Returns the names of the bodies whose shapes changed.
std::set<std::string> RepairFuse(std::vector<Body>& bodies, const FuseContext& context, std::set<std::string> check);

// The bodies to leave out of the fuse because a fused solid's final mesh isn't
// a closed 2-manifold: for each defective solid, the smaller body of every
// pair it forms with a solid touching it at the defect (within 'reach').
std::vector<size_t> PartnersOfMeshDefects(const std::vector<Body>& bodies, const std::vector<SurfaceMesh>& meshes,
                                          double reach);

constexpr int kMaxMeshPasses = 3;          // mesh, then up to 2 local re-fuses
constexpr double kTouchFraction = 1.0e-5;  // "touching": within this fraction of the model size
