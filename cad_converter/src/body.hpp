// =============================================================================
// body.hpp — one placed solid of the model.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <string>

// A body is the unit that gets its own mesh, and that overlaps are resolved
// between. Bodies keep their order through the whole pipeline.
struct Body
{
    std::string name;      // e.g. "solid_0007"; the object name with --per-body
    TopoDS_Shape shape;    // the placed solid (after the fuse, possibly several pieces)
    bool meshCut = false;  // left out of the general fuse; its overlaps are cut on the mesh
    std::string part;      // the STEP part (product) it is a placement of; "" if unknown
    size_t partBodies = 1; // bodies in one placement of that part (to count its instances)
};
