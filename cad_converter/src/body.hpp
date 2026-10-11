// =============================================================================
// body.hpp — one placed solid, shell or face of the model.
// =============================================================================
#pragma once

#include <TopoDS_Shape.hxx>

#include <string>

// A body is the unit that gets its own mesh, and that overlaps are resolved
// between. Bodies keep their order through the whole pipeline.
struct Body
{
    std::string name;     // e.g. "solid_0007"; the object name with --per-body
    TopoDS_Shape shape;   // the placed solid, shell or face
    bool isSolid = false; // only solids enclose a volume
    bool meshCut = false; // left out of the general fuse; its overlaps are cut on the mesh
    std::string part;     // the STEP part (product) it is a placement of; "" if unknown
};
