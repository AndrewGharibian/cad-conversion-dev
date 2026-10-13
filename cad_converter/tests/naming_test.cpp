// =============================================================================
// naming_test — the output object names chosen by GroupBodies (export.cpp).
// =============================================================================
#include "export.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace
{
int failures = 0;

void Expect(const std::vector<OutputObject>& objects, const std::vector<std::string>& names, const char* what)
{
    std::vector<std::string> got;
    for (const OutputObject& object : objects)
    {
        got.push_back(object.name);
    }
    if (got != names)
    {
        ++failures;
        std::cout << "FAILED: " << what << ": got";
        for (const std::string& name : got)
        {
            std::cout << " '" << name << "'";
        }
        std::cout << "\n";
    }
}

Body MakeBody(const std::string& name, const std::string& part, size_t partBodies)
{
    Body body;
    body.name = name;
    body.part = part;
    body.partBodies = partBodies;
    return body;
}
} // namespace

int main()
{
    SurfaceMesh mesh;
    mesh.vertices = {gp_Pnt(0, 0, 0), gp_Pnt(1, 0, 0), gp_Pnt(0, 1, 0)};
    mesh.triangles = {{0, 1, 2}};
    const SurfaceMesh empty;

    // Three placements of a screw, one board made of two bodies, two bodies
    // without a part name, a part named with a space, and a body whose mesh
    // was dropped.
    const std::vector<Body> bodies = {
        MakeBody("solid_0001", "Screw M3", 1),  MakeBody("solid_0002", "", 1),
        MakeBody("solid_0003", "Screw M3", 1),  MakeBody("solid_0004", "Board", 2),
        MakeBody("solid_0005", "Board", 2),     MakeBody("solid_0006", "Screw M3", 1),
        MakeBody("solid_0007", "", 1),          MakeBody("solid_0008", "Dropped", 1),
        MakeBody("solid_0009", "Cover", 3),
    };
    const std::vector<SurfaceMesh> meshes = {mesh, mesh, mesh, mesh, mesh, mesh, mesh, empty, mesh};

    const std::vector<OutputObject> byPart = GroupBodies(bodies, meshes, false);
    Expect(byPart, {"Screw_M3_instanced3", "mesh1", "Board", "mesh2", "Cover"}, "one object per part");
    if (byPart.size() == 5 && byPart[0].bodies != std::vector<size_t>{0, 2, 5})
    {
        ++failures;
        std::cout << "FAILED: the screw object should hold bodies 0, 2 and 5\n";
    }

    Expect(GroupBodies(bodies, meshes, true),
           {"solid_0001", "solid_0002", "solid_0003", "solid_0004", "solid_0005", "solid_0006", "solid_0007",
            "solid_0009"},
           "one object per body");

    // A part name that clashes with another object's name gets a suffix.
    const std::vector<Body> clash = {MakeBody("solid_0001", "", 1), MakeBody("solid_0002", "mesh1", 1)};
    Expect(GroupBodies(clash, {mesh, mesh}, false), {"mesh1", "mesh1_2"}, "name clash");

    std::cout << (failures ? "FAILED" : "OK") << "\n";
    return failures ? 1 : 0;
}
