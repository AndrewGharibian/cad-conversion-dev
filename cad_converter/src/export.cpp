// =============================================================================
// export.cpp — group the bodies into output objects, and write them.
//
// Grouping:
//
// By default every part becomes one object: all placements of a part (all
// copies of a screw, say) are written together as one mesh, named after the
// part, since they are the same material. Each placement stays its own
// closed surface inside that mesh. A body whose part is unknown is its own
// object. With --per-body, every body is its own object, named solid_NNNN,
// shell_NNNN or face_NNNN.
//
// Object names have whitespace replaced by '_' (an STL "solid" line and an
// OBJ "o" line end at the line, and some readers split names at spaces) and
// get a _2, _3, ... suffix if two different parts end up with the same name.
// =============================================================================
#include "export.hpp"

#include <gp_Vec.hxx>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <stdexcept>

std::vector<OutputObject> GroupBodies(const std::vector<Body>& bodies, bool perBody)
{
    std::vector<OutputObject> objects;
    std::map<std::string, size_t> byPart; // part name -> object index
    std::map<std::string, int> nameUses;
    auto addObject = [&](std::string name) {
        for (char& c : name)
        {
            if (std::isspace(static_cast<unsigned char>(c)))
            {
                c = '_';
            }
        }
        const int uses = ++nameUses[name];
        if (uses > 1)
        {
            name += "_" + std::to_string(uses);
        }
        objects.push_back({name, {}});
        return objects.size() - 1;
    };
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (perBody || bodies[i].part.empty())
        {
            objects[addObject(bodies[i].name)].bodies.push_back(i);
            continue;
        }
        auto found = byPart.find(bodies[i].part);
        if (found == byPart.end())
        {
            found = byPart.emplace(bodies[i].part, addObject(bodies[i].part)).first;
        }
        objects[found->second].bodies.push_back(i);
    }
    return objects;
}

// =============================================================================
// STL: MeshLab, Blender, etc. can display it, and it's a
// common input format for simulation geometry.
//
// Every output object is written as its own named mesh: ASCII STL allows
// several "solid <name> ... endsolid <name>" blocks in one file, and
// importers (Blender, MeshLab, ROOT/Geant4 readers) load each block as a
// separate object. (Binary STL has no such structure, which is why this is
// ASCII.) Within a block, each triangle has a facet normal and three
// vertices in counter-clockwise order seen from outside. 17 significant
// digits round-trip a double exactly, so shared vertices stay bit-identical
// in the file and each body stays watertight for whatever reads it.
// =============================================================================
void ExportStl(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
               const std::string& outputPath)
{
    std::ofstream out(outputPath);
    if (!out)
    {
        throw std::runtime_error("Cannot write STL file: " + outputPath);
    }
    out.precision(17);

    for (const OutputObject& object : objects)
    {
        if (std::all_of(object.bodies.begin(), object.bodies.end(),
                        [&](size_t i) { return meshes[i].triangles.empty(); }))
        {
            continue;
        }
        out << "solid " << object.name << "\n";
        for (size_t i : object.bodies)
        {
            const SurfaceMesh& mesh = meshes[i];
            for (const auto& tri : mesh.triangles)
            {
                const gp_Pnt& a = mesh.vertices[tri[0]];
                const gp_Pnt& b = mesh.vertices[tri[1]];
                const gp_Pnt& c = mesh.vertices[tri[2]];
                gp_Vec normal = gp_Vec(a, b).Crossed(gp_Vec(a, c));
                if (normal.Magnitude() > 0.0)
                {
                    normal.Normalize();
                }
                out << "  facet normal " << normal.X() << " " << normal.Y() << " " << normal.Z() << "\n"
                    << "    outer loop\n";
                for (const gp_Pnt* p : {&a, &b, &c})
                {
                    out << "      vertex " << p->X() << " " << p->Y() << " " << p->Z() << "\n";
                }
                out << "    endloop\n"
                    << "  endfacet\n";
            }
        }
        out << "endsolid " << object.name << "\n";
    }
}

// =============================================================================
// OBJ (--obj): Wavefront OBJ.
//
// OBJ is an indexed format: each vertex is written once ("v x y z") and
// triangles refer to vertices by number ("f i j k", counting from 1 across
// the whole file). So each body's welded mesh keeps its shared vertices in
// the file (STL repeats all three corners of every triangle), and the file
// is much smaller. Every output object is its own named object
// ("o <name>"), which importers load as separate objects; the bodies in an
// object never share vertices, so each stays its own closed surface.
// Triangles are counter-clockwise seen from outside, as in the STL, and 17
// significant digits keep coordinates exact, so contact faces shared by two
// bodies stay bit-identical.
// =============================================================================
void ExportObj(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
               const std::string& outputPath)
{
    std::ofstream out(outputPath);
    if (!out)
    {
        throw std::runtime_error("Cannot write OBJ file: " + outputPath);
    }
    out.precision(17);

    out << "# " << objects.size() << " objects, written by cad_converter\n";
    size_t offset = 1; // OBJ vertex numbers start at 1 and run through the whole file
    for (const OutputObject& object : objects)
    {
        if (std::all_of(object.bodies.begin(), object.bodies.end(),
                        [&](size_t i) { return meshes[i].triangles.empty(); }))
        {
            continue;
        }
        out << "o " << object.name << "\n";
        for (size_t i : object.bodies)
        {
            const SurfaceMesh& mesh = meshes[i];
            for (const gp_Pnt& p : mesh.vertices)
            {
                out << "v " << p.X() << " " << p.Y() << " " << p.Z() << "\n";
            }
            for (const auto& tri : mesh.triangles)
            {
                out << "f " << tri[0] + offset << " " << tri[1] + offset << " " << tri[2] + offset << "\n";
            }
            offset += mesh.vertices.size();
        }
    }
}
