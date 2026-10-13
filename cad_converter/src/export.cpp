// =============================================================================
// export.cpp — group the bodies into output objects, and write them.
//
// Grouping: by default every part becomes one object: all placements of a
// part (all copies of a screw, say) are written together as one mesh, since
// they are the same material. Each placement stays its own closed surface
// inside that mesh. With --per-body, every body is its own object.
//
// Naming (by default):
//   - an object is named after its part, if the part's name is known;
//   - objects without a name are numbered in output order: mesh1, mesh2, ...;
//   - a part placed more than once gets the number of placements appended:
//     <name>_instanced<n>. A placement can hold several bodies (a part made
//     of several solids), so n is the part's bodies in the object divided by
//     the bodies in one placement.
// With --per-body, objects are named solid_NNNN.
// Whitespace in names is replaced by '_' (an STL "solid" line and an OBJ "o"
// line end at the line, and some readers split names at spaces), and a name
// used twice gets a _2, _3, ... suffix.
// =============================================================================
#include "export.hpp"

#include <OSD_Parallel.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <functional>
#include <map>
#include <stdexcept>

std::vector<OutputObject> GroupBodies(const std::vector<Body>& bodies, const std::vector<SurfaceMesh>& meshes,
                                      bool perBody)
{
    std::vector<OutputObject> objects;
    std::vector<size_t> bodiesPerInstance; // per object
    std::map<std::string, size_t> byPart;  // part name -> object index
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (meshes[i].triangles.empty())
        {
            continue; // dropped: not written, and not an instance
        }
        if (perBody || bodies[i].part.empty())
        {
            objects.push_back({perBody ? bodies[i].name : std::string(), {i}});
            bodiesPerInstance.push_back(1);
            continue;
        }
        auto found = byPart.find(bodies[i].part);
        if (found == byPart.end())
        {
            found = byPart.emplace(bodies[i].part, objects.size()).first;
            objects.push_back({bodies[i].part, {}});
            bodiesPerInstance.push_back(std::max<size_t>(1, bodies[i].partBodies));
        }
        objects[found->second].bodies.push_back(i);
    }

    size_t unnamed = 0;
    std::map<std::string, int> nameUses;
    for (size_t k = 0; k < objects.size(); ++k)
    {
        std::string& name = objects[k].name;
        if (!perBody)
        {
            if (name.empty())
            {
                name = "mesh" + std::to_string(++unnamed);
            }
            const size_t instances = (objects[k].bodies.size() + bodiesPerInstance[k] - 1) / bodiesPerInstance[k];
            if (instances > 1)
            {
                name += "_instanced" + std::to_string(instances);
            }
        }
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
namespace
{
// A number as "%.17g" would print it (what an ostream with precision(17)
// writes), without the ostream's per-number overhead.
void Append(std::string& out, double value)
{
    char buffer[32];
    const auto end = std::to_chars(buffer, buffer + sizeof(buffer), value, std::chars_format::general, 17).ptr;
    out.append(buffer, end);
}

void Append(std::string& out, size_t value)
{
    char buffer[24];
    const auto end = std::to_chars(buffer, buffer + sizeof(buffer), value).ptr;
    out.append(buffer, end);
}

// Writes every non-empty object: its header, the text of each of its bodies
// ('format', run in parallel on batches of bodies) and its footer, in order.
void WriteObjects(std::ofstream& out, const std::vector<OutputObject>& objects,
                  const std::vector<SurfaceMesh>& meshes, const std::function<std::string(const OutputObject&)>& header,
                  const std::function<std::string(const OutputObject&)>& footer,
                  const std::function<void(size_t body, std::string& text)>& format)
{
    struct Piece
    {
        std::string before; // header of the object this body starts
        size_t body;
        std::string after; // footer of the object this body ends
    };
    std::vector<Piece> pieces;
    for (const OutputObject& object : objects)
    {
        if (std::all_of(object.bodies.begin(), object.bodies.end(),
                        [&](size_t i) { return meshes[i].triangles.empty(); }))
        {
            continue;
        }
        for (size_t k = 0; k < object.bodies.size(); ++k)
        {
            pieces.push_back({k == 0 ? header(object) : std::string(), object.bodies[k],
                              k + 1 == object.bodies.size() ? footer(object) : std::string()});
        }
    }
    // Batches keep the memory bounded: only one batch's text is held at once.
    constexpr size_t kBatch = 64;
    std::vector<std::string> text(kBatch);
    for (size_t first = 0; first < pieces.size(); first += kBatch)
    {
        const size_t count = std::min(kBatch, pieces.size() - first);
        OSD_Parallel::For(0, static_cast<int>(count), [&](int k) {
            text[k].clear();
            format(pieces[first + k].body, text[k]);
        });
        for (size_t k = 0; k < count; ++k)
        {
            out << pieces[first + k].before << text[k] << pieces[first + k].after;
        }
    }
}
} // namespace

void ExportStl(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
               const std::string& outputPath)
{
    std::ofstream out(outputPath);
    if (!out)
    {
        throw std::runtime_error("Cannot write STL file: " + outputPath);
    }
    WriteObjects(
        out, objects, meshes, [](const OutputObject& object) { return "solid " + object.name + "\n"; },
        [](const OutputObject& object) { return "endsolid " + object.name + "\n"; },
        [&](size_t body, std::string& text) {
            const SurfaceMesh& mesh = meshes[body];
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
                text += "  facet normal ";
                Append(text, normal.X());
                text += ' ';
                Append(text, normal.Y());
                text += ' ';
                Append(text, normal.Z());
                text += "\n    outer loop\n";
                for (const gp_Pnt* p : {&a, &b, &c})
                {
                    text += "      vertex ";
                    Append(text, p->X());
                    text += ' ';
                    Append(text, p->Y());
                    text += ' ';
                    Append(text, p->Z());
                    text += '\n';
                }
                text += "    endloop\n  endfacet\n";
            }
        });
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
    out << "# " << objects.size() << " objects, written by cad_converter\n";

    // OBJ vertex numbers start at 1 and run through the whole file.
    std::vector<size_t> offset(meshes.size(), 0);
    size_t next = 1;
    for (const OutputObject& object : objects)
    {
        if (std::all_of(object.bodies.begin(), object.bodies.end(),
                        [&](size_t i) { return meshes[i].triangles.empty(); }))
        {
            continue;
        }
        for (size_t i : object.bodies)
        {
            offset[i] = next;
            next += meshes[i].vertices.size();
        }
    }
    WriteObjects(
        out, objects, meshes, [](const OutputObject& object) { return "o " + object.name + "\n"; },
        [](const OutputObject&) { return std::string(); },
        [&](size_t body, std::string& text) {
            const SurfaceMesh& mesh = meshes[body];
            for (const gp_Pnt& p : mesh.vertices)
            {
                text += "v ";
                Append(text, p.X());
                text += ' ';
                Append(text, p.Y());
                text += ' ';
                Append(text, p.Z());
                text += '\n';
            }
            for (const auto& tri : mesh.triangles)
            {
                text += "f ";
                Append(text, tri[0] + offset[body]);
                text += ' ';
                Append(text, tri[1] + offset[body]);
                text += ' ';
                Append(text, tri[2] + offset[body]);
                text += '\n';
            }
        });
}
