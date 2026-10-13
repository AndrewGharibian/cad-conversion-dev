// =============================================================================
// geomega_export.cpp — write the objects as a MEGAlib/Geomega geometry.
//
// Layout (--geo), for model.step:
//   model.geo.setup        the geometry MEGAlib loads: the material list, the
//                          surrounding sphere, the world volume, and for each
//                          object an Include of its mesh file followed by
//                          each of its volumes' Material, Color and Visibility
//                          (the materials are placeholders, to be set there);
//   model_geo/<object>.geo one per object: a volume per body (placement),
//                          each with its TESSELLATED shape and its mesh.
//
// A Geomega volume has one shape, and MEGAlib needs a point inside every
// volume, so each body is its own volume: a part placed 12 times gives 12
// volumes (<object>_1 ... <object>_12) in one file. A single-body object's
// volume is named after the object.
//
// The shape format is the one MDShapeTessellated parses (embedded mesh):
//   V.Shape TESSELLATED <nVertices> <nFacets>
//   V_Shape.Vertices <nVertices>
//   V_Shape.Vertex <x> <y> <z>          (nVertices lines)
//   V_Shape.Facets <nFacets>
//   V_Shape.Facet <i> <j> <k>           (nFacets lines, 0-based, counter-
//                                        clockwise seen from outside)
// Geomega lengths are in cm; OCCT reads STEP models in mm, so coordinates are
// divided by 10. Each is written with the fewest digits that read back as the
// same double, so vertices shared by touching bodies stay bit-identical.
// Every volume sits in World at position 0 (the mesh is in model
// coordinates).
//
// Names: Geomega allows only letters, digits and '_', and names must be
// unique ignoring case (they are also file names, on case-insensitive file
// systems), so other characters become '_' and clashes get a _2, _3, ...
// suffix. A volume V also takes the shape name V_Shape. Mesh file names are
// the object's name cut to kMaxFileStem characters (part names can be long,
// and Windows paths are limited to 260 characters by default), also unique.
// =============================================================================
#include "geomega_export.hpp"

#include <OSD_Parallel.hxx>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>

namespace fs = std::filesystem;

namespace
{
constexpr double kMillimetresPerCentimetre = 10.0;
constexpr const char* kPlaceholderMaterial = "Aluminium";
constexpr size_t kMaxFileStem = 48;

// The shortest text that reads back as exactly 'value'.
void Append(std::string& out, double value)
{
    char buffer[32];
    out.append(buffer, std::to_chars(buffer, buffer + sizeof(buffer), value).ptr);
}

void Append(std::string& out, size_t value)
{
    char buffer[24];
    out.append(buffer, std::to_chars(buffer, buffer + sizeof(buffer), value).ptr);
}

std::string ToLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
    return text;
}

std::string GeomegaName(const std::string& name)
{
    std::string valid = name;
    for (char& c : valid)
    {
        if (!std::isalnum(static_cast<unsigned char>(c)))
        {
            c = '_';
        }
    }
    return valid.empty() ? "Object" : valid;
}

// Unique names, ignoring case. For volumes, a name also reserves
// <name>_Shape.
class NameRegistry
{
public:
    explicit NameRegistry(bool volumes) : volumes_(volumes)
    {
        if (volumes_)
        {
            Reserve("World");
        }
    }

    std::string Claim(const std::string& base)
    {
        for (int k = 1;; ++k)
        {
            const std::string name = k == 1 ? base : base + "_" + std::to_string(k);
            if (!used_.count(ToLower(name)) && !(volumes_ && used_.count(ToLower(name + "_Shape"))))
            {
                Reserve(name);
                return name;
            }
        }
    }

private:
    void Reserve(const std::string& name)
    {
        used_.insert(ToLower(name));
        if (volumes_)
        {
            used_.insert(ToLower(name + "_Shape"));
        }
    }

    bool volumes_;
    std::set<std::string> used_;
};

struct Volume
{
    std::string name;
    size_t body;
};

struct ObjectFile
{
    std::string label; // the object's name in the other outputs
    std::string name;  // its Geomega name: its volume's, or the base of its volumes'
    std::string file;  // mesh file name, without .geo
    std::vector<Volume> volumes;
};

// One body's volume: placement, shape and mesh. Only vertices used by a
// triangle are written, renumbered in order of first use.
void AppendVolume(std::string& text, const Volume& volume, const SurfaceMesh& mesh)
{
    std::vector<int> index(mesh.vertices.size(), -1);
    std::vector<size_t> order;
    for (const auto& tri : mesh.triangles)
    {
        for (int v : tri)
        {
            if (index[v] < 0)
            {
                index[v] = static_cast<int>(order.size());
                order.push_back(static_cast<size_t>(v));
            }
        }
    }
    const std::string& v = volume.name;
    text += "Volume " + v + "\n";
    text += v + ".Mother World\n";
    text += v + ".Position 0 0 0\n";
    text += v + ".Shape TESSELLATED ";
    Append(text, order.size());
    text += ' ';
    Append(text, mesh.triangles.size());
    text += "\n" + v + "_Shape.Vertices ";
    Append(text, order.size());
    text += '\n';
    const std::string vertex = v + "_Shape.Vertex ";
    for (size_t i : order)
    {
        const gp_Pnt& p = mesh.vertices[i];
        text += vertex;
        Append(text, p.X() / kMillimetresPerCentimetre);
        text += ' ';
        Append(text, p.Y() / kMillimetresPerCentimetre);
        text += ' ';
        Append(text, p.Z() / kMillimetresPerCentimetre);
        text += '\n';
    }
    text += v + "_Shape.Facets ";
    Append(text, mesh.triangles.size());
    text += '\n';
    const std::string facet = v + "_Shape.Facet ";
    for (const auto& tri : mesh.triangles)
    {
        text += facet;
        Append(text, static_cast<size_t>(index[tri[0]]));
        text += ' ';
        Append(text, static_cast<size_t>(index[tri[1]]));
        text += ' ';
        Append(text, static_cast<size_t>(index[tri[2]]));
        text += '\n';
    }
}

bool WriteFile(const fs::path& path, const std::string& text)
{
    std::ofstream out(path);
    return out && (out << text);
}
} // namespace

GeomegaReport ExportGeomega(const std::vector<OutputObject>& objects, const std::vector<SurfaceMesh>& meshes,
                            const std::string& setupPath, const std::string& source)
{
    const fs::path setup(setupPath);
    std::string model = setup.filename().string();
    const std::string suffix = ".geo.setup";
    if (model.size() > suffix.size() && model.compare(model.size() - suffix.size(), suffix.size(), suffix) == 0)
    {
        model.resize(model.size() - suffix.size());
    }
    const std::string meshDir = model + "_geo";
    fs::create_directories(setup.parent_path() / meshDir);

    // Names: the objects' names first, then the volumes'.
    NameRegistry names(true), fileNames(false);
    std::vector<ObjectFile> files;
    for (const OutputObject& object : objects)
    {
        std::vector<size_t> bodies;
        for (size_t i : object.bodies)
        {
            if (!meshes[i].triangles.empty())
            {
                bodies.push_back(i);
            }
        }
        if (!bodies.empty())
        {
            const std::string name = names.Claim(GeomegaName(object.name));
            files.push_back({object.name, name, fileNames.Claim(name.substr(0, kMaxFileStem)), {}});
            for (size_t i : bodies)
            {
                files.back().volumes.push_back({std::string(), i});
            }
        }
    }
    GeomegaReport report;
    report.meshDir = (setup.parent_path() / meshDir).make_preferred().string();
    for (ObjectFile& file : files)
    {
        for (size_t k = 0; k < file.volumes.size(); ++k)
        {
            file.volumes[k].name =
                file.volumes.size() == 1 ? file.name : names.Claim(file.name + "_" + std::to_string(k + 1));
        }
        report.volumes += file.volumes.size();
    }

    // The mesh files, in parallel (each is independent).
    std::vector<char> written(files.size(), 0);
    OSD_Parallel::For(0, static_cast<int>(files.size()), [&](int f) {
        const ObjectFile& file = files[f];
        std::string text = "// " + file.label + ": " + std::to_string(file.volumes.size()) +
                           " solid(s), written by cad_converter from " + source + "\n" +
                           "// Lengths in cm. Included by " + model + ".geo.setup, which sets the materials.\n";
        for (const Volume& volume : file.volumes)
        {
            text += '\n';
            AppendVolume(text, volume, meshes[volume.body]);
        }
        written[f] = WriteFile(setup.parent_path() / meshDir / (file.file + ".geo"), text) ? 1 : 0;
    });
    for (size_t f = 0; f < files.size(); ++f)
    {
        if (!written[f])
        {
            throw std::runtime_error("Cannot write Geomega file: " + files[f].file + ".geo in " + report.meshDir);
        }
    }

    // The surrounding sphere (where MEGAlib starts particles) encloses the
    // model; the world, a cube around the origin, encloses the sphere.
    double lo[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max()};
    double hi[3] = {-lo[0], -lo[1], -lo[2]};
    for (const ObjectFile& file : files)
    {
        for (const Volume& volume : file.volumes)
        {
            for (const gp_Pnt& p : meshes[volume.body].vertices)
            {
                const double c[3] = {p.X() / kMillimetresPerCentimetre, p.Y() / kMillimetresPerCentimetre,
                                     p.Z() / kMillimetresPerCentimetre};
                for (int a = 0; a < 3; ++a)
                {
                    lo[a] = std::min(lo[a], c[a]);
                    hi[a] = std::max(hi[a], c[a]);
                }
            }
        }
    }
    double centre[3] = {0.0, 0.0, 0.0}, radius = 1.0, world = 1.0;
    if (!files.empty())
    {
        double diagonal2 = 0.0;
        for (int a = 0; a < 3; ++a)
        {
            centre[a] = 0.5 * (lo[a] + hi[a]);
            diagonal2 += (hi[a] - lo[a]) * (hi[a] - lo[a]);
        }
        radius = std::ceil(0.5 * std::sqrt(diagonal2) + 1.0); // whole cm, at least 1 cm clearance
        for (int a = 0; a < 3; ++a)
        {
            world = std::max(world, std::ceil(1.1 * (std::fabs(centre[a]) + radius)));
        }
    }

    std::string text = "// " + model + ".geo.setup: written by cad_converter from " + source + "\n" +
                       "// Every solid is a TESSELLATED volume; the meshes are in " + meshDir +
                       "/, one file per object.\n"
                       "// Lengths in cm (the model's mm / 10). The materials are placeholders (" +
                       kPlaceholderMaterial + "): set them below.\n\n" +
                       "Include $(MEGALIB)/resource/examples/geomega/materials/Materials.geo\n\n";
    text += "SurroundingSphere ";
    Append(text, radius);
    for (double c : centre)
    {
        text += ' ';
        Append(text, c);
    }
    text += ' ';
    Append(text, radius);
    text += "\n\nVolume World\nWorld.Material Vacuum\nWorld.Shape BRIK ";
    for (int a = 0; a < 3; ++a)
    {
        Append(text, world);
        text += a < 2 ? " " : "\n";
    }
    text += "World.Visibility 0\nWorld.Mother 0\n";
    text += "\n// Each object's mesh file, then its volumes' properties.\n";
    for (size_t f = 0; f < files.size(); ++f)
    {
        const ObjectFile& file = files[f];
        text += "\n// " + file.label + "\nInclude " + meshDir + "/" + file.file + ".geo\n";
        const std::string color = std::to_string(2 + f % 8); // ROOT colours 2-9
        for (const Volume& volume : file.volumes)
        {
            text += volume.name + ".Material " + kPlaceholderMaterial + "\n";
            text += volume.name + ".Color " + color + "\n";
            text += volume.name + ".Visibility 1\n";
        }
    }
    if (!WriteFile(setup, text))
    {
        throw std::runtime_error("Cannot write Geomega file: " + setup.string());
    }
    return report;
}
