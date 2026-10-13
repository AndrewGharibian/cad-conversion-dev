// =============================================================================
// geo_check — check a cad_converter --geo output (model.geo.setup and the
// .geo files it includes) the way MEGAlib/Geomega will read it.
//
//   geo_check model.geo.setup
//
// Checks:
//   - names: letters, digits and '_' only, unique ignoring case (volumes and
//     their <volume>_Shape shapes);
//   - every volume has a Material, a Mother, a Position and a TESSELLATED
//     shape, and its embedded mesh follows MDShapeTessellated's order rules:
//     "Shape TESSELLATED nv nf", "Vertices nv", nv "Vertex x y z", "Facets
//     nf", nf "Facet i j k", with the counts matching;
//   - every mesh passes MDShapeTessellated::CheckMesh (ported below, same
//     rules): at least 4 vertices and facets, valid indices, no degenerate
//     or repeated-index triangle, after welding identical vertices every edge
//     used exactly twice in opposite directions, positive enclosed volume,
//     extent in all three dimensions;
//   - the surrounding sphere encloses every vertex, and the world box the
//     sphere.
// Includes starting with $(MEGALIB) (the material list) are skipped.
// Prints what fails and exits with 1 if anything does.
// =============================================================================
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
using Vertex = std::array<double, 3>;
using Facet = std::array<unsigned int, 3>;

int failures = 0;

void Fail(const std::string& what)
{
    if (++failures <= 20)
    {
        std::cout << "FAILED: " << what << "\n";
    }
}

struct VolumeData
{
    std::string name;
    std::set<std::string> properties; // Material, Mother, Position, ...
    bool shapeDeclared = false;
    size_t declaredVertices = 0, declaredFacets = 0; // from "Shape TESSELLATED nv nf"
    size_t vertexHeader = 0, facetHeader = 0;        // from "Vertices n" / "Facets n"
    bool hasVertexHeader = false, hasFacetHeader = false;
    std::vector<Vertex> vertices;
    std::vector<Facet> facets;
};

std::string Lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return std::tolower(c); });
    return text;
}

bool ValidName(const std::string& name)
{
    return !name.empty() && std::all_of(name.begin(), name.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_';
    });
}

bool ParseDouble(const std::string& text, double& value)
{
    char* end = nullptr;
    value = std::strtod(text.c_str(), &end);
    return end != text.c_str() && *end == '\0' && std::isfinite(value);
}

bool ParseIndex(const std::string& text, size_t& value)
{
    if (text.empty() || !std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c); }))
    {
        return false;
    }
    value = std::strtoull(text.c_str(), nullptr, 10);
    return true;
}

// MDShapeTessellated::CheckMesh, with the same rules.
bool CheckMesh(std::vector<Vertex> vertices, std::vector<Facet> facets, std::string& error)
{
    if (vertices.size() < 4 || facets.size() < 4)
    {
        error = "a closed solid needs at least four vertices and four facets";
        return false;
    }
    std::vector<bool> used(vertices.size(), false);
    for (const Facet& f : facets)
    {
        for (unsigned int i : f)
        {
            if (i >= vertices.size())
            {
                error = "invalid facet vertex index";
                return false;
            }
            used[i] = true;
        }
    }
    std::map<Vertex, unsigned int> unique;
    std::vector<Vertex> welded;
    std::vector<unsigned int> remap(vertices.size());
    for (unsigned int i = 0; i < vertices.size(); ++i)
    {
        if (!used[i])
        {
            continue;
        }
        Vertex v = vertices[i];
        for (double& c : v)
        {
            if (c == 0.0)
            {
                c = 0.0;
            }
        }
        auto found = unique.find(v);
        if (found == unique.end())
        {
            remap[i] = static_cast<unsigned int>(welded.size());
            unique[v] = remap[i];
            welded.push_back(v);
        }
        else
        {
            remap[i] = found->second;
        }
    }
    for (Facet& f : facets)
    {
        for (unsigned int& i : f)
        {
            i = remap[i];
        }
    }
    vertices.swap(welded);
    if (vertices.size() < 4)
    {
        error = "a closed solid needs at least four distinct vertices";
        return false;
    }

    Vertex lo = vertices.front(), hi = vertices.front();
    double reference[3] = {0.0, 0.0, 0.0};
    for (const Vertex& v : vertices)
    {
        for (int a = 0; a < 3; ++a)
        {
            lo[a] = std::min(lo[a], v[a]);
            hi[a] = std::max(hi[a], v[a]);
            reference[a] += v[a] / vertices.size();
        }
    }
    std::map<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, int>> edges;
    long double sixVolume = 0.0L;
    for (const Facet& f : facets)
    {
        if (f[0] == f[1] || f[1] == f[2] || f[2] == f[0])
        {
            error = "a facet repeats a vertex (after welding)";
            return false;
        }
        const Vertex& a = vertices[f[0]];
        const Vertex& b = vertices[f[1]];
        const Vertex& c = vertices[f[2]];
        const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const double cross[3] = {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2],
                                 ab[0] * ac[1] - ab[1] * ac[0]};
        if (cross[0] == 0.0 && cross[1] == 0.0 && cross[2] == 0.0)
        {
            error = "degenerate triangle";
            return false;
        }
        const long double x[3] = {a[0] - reference[0], a[1] - reference[1], a[2] - reference[2]};
        const long double y[3] = {b[0] - reference[0], b[1] - reference[1], b[2] - reference[2]};
        const long double z[3] = {c[0] - reference[0], c[1] - reference[1], c[2] - reference[2]};
        sixVolume += x[0] * (y[1] * z[2] - y[2] * z[1]) + x[1] * (y[2] * z[0] - y[0] * z[2]) +
                     x[2] * (y[0] * z[1] - y[1] * z[0]);
        for (int k = 0; k < 3; ++k)
        {
            const unsigned int from = f[k], to = f[(k + 1) % 3];
            auto& entry = edges[{std::min(from, to), std::max(from, to)}];
            ++entry.first;
            entry.second += from < to ? 1 : -1;
        }
    }
    for (const auto& entry : edges)
    {
        if (entry.second.first != 2 || entry.second.second != 0)
        {
            error = "mesh is open, non-manifold, or has inconsistent facet winding";
            return false;
        }
    }
    if (!(sixVolume > 0.0L))
    {
        error = "facets don't enclose a positive volume (inside out?)";
        return false;
    }
    if (hi[0] == lo[0] || hi[1] == lo[1] || hi[2] == lo[2])
    {
        error = "mesh does not enclose a three-dimensional volume";
        return false;
    }
    return true;
}

class Reader
{
public:
    std::map<std::string, VolumeData> volumes; // by lower-case name
    std::vector<std::string> order;
    std::map<std::string, std::string> shapeOwner; // lower-case "<v>_shape" -> volume key
    double sphere[5] = {0, 0, 0, 0, 0};
    bool hasSphere = false;
    std::array<double, 3> world = {0, 0, 0};
    bool hasWorld = false;
    size_t files = 0;

    void Read(const fs::path& path)
    {
        std::ifstream in(path);
        if (!in)
        {
            Fail("cannot open " + path.string());
            return;
        }
        ++files;
        std::string line;
        size_t number = 0;
        while (std::getline(in, line))
        {
            ++number;
            const size_t comment = line.find("//");
            if (comment != std::string::npos)
            {
                line.resize(comment);
            }
            std::istringstream words(line);
            std::vector<std::string> t;
            for (std::string w; words >> w;)
            {
                t.push_back(w);
            }
            if (!t.empty())
            {
                Line(path, number, t);
            }
        }
    }

private:
    std::string Where(const fs::path& path, size_t number)
    {
        return path.filename().string() + ":" + std::to_string(number);
    }

    void Line(const fs::path& path, size_t number, const std::vector<std::string>& t)
    {
        if (t[0] == "Include")
        {
            if (t.size() != 2)
            {
                Fail(Where(path, number) + ": Include needs one file");
            }
            else if (t[1].rfind("$(", 0) != 0)
            {
                Read(path.parent_path() / t[1]);
            }
            return;
        }
        if (t[0] == "SurroundingSphere")
        {
            hasSphere = t.size() == 6;
            for (size_t k = 1; k < t.size() && k < 6; ++k)
            {
                hasSphere = ParseDouble(t[k], sphere[k - 1]) && hasSphere;
            }
            if (!hasSphere || sphere[0] != sphere[4])
            {
                Fail(Where(path, number) + ": SurroundingSphere needs radius, centre and an equal distance");
            }
            return;
        }
        if (t[0] == "Volume")
        {
            if (t.size() != 2 || !ValidName(t[1]))
            {
                Fail(Where(path, number) + ": invalid volume name");
                return;
            }
            const std::string key = Lower(t[1]);
            if (key == "world")
            {
                return; // the world box, checked separately
            }
            if (volumes.count(key) || shapeOwner.count(key))
            {
                Fail(Where(path, number) + ": name used twice (ignoring case): " + t[1]);
                return;
            }
            volumes[key].name = t[1];
            order.push_back(key);
            shapeOwner[Lower(t[1] + "_Shape")] = key;
            return;
        }
        const size_t dot = t[0].find('.');
        if (dot == std::string::npos)
        {
            Fail(Where(path, number) + ": unknown line: " + t[0]);
            return;
        }
        const std::string owner = Lower(t[0].substr(0, dot));
        const std::string property = t[0].substr(dot + 1);
        if (owner == "world")
        {
            if (property == "Shape" && t.size() == 5 && Lower(t[1]) == "brik")
            {
                hasWorld = ParseDouble(t[2], world[0]) && ParseDouble(t[3], world[1]) && ParseDouble(t[4], world[2]);
            }
            return;
        }
        if (volumes.count(owner))
        {
            VolumeData& v = volumes[owner];
            v.properties.insert(property);
            if (property == "Shape")
            {
                if (t.size() != 4 || Lower(t[1]) != "tessellated" || !ParseIndex(t[2], v.declaredVertices) ||
                    !ParseIndex(t[3], v.declaredFacets))
                {
                    Fail(Where(path, number) + ": expected " + v.name + ".Shape TESSELLATED <nv> <nf>");
                }
                v.shapeDeclared = true;
            }
            return;
        }
        auto shape = shapeOwner.find(owner);
        if (shape == shapeOwner.end())
        {
            Fail(Where(path, number) + ": unknown volume or shape: " + t[0].substr(0, dot));
            return;
        }
        VolumeData& v = volumes[shape->second];
        if (!v.shapeDeclared)
        {
            Fail(Where(path, number) + ": mesh data before " + v.name + ".Shape TESSELLATED");
            return;
        }
        if (property == "Vertices")
        {
            if (t.size() != 2 || v.hasVertexHeader || !ParseIndex(t[1], v.vertexHeader) || !v.vertices.empty())
            {
                Fail(Where(path, number) + ": bad Vertices line");
            }
            v.hasVertexHeader = true;
        }
        else if (property == "Vertex")
        {
            Vertex p;
            if (t.size() != 4 || !v.hasVertexHeader || v.vertices.size() >= v.vertexHeader ||
                !ParseDouble(t[1], p[0]) || !ParseDouble(t[2], p[1]) || !ParseDouble(t[3], p[2]))
            {
                Fail(Where(path, number) + ": bad Vertex line");
                return;
            }
            v.vertices.push_back(p);
        }
        else if (property == "Facets")
        {
            if (t.size() != 2 || v.hasFacetHeader || !ParseIndex(t[1], v.facetHeader) ||
                v.vertices.size() != v.vertexHeader)
            {
                Fail(Where(path, number) + ": bad Facets line (or not after all vertices)");
            }
            v.hasFacetHeader = true;
        }
        else if (property == "Facet")
        {
            size_t i[3];
            if (t.size() != 4 || !v.hasFacetHeader || v.facets.size() >= v.facetHeader || !ParseIndex(t[1], i[0]) ||
                !ParseIndex(t[2], i[1]) || !ParseIndex(t[3], i[2]))
            {
                Fail(Where(path, number) + ": bad Facet line");
                return;
            }
            v.facets.push_back({static_cast<unsigned int>(i[0]), static_cast<unsigned int>(i[1]),
                                static_cast<unsigned int>(i[2])});
        }
        else
        {
            Fail(Where(path, number) + ": unknown shape property " + property);
        }
    }
};
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: geo_check model.geo.setup\n";
        return 2;
    }
    Reader reader;
    reader.Read(argv[1]);

    size_t vertices = 0, facets = 0;
    double reach = 0.0; // largest distance of a vertex from the sphere's centre
    for (const std::string& key : reader.order)
    {
        const VolumeData& v = reader.volumes[key];
        for (const char* property : {"Material", "Mother", "Position", "Shape"})
        {
            if (!v.properties.count(property))
            {
                Fail(v.name + ": no " + property);
            }
        }
        if (v.vertices.size() != v.declaredVertices || v.vertexHeader != v.declaredVertices ||
            v.facets.size() != v.declaredFacets || v.facetHeader != v.declaredFacets)
        {
            Fail(v.name + ": vertex or facet count doesn't match its declaration");
            continue;
        }
        std::string error;
        if (!CheckMesh(v.vertices, v.facets, error))
        {
            Fail(v.name + ": " + error);
        }
        vertices += v.vertices.size();
        facets += v.facets.size();
        for (const Vertex& p : v.vertices)
        {
            reach = std::max(reach, std::sqrt(std::pow(p[0] - reader.sphere[1], 2) + std::pow(p[1] - reader.sphere[2], 2) +
                                              std::pow(p[2] - reader.sphere[3], 2)));
        }
    }
    if (reader.order.empty())
    {
        Fail("no volumes");
    }
    if (!reader.hasSphere || reach > reader.sphere[0])
    {
        Fail("the surrounding sphere doesn't enclose the model");
    }
    for (int a = 0; a < 3 && reader.hasSphere; ++a)
    {
        if (!reader.hasWorld || std::fabs(reader.sphere[a + 1]) + reader.sphere[0] > reader.world[a])
        {
            Fail("the world box doesn't enclose the surrounding sphere");
            break;
        }
    }
    std::cout << reader.files << " file(s), " << reader.order.size() << " TESSELLATED volume(s), " << vertices
              << " vertices, " << facets << " facets: " << (failures ? "FAILED" : "OK") << " (" << failures
              << " problem(s))\n";
    return failures ? 1 : 0;
}
