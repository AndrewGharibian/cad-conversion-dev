// =============================================================================
// mesh_check — check a cad_converter OBJ output independently of the
// converter's own report: what Geant4 needs from the solids.
//
//   mesh_check model.obj
//
// Use it on output written with --per-body, where every object is one body
// and solids are named solid_NNNN (shells and faces enclose no volume and
// are skipped). For every solid:
//   - every edge is used by exactly two triangles, in opposite directions
//     (a closed, consistently oriented surface);
//   - Manifold accepts it as a closed 2-manifold, with a positive volume.
// For every pair of solids whose boxes overlap: their meshes don't overlap
// (common volume at most 1e-12 of the smaller solid's, i.e. rounding noise).
// Prints what fails and exits with 1 if anything does.
// =============================================================================
#include <manifold/manifold.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{
constexpr double kOverlapFraction = 1.0e-12;

struct Object
{
    std::string name;
    std::vector<std::array<uint64_t, 3>> triangles; // indices into the file's vertex list
};

struct Solid
{
    std::string name;
    manifold::Manifold mesh;
    double volume = 0.0;
    std::array<double, 3> lo{}, hi{};
};
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " model.obj\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in)
    {
        std::cerr << "Cannot read " << argv[1] << "\n";
        return 2;
    }
    std::vector<double> vertices;
    std::vector<Object> objects;
    for (std::string line; std::getline(in, line);)
    {
        std::istringstream fields(line.size() > 2 ? line.substr(2) : std::string());
        if (line.rfind("v ", 0) == 0)
        {
            double x, y, z;
            fields >> x >> y >> z;
            vertices.insert(vertices.end(), {x, y, z});
        }
        else if (line.rfind("o ", 0) == 0)
        {
            objects.push_back({line.substr(2), {}});
        }
        else if (line.rfind("f ", 0) == 0 && !objects.empty())
        {
            uint64_t a, b, c;
            fields >> a >> b >> c;
            objects.back().triangles.push_back({a - 1, b - 1, c - 1});
        }
    }

    size_t failures = 0;
    if (objects.empty())
    {
        ++failures;
        std::cout << argv[1] << ": no objects\n";
    }
    std::vector<Solid> solids;
    for (const Object& object : objects)
    {
        if (object.name.rfind("solid", 0) != 0 || object.triangles.empty())
        {
            continue;
        }
        // Edge use: each directed edge once, each undirected edge twice.
        std::map<std::pair<uint64_t, uint64_t>, int> directed;
        uint64_t first = UINT64_MAX, last = 0;
        for (const auto& t : object.triangles)
        {
            for (int k = 0; k < 3; ++k)
            {
                ++directed[{t[k], t[(k + 1) % 3]}];
                first = std::min(first, t[k]);
                last = std::max(last, t[k]);
            }
        }
        size_t open = 0, nonManifold = 0, misoriented = 0;
        for (const auto& entry : directed)
        {
            misoriented += entry.second > 1;
            const auto reverse = directed.find({entry.first.second, entry.first.first});
            const int uses = entry.second + (reverse == directed.end() ? 0 : reverse->second);
            open += uses == 1;
            nonManifold += uses > 2;
        }
        if (open || nonManifold || misoriented)
        {
            ++failures;
            std::cout << object.name << ": " << open << " open edge(s), " << nonManifold
                      << " edge(s) used by more than two triangles, " << misoriented
                      << " edge(s) used twice in the same direction\n";
            continue;
        }

        manifold::MeshGL64 gl;
        gl.numProp = 3;
        gl.vertProperties.assign(vertices.begin() + 3 * first, vertices.begin() + 3 * (last + 1));
        for (const auto& t : object.triangles)
        {
            for (uint64_t v : t)
            {
                gl.triVerts.push_back(v - first);
            }
        }
        Solid solid{object.name, manifold::Manifold(gl)};
        if (solid.mesh.Status() != manifold::Manifold::Error::NoError || solid.mesh.Volume() <= 0.0)
        {
            ++failures;
            std::cout << object.name << ": not a closed, outward-facing 2-manifold (Manifold status "
                      << static_cast<int>(solid.mesh.Status()) << ", volume " << solid.mesh.Volume() << ")\n";
            continue;
        }
        solid.volume = solid.mesh.Volume();
        solid.lo = {gl.vertProperties[0], gl.vertProperties[1], gl.vertProperties[2]};
        solid.hi = solid.lo;
        for (size_t i = 0; i < gl.vertProperties.size(); i += 3)
        {
            for (int k = 0; k < 3; ++k)
            {
                solid.lo[k] = std::min(solid.lo[k], gl.vertProperties[i + k]);
                solid.hi[k] = std::max(solid.hi[k], gl.vertProperties[i + k]);
            }
        }
        solids.push_back(std::move(solid));
    }

    size_t pairs = 0;
    for (size_t i = 0; i < solids.size(); ++i)
    {
        for (size_t j = i + 1; j < solids.size(); ++j)
        {
            bool boxesOverlap = true;
            for (int k = 0; k < 3; ++k)
            {
                boxesOverlap = boxesOverlap && solids[i].lo[k] <= solids[j].hi[k] && solids[j].lo[k] <= solids[i].hi[k];
            }
            if (!boxesOverlap)
            {
                continue;
            }
            ++pairs;
            const double common = (solids[i].mesh ^ solids[j].mesh).Volume();
            if (common > kOverlapFraction * std::min(solids[i].volume, solids[j].volume))
            {
                ++failures;
                std::cout << solids[i].name << " and " << solids[j].name << " overlap by " << common << "\n";
            }
        }
    }

    std::cout << objects.size() << " object(s), " << solids.size() << " closed solid(s) checked, " << pairs
              << " touching pair(s) checked for overlaps: " << (failures ? "FAILED" : "OK") << " (" << failures
              << " problem(s))\n";
    return failures ? 1 : 0;
}
