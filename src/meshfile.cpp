#include "meshfile.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <limits>
#include <map>
#include <sstream>
#include <utility>

namespace
{
static_assert(sizeof(float) == sizeof(std::uint32_t), "STL coordinates require 32-bit floats");

bool ReadBinarySTL(const std::string& path, MeshFile& mesh, std::string& error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        error = "unable to open mesh file";
        return false;
    }

    input.seekg(80);
    unsigned char countBytes[4];
    if (!input.read(reinterpret_cast<char*>(countBytes), sizeof(countBytes)))
    {
        error = "binary STL header is incomplete";
        return false;
    }

    const std::uint32_t count =
        static_cast<std::uint32_t>(countBytes[0]) |
        (static_cast<std::uint32_t>(countBytes[1]) << 8) |
        (static_cast<std::uint32_t>(countBytes[2]) << 16) |
        (static_cast<std::uint32_t>(countBytes[3]) << 24);
    if (count > std::numeric_limits<std::uint32_t>::max() / 3)
    {
        error = "binary STL contains too many vertices";
        return false;
    }
    const std::uint64_t expectedSize = 84ULL + 50ULL * count;
    input.seekg(0, std::ios::end);
    const std::streamoff size = input.tellg();
    if (size < 0 || static_cast<std::uint64_t>(size) != expectedSize)
    {
        error = "binary STL size does not match its triangle count";
        return false;
    }

    input.seekg(84);
    mesh.vertices.clear();
    mesh.triangles.clear();
    mesh.vertices.reserve(static_cast<std::size_t>(count) * 3);
    mesh.triangles.reserve(count);

    for (std::uint32_t i = 0; i < count; ++i)
    {
        unsigned char record[50];
        if (!input.read(reinterpret_cast<char*>(record), sizeof(record)))
        {
            error = "binary STL contains a truncated triangle";
            return false;
        }

        MeshFile::Triangle triangle;
        for (unsigned int vertex = 0; vertex < 3; ++vertex)
        {
            MeshFile::Vertex point;
            for (unsigned int axis = 0; axis < 3; ++axis)
            {
                const unsigned int offset = 12 + vertex * 12 + axis * 4;
                const std::uint32_t bits =
                    static_cast<std::uint32_t>(record[offset]) |
                    (static_cast<std::uint32_t>(record[offset + 1]) << 8) |
                    (static_cast<std::uint32_t>(record[offset + 2]) << 16) |
                    (static_cast<std::uint32_t>(record[offset + 3]) << 24);
                float coordinate;
                std::memcpy(&coordinate, &bits, sizeof(coordinate));
                point[axis] = coordinate;
            }
            triangle[vertex] = static_cast<std::uint32_t>(mesh.vertices.size());
            mesh.vertices.push_back(point);
        }
        mesh.triangles.push_back(triangle);
    }
    return true;
}

bool ReadASCIISTL(const std::string& path, MeshFile& mesh, std::string& error)
{
    std::ifstream input(path);
    if (!input)
    {
        error = "unable to open mesh file";
        return false;
    }

    mesh.vertices.clear();
    mesh.triangles.clear();
    std::vector<MeshFile::Vertex> facetVertices;
    std::string line;
    while (std::getline(input, line))
    {
        std::istringstream lineInput(line);
        std::string token;
        while (lineInput >> token)
        {
            if (token != "vertex")
                continue;

            MeshFile::Vertex point;
            if (!(lineInput >> point[0] >> point[1] >> point[2]))
            {
                error = "ASCII STL has an incomplete vertex";
                return false;
            }
            if (facetVertices.size() == std::numeric_limits<std::uint32_t>::max())
            {
                error = "ASCII STL contains too many vertices";
                return false;
            }
            facetVertices.push_back(point);
            break;
        }
    }

    if (facetVertices.empty() || facetVertices.size() % 3 != 0)
    {
        error = "ASCII STL must contain complete triangular facets";
        return false;
    }
    mesh.vertices.reserve(facetVertices.size());
    mesh.triangles.reserve(facetVertices.size() / 3);
    for (std::size_t i = 0; i < facetVertices.size(); i += 3)
    {
        MeshFile::Triangle triangle;
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            triangle[vertex] = static_cast<std::uint32_t>(mesh.vertices.size());
            mesh.vertices.push_back(facetVertices[i + vertex]);
        }
        mesh.triangles.push_back(triangle);
    }
    return true;
}

bool ParseOBJIndex(const std::string& token, std::size_t vertexCount,
                   std::uint32_t& index)
{
    const std::size_t slash = token.find('/');
    const std::string number = token.substr(0, slash);
    if (number.empty())
        return false;

    errno = 0;
    char* end = nullptr;
    const long long parsed = std::strtoll(number.c_str(), &end, 10);
    if (errno == ERANGE || end == number.c_str() || *end != '\0' || parsed == 0)
        return false;

    long long resolved;
    if (parsed > 0)
    {
        resolved = parsed - 1;
    }
    else
    {
        if (vertexCount > static_cast<std::size_t>(std::numeric_limits<long long>::max()))
            return false;
        resolved = static_cast<long long>(vertexCount) + parsed;
    }

    if (resolved < 0 || static_cast<unsigned long long>(resolved) >= vertexCount ||
        static_cast<unsigned long long>(resolved) >
            std::numeric_limits<std::uint32_t>::max())
        return false;
    index = static_cast<std::uint32_t>(resolved);
    return true;
}

bool ReadOBJ(const std::string& path, MeshFile& mesh, std::string& error)
{
    std::vector<NamedMesh> objects;
    if (!ReadOBJObjects(path, objects, error))
        return false;

    mesh.vertices.clear();
    mesh.triangles.clear();
    for (const NamedMesh& object : objects)
    {
        if (object.mesh.vertices.size() >
            std::numeric_limits<std::uint32_t>::max() - mesh.vertices.size())
        {
            error = "OBJ contains too many vertices";
            return false;
        }
        const std::uint32_t vertexOffset =
            static_cast<std::uint32_t>(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(), object.mesh.vertices.begin(),
                             object.mesh.vertices.end());
        for (const MeshFile::Triangle& triangle : object.mesh.triangles)
        {
            mesh.triangles.push_back({triangle[0] + vertexOffset,
                                      triangle[1] + vertexOffset,
                                      triangle[2] + vertexOffset});
        }
    }
    return true;
}

}

bool ReadOBJObjects(const std::string& path, std::vector<NamedMesh>& meshes,
                    std::string& error)
{
    std::ifstream input(path);
    if (!input)
    {
        error = "unable to open mesh file";
        return false;
    }

    struct RawObject
    {
        std::string name = "Mesh";
        std::vector<MeshFile::Triangle> triangles;
    };

    std::vector<MeshFile::Vertex> vertices;
    std::vector<RawObject> rawObjects;
    RawObject currentObject;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line))
    {
        ++lineNumber;
        std::istringstream lineInput(line);
        std::string kind;
        if (!(lineInput >> kind) || kind[0] == '#')
            continue;

        if (kind == "o")
        {
            if (!currentObject.triangles.empty())
                rawObjects.push_back(std::move(currentObject));
            currentObject = RawObject{};
            std::getline(lineInput, currentObject.name);
            const std::size_t first = currentObject.name.find_first_not_of(" \t");
            if (first == std::string::npos)
                currentObject.name = "Mesh";
            else
            {
                currentObject.name.erase(0, first);
                const std::size_t last = currentObject.name.find_last_not_of(" \t\r");
                currentObject.name.erase(last + 1);
            }
        }
        else if (kind == "v")
        {
            MeshFile::Vertex vertex;
            if (!(lineInput >> vertex[0] >> vertex[1] >> vertex[2]))
            {
                error = "OBJ has an invalid vertex on line " + std::to_string(lineNumber);
                return false;
            }
            if (vertices.size() == std::numeric_limits<std::uint32_t>::max())
            {
                error = "OBJ contains too many vertices";
                return false;
            }
            vertices.push_back(vertex);
        }
        else if (kind == "f")
        {
            std::vector<std::uint32_t> indices;
            std::string token;
            while (lineInput >> token)
            {
                if (!token.empty() && token[0] == '#')
                    break;
                std::uint32_t index = 0;
                if (!ParseOBJIndex(token, vertices.size(), index))
                {
                    error = "OBJ has an invalid face index on line " +
                            std::to_string(lineNumber);
                    return false;
                }
                indices.push_back(index);
            }
            if (indices.size() < 3)
            {
                error = "OBJ face has fewer than three vertices on line " +
                        std::to_string(lineNumber);
                return false;
            }
            const std::size_t maxTriangles = std::numeric_limits<std::uint32_t>::max();
            if (currentObject.triangles.size() > maxTriangles ||
                indices.size() - 2 > maxTriangles - currentObject.triangles.size())
            {
                error = "OBJ contains too many triangles";
                return false;
            }
            for (std::size_t i = 1; i + 1 < indices.size(); ++i)
                currentObject.triangles.push_back({indices[0], indices[i], indices[i + 1]});
        }
    }

    if (!currentObject.triangles.empty())
        rawObjects.push_back(std::move(currentObject));
    if (rawObjects.empty())
    {
        error = "OBJ contains no faces";
        return false;
    }

    meshes.clear();
    for (const RawObject& rawObject : rawObjects)
    {
        NamedMesh object;
        object.name = rawObject.name;
        std::map<std::uint32_t, std::uint32_t> remap;
        for (const MeshFile::Triangle& triangle : rawObject.triangles)
        {
            MeshFile::Triangle localizedTriangle;
            for (unsigned int i = 0; i < 3; ++i)
            {
                auto found = remap.find(triangle[i]);
                if (found == remap.end())
                {
                    const auto nextIndex =
                        static_cast<std::uint32_t>(object.mesh.vertices.size());
                    object.mesh.vertices.push_back(vertices[triangle[i]]);
                    found = remap.emplace(triangle[i], nextIndex).first;
                }
                localizedTriangle[i] = found->second;
            }
            object.mesh.triangles.push_back(localizedTriangle);
        }
        meshes.push_back(std::move(object));
    }
    return true;
}

bool ReadMeshFile(const std::string& path, MeshFile& mesh, std::string& error)
{
    std::string extension = std::filesystem::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (extension == ".obj")
        return ReadOBJ(path, mesh, error);
    if (extension != ".stl")
    {
        error = "mesh file extension must be .stl or .obj";
        return false;
    }

    std::ifstream probe(path, std::ios::binary);
    if (!probe)
    {
        error = "unable to open mesh file";
        return false;
    }
    probe.seekg(0, std::ios::end);
    const std::streamoff size = probe.tellg();
    bool isBinary = false;
    if (size >= 84)
    {
        probe.seekg(80);
        unsigned char countBytes[4];
        if (probe.read(reinterpret_cast<char*>(countBytes), sizeof(countBytes)))
        {
            const std::uint32_t count =
                static_cast<std::uint32_t>(countBytes[0]) |
                (static_cast<std::uint32_t>(countBytes[1]) << 8) |
                (static_cast<std::uint32_t>(countBytes[2]) << 16) |
                (static_cast<std::uint32_t>(countBytes[3]) << 24);
            isBinary = 84ULL + 50ULL * count == static_cast<std::uint64_t>(size);
        }
    }
    return isBinary ? ReadBinarySTL(path, mesh, error) :
                      ReadASCIISTL(path, mesh, error);
}

bool ValidateMesh(MeshFile& mesh, std::string& error)
{
    if (mesh.vertices.size() < 4 || mesh.triangles.size() < 4)
    {
        error = "a closed solid needs at least four vertices and four facets";
        return false;
    }
    if (mesh.vertices.size() > std::numeric_limits<std::uint32_t>::max())
    {
        error = "mesh contains too many vertices";
        return false;
    }
    if (mesh.triangles.size() > std::numeric_limits<std::uint32_t>::max())
    {
        error = "mesh contains too many facets";
        return false;
    }

    std::vector<bool> used(mesh.vertices.size(), false);
    for (const MeshFile::Triangle& triangle : mesh.triangles)
    {
        for (std::uint32_t index : triangle)
        {
            if (index >= mesh.vertices.size())
            {
                error = "mesh contains an invalid facet vertex index";
                return false;
            }
            used[index] = true;
        }
    }

    std::map<MeshFile::Vertex, std::uint32_t> uniqueVertices;
    std::vector<MeshFile::Vertex> weldedVertices;
    std::vector<std::uint32_t> remap(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
    {
        if (!used[i])
            continue;
        MeshFile::Vertex vertex = mesh.vertices[i];
        for (double& coordinate : vertex)
        {
            if (!std::isfinite(coordinate))
            {
                error = "mesh contains a non-finite coordinate";
                return false;
            }
            if (coordinate == 0.0)
                coordinate = 0.0;
        }
        auto found = uniqueVertices.find(vertex);
        if (found == uniqueVertices.end())
        {
            remap[i] = static_cast<std::uint32_t>(weldedVertices.size());
            uniqueVertices.emplace(vertex, remap[i]);
            weldedVertices.push_back(vertex);
        }
        else
        {
            remap[i] = found->second;
        }
    }
    for (MeshFile::Triangle& triangle : mesh.triangles)
    {
        for (std::uint32_t& index : triangle)
            index = remap[index];
    }
    mesh.vertices.swap(weldedVertices);
    if (mesh.vertices.size() < 4)
    {
        error = "a closed solid needs at least four distinct vertices";
        return false;
    }

    using Edge = std::pair<std::uint32_t, std::uint32_t>;
    std::map<Edge, std::pair<unsigned int, int>> edges;
    const MeshFile::Vertex& first = mesh.vertices.front();
    MeshFile::Vertex minimum = first;
    MeshFile::Vertex maximum = first;
    MeshFile::Vertex reference = {0.0, 0.0, 0.0};
    for (const MeshFile::Vertex& vertex : mesh.vertices)
    {
        for (unsigned int axis = 0; axis < 3; ++axis)
        {
            minimum[axis] = std::min(minimum[axis], vertex[axis]);
            maximum[axis] = std::max(maximum[axis], vertex[axis]);
            reference[axis] += vertex[axis] / mesh.vertices.size();
        }
    }

    long double sixVolume = 0.0L;
    for (const MeshFile::Triangle& triangle : mesh.triangles)
    {
        if (triangle[0] == triangle[1] || triangle[1] == triangle[2] ||
            triangle[2] == triangle[0])
        {
            error = "mesh contains a facet with repeated vertices";
            return false;
        }
        const MeshFile::Vertex& a = mesh.vertices[triangle[0]];
        const MeshFile::Vertex& b = mesh.vertices[triangle[1]];
        const MeshFile::Vertex& c = mesh.vertices[triangle[2]];
        const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const double cross[3] = {
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]
        };
        if (cross[0] == 0.0 && cross[1] == 0.0 && cross[2] == 0.0)
        {
            error = "mesh contains a degenerate triangle";
            return false;
        }

        const long double x[3] = {a[0] - reference[0], a[1] - reference[1],
                                  a[2] - reference[2]};
        const long double y[3] = {b[0] - reference[0], b[1] - reference[1],
                                  b[2] - reference[2]};
        const long double z[3] = {c[0] - reference[0], c[1] - reference[1],
                                  c[2] - reference[2]};
        sixVolume += x[0] * (y[1] * z[2] - y[2] * z[1]) +
                     x[1] * (y[2] * z[0] - y[0] * z[2]) +
                     x[2] * (y[0] * z[1] - y[1] * z[0]);

        for (unsigned int i = 0; i < 3; ++i)
        {
            const std::uint32_t from = triangle[i];
            const std::uint32_t to = triangle[(i + 1) % 3];
            auto& edge = edges[{std::min(from, to), std::max(from, to)}];
            ++edge.first;
            edge.second += from < to ? 1 : -1;
        }
    }

    for (const auto& edge : edges)
    {
        if (edge.second.first != 2 || edge.second.second != 0)
        {
            error = "mesh is open, non-manifold, or has inconsistent facet winding";
            return false;
        }
    }
    if (!(sixVolume > 0.0L) || !std::isfinite(static_cast<double>(sixVolume)))
    {
        error = "mesh facets must have an outward orientation and enclose positive volume";
        return false;
    }
    if (maximum[0] == minimum[0] || maximum[1] == minimum[1] ||
        maximum[2] == minimum[2])
    {
        error = "mesh does not enclose a three-dimensional volume";
        return false;
    }
    return true;
}
