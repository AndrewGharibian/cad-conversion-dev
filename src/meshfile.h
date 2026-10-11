#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct MeshFile
{
    using Vertex = std::array<double, 3>;
    using Triangle = std::array<std::uint32_t, 3>;

    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
};

bool ReadMeshFile(const std::string& path, MeshFile& mesh, std::string& error);
bool ValidateMesh(MeshFile& mesh, std::string& error);
