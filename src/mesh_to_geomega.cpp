#include "meshfile.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <string>

namespace
{
constexpr double UnitScale = 0.1;

std::string SanitizeName(const std::string& raw, std::set<std::string>& usedNames)
{
    std::string name;
    for (unsigned char c : raw)
    {
        if (std::isalnum(c))
            name += static_cast<char>(c);
        else if (c == '_' || c == '-')
            name += '_';
    }
    if (name.empty())
        name = "Mesh";
    if (std::isdigit(static_cast<unsigned char>(name.front())))
        name = "M_" + name;

    const std::string base = name;
    unsigned int suffix = 1;
    while (usedNames.count(name) != 0)
        name = base + "_" + std::to_string(suffix++);
    usedNames.insert(name);
    return name;
}

int ConvertMesh(const std::string& inputPath, const std::string& outputPath)
{
    MeshFile mesh;
    std::string error;
    if (!ReadMeshFile(inputPath, mesh, error))
    {
        std::cerr << "Failed to read mesh file: " << error << "\n";
        return 1;
    }
    if (!ValidateMesh(mesh, error))
    {
        std::cerr << "Invalid mesh: " << error << "\n";
        return 1;
    }

    std::set<std::string> usedNames{"World"};
    const std::string volumeName =
        SanitizeName(std::filesystem::path(inputPath).stem().string(), usedNames);
    const std::string shapeName = volumeName + "_Shape";

    MeshFile::Vertex extent = {0.0, 0.0, 0.0};
    for (const MeshFile::Vertex& vertex : mesh.vertices)
    {
        for (unsigned int axis = 0; axis < 3; ++axis)
            extent[axis] = std::max(extent[axis], std::fabs(vertex[axis] * UnitScale));
    }
    const double worldHalfLength = *std::max_element(extent.begin(), extent.end()) + 1.0;
    const double sphereRadius = std::hypot(extent[0], extent[1], extent[2]) + 1.0;
    if (!std::isfinite(worldHalfLength) || !std::isfinite(sphereRadius))
    {
        std::cerr << "Mesh coordinates are too large to represent in Geomega units\n";
        return 1;
    }

    std::ofstream out(outputPath);
    if (!out)
    {
        std::cerr << "Failed to open output file: " << outputPath << "\n";
        return 1;
    }

    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "// Auto-generated from " << inputPath << " by mesh_to_geomega\n";
    out << "// Mesh coordinates converted from millimeters to centimeters (scale 0.1).\n";
    out << "Include $(MEGALIB)/resource/examples/geomega/materials/Materials.geo\n\n";
    out << "SurroundingSphere " << sphereRadius << " 0.0 0.0 0.0 "
        << sphereRadius << "\n\n";
    out << "Volume World\n";
    out << "World.Material Vacuum\n";
    out << "World.Shape BRIK " << worldHalfLength << " " << worldHalfLength << " "
        << worldHalfLength << "\n";
    out << "World.Mother 0\n\n";
    out << "Volume " << volumeName << "\n";
    out << volumeName << ".Material Vacuum\n";
    out << volumeName << ".Shape TESSELLATED " << mesh.vertices.size() << " "
        << mesh.triangles.size() << "\n";
    out << shapeName << ".Vertices " << mesh.vertices.size() << "\n";
    for (const MeshFile::Vertex& vertex : mesh.vertices)
    {
        out << shapeName << ".Vertex " << vertex[0] * UnitScale << " "
            << vertex[1] * UnitScale << " " << vertex[2] * UnitScale << "\n";
    }
    out << shapeName << ".Facets " << mesh.triangles.size() << "\n";
    for (const MeshFile::Triangle& triangle : mesh.triangles)
    {
        out << shapeName << ".Facet " << triangle[0] << " " << triangle[1] << " "
            << triangle[2] << "\n";
    }
    out << volumeName << ".Mother World\n";
    out << volumeName << ".Position 0.0 0.0 0.0\n";
    out.close();
    if (!out)
    {
        std::cerr << "Failed while writing Geomega output: " << outputPath << "\n";
        return 1;
    }

    std::cout << "Wrote " << outputPath << " (" << mesh.vertices.size() << " vertices, "
              << mesh.triangles.size() << " facets)\n";
    return 0;
}
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " input.stl|input.obj output.geo.setup\n";
        return 1;
    }

    std::string extension = std::filesystem::path(argv[1]).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (extension != ".stl" && extension != ".obj")
    {
        std::cerr << "Input file must have a .stl or .obj extension\n";
        return 1;
    }

    return ConvertMesh(argv[1], argv[2]);
}
