// Run with ROOT: root -l -b -q 'scripts/validate_geomega_meshes.C("output/cosi_test_optimized_geo")'
// Use the same coordinate-based AddFacet path as local MEGAlib.
#include <TGeoTessellated.h>
#include <TSystem.h>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

void validate_geomega_meshes(const char* directory)
{
    unsigned long long meshes = 0, facets = 0, failures = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() != ".geo") continue;
        std::ifstream input(entry.path());
        std::vector<TGeoTessellated::Vertex_t> vertices;
        std::vector<std::array<int, 3>> triangles;
        std::string line;
        while (std::getline(input, line)) {
            std::istringstream tokens(line);
            std::string key;
            tokens >> key;
            if (key.size() >= 7 && key.substr(key.size()-7) == ".Vertex") {
                double x, y, z;
                tokens >> x >> y >> z;
                vertices.emplace_back(x, y, z);
            } else if (key.size() >= 6 && key.substr(key.size()-6) == ".Facet") {
                std::array<int, 3> face;
                tokens >> face[0] >> face[1] >> face[2];
                triangles.push_back(face);
            }
        }
        TGeoTessellated shape(entry.path().stem().c_str());
        bool valid = !triangles.empty();
        for (const auto& f : triangles) {
            if (!shape.AddFacet(vertices.at(f[0]), vertices.at(f[1]), vertices.at(f[2])))
                valid = false;
        }
        for (int i = 0; i < shape.GetNfacets(); ++i)
            if (!shape.FacetCheck(i)) valid = false;
        shape.CloseShape(true, false, false);
        if (!shape.IsClosedBody() || !(shape.Capacity() > 0.0)) valid = false;
        if (!valid) {
            std::cout << "FAILED: " << entry.path() << std::endl;
            ++failures;
        }
        ++meshes;
        facets += triangles.size();
    }
    std::cout << "ROOT validation: " << meshes << " meshes, " << facets
              << " facets, " << failures << " failures" << std::endl;
    gSystem->Exit(meshes == 0 || failures != 0 ? 1 : 0);
}
