# mesh_to_geomega — Codebase Overview

**Purpose:** Convert closed STL/OBJ triangle meshes to MEGAlib / Geomega
geometry descriptions. The converter consumes existing meshes; it does not
read CAD files or generate meshes.

## Files

| File | Role |
|---|---|
| `src/mesh_to_geomega.cpp` | CLI and inline Geomega geometry writer. |
| `src/meshfile.h` / `src/meshfile.cpp` | ASCII/binary STL and OBJ readers, OBJ polygon triangulation, vertex welding, and closed-solid validation. |
| `shapes/MDShapeTESSELLATED.cxx` | MEGAlib-dependent shape implementation/reference; not part of the converter build. |
| `CMakeLists.txt` | Builds the standalone converter using C++17; no CAD-library dependencies. |

## Build and usage

```sh
cmake -S . -B build
cmake --build build
./build/mesh_to_geomega input.stl output.geo.setup
./build/mesh_to_geomega input.obj output.geo.setup
```

STL inputs may be ASCII or binary. OBJ polygon faces are triangulated, and
positive and negative face indices are supported. Only closed,
consistently outward-oriented meshes with positive volume are accepted.
Coordinates are assumed to be millimeters and scaled by 0.1 to Geomega
centimeters.

## Output and known limitation

For STL inputs, the converter writes inline Geomega `TESSELLATED` vertices
and facets. For OBJ inputs, each `o` object is validated and converted
independently. The converter writes the main setup file plus a sibling
`<output-name>_meshes/` directory containing a subdirectory per object, with
its centimeter-scaled, triangulated OBJ and an included `.geo.setup` file that
references it. The main setup file declares `World` and includes each object
setup. Keep the mesh directory with the main setup file.

The local Geomega parser accepts and validates this shape format, but the
local Cosima Geant4 construction currently reports `TESSELLATED` as an
unsupported volume type. Backend support is required before these geometries
can be used in simulations.
