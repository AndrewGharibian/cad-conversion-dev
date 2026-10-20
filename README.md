# Mesh-to-Geomega converter

Convert closed STL and OBJ triangle meshes into MEGAlib / Geomega geometry.
This converter consumes existing meshes; it does not read CAD files or
generate meshes, and has no CAD-library dependencies.

## Project structure

- `src/` — STL/OBJ readers, validation, and Geomega output
- `shapes/` — MEGAlib/Geomega tessellated-shape prototype (not built by the converter)
- `examples/` — Geomega tessellated geometry example
- `scripts/` — example validation utility
- `build/` — local CMake build directory

## Build

```sh
cmake -S . -B build
cmake --build build
```

## Usage

```sh
./build/mesh_to_geomega ./mesh/model.stl ./model.geo.setup
./build/mesh_to_geomega ./mesh/model.obj ./model.geo.setup
```

STL conversion writes the mesh inline in the requested setup file. For OBJ
input, each OBJ `o` object is converted and validated independently. The
converter writes the main setup file and a `model_meshes/` directory beside it.
That directory contains a subdirectory for each object, with its
centimeter-scaled, triangulated OBJ and its own `.geo.setup` file. The main
setup file includes each object setup file. Keep the directory alongside the
main setup file when moving the output.

The converter accepts ASCII or binary STL and OBJ. OBJ polygon faces are
triangulated, and positive and negative face indices are supported. Input
coordinates, indices, degenerate triangles, closed manifold edges, consistent
outward winding, and positive enclosed volume are checked before output is
written. Mesh coordinates are assumed to be millimeters and scaled to Geomega
centimeters (0.1); mesh formats do not reliably encode units.

Output uses a Geomega `TESSELLATED` volume and includes MEGAlib's standard
material definitions for `Vacuum`. The Geomega parser accepts this shape
format, but the local Cosima Geant4 construction currently reports
`TESSELLATED` volumes as unsupported; backend support is required to use these
volumes in simulations.
