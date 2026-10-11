# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

Development of a next-generation CAD import/conversion tool for MEGAlib (Dr. Andreas Zoglauer's group; team of 3). The code is C++/CMake and must build on both **Linux** (release/production target) and **Windows** (day-to-day testing) — any build change has to work on both.

- **`cad_converter/`**: the converter itself (STEP → repair → conform → mesh → resolve overlaps → STL/OBJ), built on OpenCASCADE and Manifold. It has its own `CLAUDE.md`; read it before working there. This is the code to push.

The other subdirectories are exploratory prototypes, one per library being evaluated:

- **`OpenCascadeTesting/`** — evaluation of OpenCASCADE (OCCT 8.0.1). Has its own, more detailed `CLAUDE.md`; read it before working there.
  - `starter_project/`: the prototype `cad_converter` grew out of (superseded by it).
  - `PrimitiveSolids/` — hand-made STEP test solids (Fusion 360); `starter_project/data/` holds copies.
  - `OCCT/` — a local, gitignored `vcpkg export` of OCCT (~2 GB). Not used by the committed build; never commit it.

## Dependency management conventions

- Third-party libraries come from **vcpkg in manifest mode** (`vcpkg.json` + `vcpkg-configuration.json` with a pinned baseline next to each `CMakeLists.txt`). Never commit vcpkg install trees, exports, or build directories.
- Builds are driven by **`CMakePresets.json`**; the only machine-specific input is the `VCPKG_ROOT` environment variable. Machine-local overrides go in the gitignored `CMakeUserPresets.json`.
