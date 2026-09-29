# cad-conversion
Development of a next generation CAD conversion tool for MEGAlib.

## Project structure

- src/ — OpenCascade converter and mesh utilities
- shapes/ — MEGAlib/Geomega shape prototypes and experimental geometry classes
- examples/ — sample STEP inputs and hardcoded Geomega geometry examples
- scripts/ — validation and utility scripts
- build/ — local CMake build directory

## Build

```powershell
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

## Usage

```powershell
./build/Release/step_to_geomega.exe ./examples/step_files/box.step ./output.geo --tessellate-all
```
