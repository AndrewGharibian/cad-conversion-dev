# =============================================================================
# cad_converter — STEP to simulation-ready meshes for MEGAlib/Geant4.
#
# Dependencies come from vcpkg in manifest mode (vcpkg.json, pinned by
# "builtin-baseline"); configuration is driven by CMakePresets.json:
#   cmake --preset windows        |  cmake --preset linux-release
#   cmake --build --preset windows-release  |  cmake --build --preset linux-release
#   ctest --preset windows-release          |  ctest --preset linux-release
# =============================================================================

# 3.21 is the minimum for $<TARGET_RUNTIME_DLLS> and install(RUNTIME_DEPENDENCY_SET).
cmake_minimum_required(VERSION 3.21)
project(cad_converter VERSION 0.1.0 LANGUAGES CXX)

# Normalize Windows path separators before matching the install() exclude
# regexes below (CMake 4.x warns otherwise).
if(POLICY CMP0207)
    cmake_policy(SET CMP0207 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)

# Installed executables on Linux find their bundled .so files in ../lib
# relative to themselves, so the install tree can be copied anywhere.
if(NOT APPLE)
    set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
endif()

# OpenCASCADE: CAD geometry (STEP import, repair, the general fuse, meshing).
# Its CMake package exports one imported target per toolkit (TKernel, ...).
find_package(OpenCASCADE CONFIG REQUIRED)
# Manifold: robust mesh booleans (cutting overlaps between meshes).
find_package(manifold CONFIG REQUIRED)

add_executable(cad_converter
    src/main.cpp
    src/step_import.cpp
    src/repair.cpp
    src/folded_faces.cpp
    src/thread_pairs.cpp
    src/conform.cpp
    src/meshing.cpp
    src/boundary_mesh.cpp
    src/revolved_mesh.cpp
    src/weld.cpp
    src/mesh_cut.cpp
    src/export.cpp
    src/geomega_export.cpp
)

# Only the OCCT toolkits used are linked; their dependencies come in
# transitively. (All of ${OpenCASCADE_LIBRARIES} would also pull in the
# visualization/OpenGL libraries, which this tool never loads.)
target_link_libraries(cad_converter PRIVATE
    TKernel       # Standard_*, NCollection, OSD_Parallel
    TKMath        # gp_*, Bnd_Box
    TKG2d
    TKG3d
    TKGeomBase
    TKBRep        # TopoDS_*, BRep_Tool, BinTools
    TKGeomAlgo
    TKTopAlgo     # BRepCheck, BRepLib, BRepBndLib, BRepExtrema, BRepClass3d
    TKShHealing   # ShapeFix_Shape, ShapeUpgrade, ShapeAnalysis
    TKMesh        # BRepMesh_IncrementalMesh
    TKBO          # BRepAlgoAPI_BuilderAlgo (the general fuse)
    TKXSBase
    TKDESTEP      # STEPCAFControl_Reader
    TKXCAF        # XCAFDoc_ShapeTool: the assembly tree and part names
    TKLCAF        # TDocStd_Document, TDataStd_Name
    manifold::manifold
)

if(MSVC)
    target_compile_options(cad_converter PRIVATE /W3 /permissive- /utf-8)
else()
    target_compile_options(cad_converter PRIVATE -Wall -Wextra)
endif()

# -----------------------------------------------------------------------------
# Runtime libraries next to the executable (Windows has no RPATH: a DLL is
# only found next to the .exe or on PATH; a missing one makes the exe exit
# silently). Linux build trees use the RPATH the linker embeds.
# -----------------------------------------------------------------------------
if(WIN32)
    set(_runtime_dlls "$<TARGET_RUNTIME_DLLS:cad_converter>")
    add_custom_command(TARGET cad_converter POST_BUILD
        COMMAND "$<$<BOOL:${_runtime_dlls}>:${CMAKE_COMMAND};-E;copy_if_different;${_runtime_dlls};$<TARGET_FILE_DIR:cad_converter>>"
        COMMAND_EXPAND_LISTS
        VERBATIM
    )
endif()

# -----------------------------------------------------------------------------
# Install: a self-contained, relocatable bundle
#   cmake --install build/<preset> [--config Release]
# gives install/<preset>/bin/cad_converter[.exe] plus its libraries (next to
# the .exe on Windows, in ../lib with RPATH $ORIGIN/../lib on Linux).
# -----------------------------------------------------------------------------
install(TARGETS cad_converter
    RUNTIME_DEPENDENCY_SET cad_converter_runtime_deps
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(RUNTIME_DEPENDENCY_SET cad_converter_runtime_deps
    # Only bundle third-party libraries, never OS / compiler runtimes.
    PRE_EXCLUDE_REGEXES
        "^api-ms-" "^ext-ms-"
    POST_EXCLUDE_REGEXES
        "[Ss]ystem32/" "[Ss]ys[Ww][Oo][Ww]64/"
        "^/lib/" "^/lib64/" "^/usr/lib/" "^/usr/lib64/"
    DIRECTORIES $<TARGET_FILE_DIR:cad_converter>
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# -----------------------------------------------------------------------------
# Tests (tests/CMakeLists.txt)
# -----------------------------------------------------------------------------
enable_testing()
add_subdirectory(tests)
