cmake_minimum_required(VERSION 3.20)
project(mesh_to_geomega)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(mesh_to_geomega
    src/mesh_to_geomega.cpp
    src/meshfile.cpp
)
target_include_directories(mesh_to_geomega PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
