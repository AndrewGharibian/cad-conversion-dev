cmake_minimum_required(VERSION 3.20)
project(step_to_geomega)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCASCADE CONFIG REQUIRED)

add_executable(step_to_geomega
    src/step_to_geomega.cpp
    src/tessellatedsolid.cpp
)
target_link_libraries(step_to_geomega PRIVATE ${OpenCASCADE_LIBRARIES})
target_include_directories(step_to_geomega PRIVATE
    ${OpenCASCADE_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
