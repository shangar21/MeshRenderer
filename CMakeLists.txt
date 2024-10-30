#cmake_minimum_required(VERSION 3.10)
#
#project(MeshRenderer LANGUAGES CXX)
#
#find_package(OpenMP REQUIRED)
#
## Correct capitalization for OpenCV
#find_package(OpenCV REQUIRED)
#
## Include OpenCV directories
#include_directories(${OpenCV_INCLUDE_DIRS})
#
#
#include_directories(${CMAKE_SOURCE_DIR}/submodules/tinyobjloader)
#
## Add the executable
#add_executable(MeshRenderer src/main.cpp src/Mesh.cpp src/Camera.cpp src/Renderer.cpp src/Ray.cpp src/Hit.cpp src/BVH.cpp src/BVHNode.cpp src/Triangle.cpp)
#
## Add target link for OpenCV libraries
#target_link_libraries(MeshRenderer ${OpenCV_LIBS})
#
#target_link_libraries(MeshRenderer OpenMP::OpenMP_CXX)
#
## Include TinyObjLoader for mesh loading
#target_include_directories(MeshRenderer PRIVATE ${CMAKE_SOURCE_DIR}/submodules/tinyobjloader)

cmake_minimum_required(VERSION 3.10)
project(MeshRenderer LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

# Include OpenCV and TinyObjLoader directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/submodules/tinyobjloader)

# Add the src library
set(SOURCES
    src/Mesh.cpp
    src/Camera.cpp
    src/Renderer.cpp
    src/Ray.cpp
    src/Hit.cpp
    src/BVH.cpp
    src/BVHNode.cpp
    src/Triangle.cpp
)

add_library(MeshRendererLib ${SOURCES})

# Link libraries for the main library
target_link_libraries(MeshRendererLib ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

# Add the main executable
add_executable(MeshRenderer src/main.cpp)
target_link_libraries(MeshRenderer MeshRendererLib)

# Add test executables and link with the MeshRendererLib library
add_executable(test_triangle test/test_triangle.cpp)
target_link_libraries(test_triangle MeshRendererLib)

add_executable(test_bvhnode test/test_bvhnode.cpp)
target_link_libraries(test_bvhnode MeshRendererLib)

add_executable(test_bvh test/test_bvh.cpp)
target_link_libraries(test_bvh MeshRendererLib)

# Ensure the correct include directories are set for all targets
target_include_directories(MeshRendererLib PUBLIC ${CMAKE_SOURCE_DIR}/src)


