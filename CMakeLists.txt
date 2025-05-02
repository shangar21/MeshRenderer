cmake_minimum_required(VERSION 3.10)
project(MeshRenderer LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# Find required packages
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Include OpenCV and TinyObjLoader directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/submodules/tinyobjloader)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/cuda/src)

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
		cuda/src/CudaTriangle.cu
		cuda/src/CudaCamera.cu
		cuda/src/Project.cu
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


