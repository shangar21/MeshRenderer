cmake_minimum_required(VERSION 3.10)
project(MeshRenderer LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_CUDA_ARCHITECTURES 86)

# Manually set CUDA compute capability (since CMake 3.10 doesn't support CMAKE_CUDA_ARCHITECTURES)
set(CUDA_ARCH_FLAGS "-gencode arch=compute_86,code=sm_86")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${CUDA_ARCH_FLAGS}")


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
		cuda/src/Rasterize.cu
)

add_library(MeshRendererLib ${SOURCES})

set_target_properties(MeshRendererLib PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

# Link libraries for the main library
target_link_libraries(MeshRendererLib ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

# Add the main executable
add_executable(MeshRenderer examples/basicRender.cpp)
target_link_libraries(MeshRenderer MeshRendererLib)

# Ensure the correct include directories are set for all targets
target_include_directories(MeshRendererLib PUBLIC ${CMAKE_SOURCE_DIR}/src)


