cmake_minimum_required(VERSION 3.10)
project(CudaMeshRenderer LANGUAGES CXX CUDA)

# Set C++ and CUDA standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 11)

# Find CUDA
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Add CUDA include directories explicitly
include_directories(${CUDA_INCLUDE_DIRS})

# Add the ../submodules directory to the include path for tiny_obj_loader
include_directories(${CMAKE_SOURCE_DIR}/../submodules/tinyobjloader)

# Source files
set(SRC_FILES
    src/main.cpp
    src/camera.cu
    src/hit.cu
    src/mesh.cu
    src/ray.cu
    src/renderer.cu
)

# Create executable
add_executable(CudaMeshRenderer ${SRC_FILES})

# Link CUDA libraries
target_link_libraries(CudaMeshRenderer ${CUDA_LIBRARIES} ${OpenCV_LIBS})

