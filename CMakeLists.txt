cmake_minimum_required(VERSION 3.10)

# Correct capitalization for OpenCV
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

project(MeshRenderer)

include_directories(${CMAKE_SOURCE_DIR}/submodules/tinyobjloader)

# Add the executable
add_executable(MeshRenderer src/main.cpp src/Mesh.cpp src/Camera.cpp src/Renderer.cpp src/Ray.cpp src/Hit.cpp)

# Add target link for OpenCV libraries
target_link_libraries(MeshRenderer ${OpenCV_LIBS})

# Include TinyObjLoader for mesh loading
target_include_directories(MeshRenderer PRIVATE ${CMAKE_SOURCE_DIR}/submodules/tinyobjloader)

