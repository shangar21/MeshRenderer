# MeshRenderer

A simple ray tracing (rasterizing in the future) application that loads and renders 3D meshes from .obj files. Can use the headers and other CPP files for your own applications, but to quickly render can build and pass in an obj path and output path to render and save as a png.

## Dependencies
- Eigen
- TinyObjLoader (in submodules)
- OpenCV

## TODO
1. Read Textures
2. Radiance model
3. Bump Mapping
4. Animation
5. Rasterization
6. CUDA

## How to Build

First clone repo, then:

```
git submodule update --init --recursive
```
Once submodules have been added and dependencies installed

```
mkdir build/ && cd build/
cmake ..
make
```
