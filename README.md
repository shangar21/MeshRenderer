# MeshRenderer

A simple ray tracing (rasterizing in the future) application that loads and renders 3D meshes from .obj files. Can use the headers and other CPP files for your own applications, but to quickly render can build and pass in an obj path and output path to render and save as a png.

## Dependencies
- Eigen
- TinyObjLoader (in submodules)
- OpenCV

## TODO
1. Read Textures
2. Radiance model
3. BVH
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
## Example image

Image below is just normal map with barycentric interpolation

![mesh_render](https://github.com/user-attachments/assets/e2f48e0e-e11f-4848-b90d-f0a35948dec7)

Image below is with texture mapping as well

![mesh_render](https://github.com/user-attachments/assets/f47c26f3-3bdf-4314-b984-7c758cff3ed8)

Image below is first iteration of rasterization (some math for the projections are wrong and need to be fixed)

![image](https://github.com/user-attachments/assets/1e8b4772-86e7-4d26-82ad-721877c25492)
