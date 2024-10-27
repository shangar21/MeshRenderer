#pragma once

#include "camera.cuh"
#include "hit.h"
#include "mesh.cuh"
#include "ray.cuh"

__global__ void render_ray_trace(camera cam, mesh m, float* r, float* g, float* b);

void launch_ray_trace_kernel(camera cam, mesh m, float* r, float* g, float* b)
