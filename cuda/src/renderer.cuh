#pragma once

#include "camera.cuh"
#include "hit.cuh"
#include "mesh.cuh"
#include "ray.cuh"

__global__ void render_ray_trace(camera cam, mesh m, float* r, float* g, float* b, float vh, float vw, float3 du, float3 dv);

void launch_ray_trace_kernel(camera cam, mesh m, float* r, float* g, float* b, float vh, float vw, float3 du, float3 dv);
