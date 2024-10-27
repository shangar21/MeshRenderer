# pragma once

#include <cuda_runtime.h>
#include "utils.cuh"

struct ray{
	float3 origin;
	float3 direction;
};

__host__ __device__ float3 point_at(ray r, float lambda);


