#pragma once

#include <cuda_runtime.h>
#include "utils.cuh"

struct camera {
	float3 eye;
	float3 target;
	float3 up;

	float fl;
	int image_width;
	int image_height;
	float aspect_ratio;
};

__device__ void getC2W(const camera& cam, float* c2w);
__device__ void getW2C(const camera& cam, float* w2c);
