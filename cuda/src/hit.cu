#include "hit.cuh"

__host__ __device__ hit init_hit(){
	return {
		.hit = 0,
		.point = make_float3(0.0f, 0.0f, 0.0f),
		.normal = make_float3(0.0f, 0.0f, 0.0f),
		.lambda = INFINITY,
		.colour = make_float3(0.0f, 0.0f, 0.0f)
	}
}
