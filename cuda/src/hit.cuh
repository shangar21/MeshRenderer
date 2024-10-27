#pragma once

struct hit {
	int hit;
	float3 point;
	float3 normal;
	float lambda;
	float3 colour;
};

__device__ hit init_hit();
