#pragma once

#include "utils.cuh"
#include "hit.cuh"
#include "ray.cuh"

struct mesh {
	int num_faces;
	int num_vertices;
	int num_normals;

	float3* vertices;
	float3* normals;
	float3* texcoors;

	int3* faces;
	int3* face_normals;
	int3* face_texture;
};

__host__ __device__ hit intersect(mesh m, ray r);
