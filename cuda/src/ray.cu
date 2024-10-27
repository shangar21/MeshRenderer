#include "ray.cuh"

__host__ __device__ float3 point_at(ray r, float lambda) {
	return make_float3(
    r.origin.x + lambda * r.direction.x,
    r.origin.y + lambda * r.direction.y,
    r.origin.z + lambda * r.direction.z
	);
}
