#include "mesh.cuh"

__device__ hit intersect(mesh m, ray r) {
    hit closest_hit = init_hit();

    for (int i = 0; i < m.num_faces; i++) {
        int3 vector_indices = m.faces[i];
        float3 a = m.vertices[vector_indices.x];
        float3 b = m.vertices[vector_indices.y];
        float3 c = m.vertices[vector_indices.z];

        float3 edge1 = b - a;
        float3 edge2 = c - a;

        float3 h = cross(r.direction, edge2);
        float det = dot(edge1, h);

        if (fabs(det) < FP_TOLERANCE) continue;

        float inv_det = 1.0f / det;
        float3 s = r.origin - a;

        float u = inv_det * dot(s, h);
        if (u < 0.0f || u > 1.0f) continue;

        float3 q = cross(s, edge1);
        float v = inv_det * dot(r.direction, q);
        if (v < 0.0f || u + v > 1.0f) continue;

        float lambda = inv_det * dot(edge2, q);
        if (lambda < FP_TOLERANCE) continue;

        if (lambda < closest_hit.lambda) {
            closest_hit.hit = 1;
            closest_hit.lambda = lambda;
            closest_hit.point = point_at(r, lambda);
            closest_hit.normal = normalize(cross(edge1, edge2));
            closest_hit.colour = make_float3(
                (closest_hit.normal.x + 1) / 2,
                (closest_hit.normal.y + 1) / 2,
                (closest_hit.normal.z + 1) / 2
            );
        }
    }

    return closest_hit;
}

