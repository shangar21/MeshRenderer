#include "renderer.cuh"
#include <stdio.h>
#include <cstdio>
#include <cmath>

__global__ void render_ray_trace(camera cam, mesh m, float *r, float *g,
                                 float *b, float vh, float vw, float3 du,
                                 float3 dv) {
	int pixel_index = blockIdx.x * blockDim.x + threadIdx.x;
	
	int x = pixel_index % cam.image_width;
	int y = pixel_index / cam.image_width;

	//printf("x y: %d %d\n", x, y);

  if (x >= cam.image_width || y >= cam.image_height)
    return;

  // Calculate normalized camera-space ray direction
  float u_offset = (x - cam.image_width / 2.0f) * du.x;
  float v_offset = (y - cam.image_height / 2.0f) * dv.y;

  float3 ray_dir_cam = normalize(make_float3(u_offset, v_offset, -cam.fl));
  float4 ray_dir_cam_homo =
      make_float4(ray_dir_cam.x, ray_dir_cam.y, ray_dir_cam.z, 0.0f);

  // Create camera-to-world (C2W) matrix
  float3 w_c = normalize(cam.target - cam.eye);
  float3 u_c = normalize(cross(cam.up, w_c));
  float3 v_c = cross(w_c, u_c);

	float c2w[16] = {
			u_c.x, v_c.x, w_c.x, cam.eye.x,
			u_c.y, v_c.y, w_c.y, cam.eye.y,
			u_c.z, v_c.z, w_c.z, cam.eye.z,
			0.0f,  0.0f,  0.0f,  1.0f
	};

  float4 ray_dir_world_homo = normalize(matMul4x4(c2w, ray_dir_cam_homo));
  float3 ray_dir_world = make_float3(ray_dir_world_homo.x, ray_dir_world_homo.y,
                                     ray_dir_world_homo.z);

  ray ray_obj = {.origin = cam.eye, .direction = ray_dir_world};

  // Initialize closest hit data
  hit closest_hit = {.hit = 0,
                     .point = make_float3(0.0f, 0.0f, 0.0f),
                     .normal = make_float3(0.0f, 0.0f, 0.0f),
                     .lambda = INFINITY,
                     .colour = make_float3(0.0f, 0.0f, 0.0f)};

  for (int i = 0; i < m.num_faces; i++) {
    int3 vector_indices = m.faces[i];
    float3 a = m.vertices[vector_indices.x];
    float3 b = m.vertices[vector_indices.y];
    float3 c = m.vertices[vector_indices.z];

    float3 edge1 = b - a;
    float3 edge2 = c - a;

    float3 h = cross(ray_obj.direction, edge2);
    float det = dot(edge1, h);

    if (fabs(det) < FP_TOLERANCE) continue;

    float inv_det = 1.0f / det;
    float3 s = ray_obj.origin - a;

    float u_bary = inv_det * dot(s, h);
    if (u_bary < 0.0f || u_bary > 1.0f) continue;

    float3 q = cross(s, edge1);
    float v_bary = inv_det * dot(ray_obj.direction, q);
    if (v_bary < 0.0f || u_bary + v_bary > 1.0f) continue;

    float lambda = inv_det * dot(edge2, q);
    if (lambda < FP_TOLERANCE) continue;

    if (lambda < closest_hit.lambda) {
      closest_hit.hit = 1;
      closest_hit.lambda = lambda;
      closest_hit.point = ray_obj.origin + lambda * ray_obj.direction;
      closest_hit.normal = normalize(cross(edge1, edge2));
      closest_hit.colour = make_float3((closest_hit.normal.x + 1) / 2,
                                       (closest_hit.normal.y + 1) / 2,
                                       (closest_hit.normal.z + 1) / 2);
    }
  }

  if (closest_hit.hit) {
    printf("Hit detected at pixel (%d, %d)\n", x, y);
    r[pixel_index] = closest_hit.colour.x;
    g[pixel_index] = closest_hit.colour.y;
    b[pixel_index] = closest_hit.colour.z;
  } else {
    r[pixel_index] = 0.0f;
    g[pixel_index] = 0.0f;
    b[pixel_index] = 0.0f;
  }
}

void launch_ray_trace_kernel(camera cam, mesh m, float *r, float *g, float *b,
                             float vh, float vw, float3 du, float3 dv) {
	int total_pixels = cam.image_width * cam.image_height;
	int threadsPerBlock = 256;
	int blocksPerGrid = (total_pixels + threadsPerBlock - 1) / threadsPerBlock;
	render_ray_trace<<<blocksPerGrid, threadsPerBlock>>>(cam, m, r, g, b, vh, vw, du, dv);
  cudaDeviceSynchronize();
}
