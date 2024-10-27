#include "renderer.cuh"
#include <stdio.h>

__global__ void render_ray_trace(camera cam, mesh m, float *r, float *g,
                                 float *b, float vh, float vw, float3 du,
                                 float3 dv) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.x * blockDim.y + threadIdx.y;

  if (x >= cam.image_width || y >= cam.image_height)
    return;

  float u = (x - cam.image_width / 2.0f) * du.x;
  float v = (y - cam.image_height / 2.0f) * dv.y;

  float3 ray_dir_cam = normalize(make_float3(u, v, -cam.fl));
  float4 ray_dir_cam_homo =
      make_float4(ray_dir_cam.x, ray_dir_cam.y, ray_dir_cam.z, 0.0f);
  float c2w[16];

  float3 w_c = normalize(cam.target - cam.eye);
  float3 u_c = normalize(cross(cam.up, w_c));
  float3 v_c = cross(w_c, u_c);

  c2w[0] = u_c.x;
  c2w[1] = u_c.y;
  c2w[2] = u_c.z;
  c2w[3] = 0.0f;
  c2w[4] = v_c.x;
  c2w[5] = v_c.y;
  c2w[6] = v_c.z;
  c2w[7] = 0.0f;
  c2w[8] = w_c.x;
  c2w[9] = w_c.y;
  c2w[10] = w_c.z;
  c2w[11] = 0.0f;
  c2w[12] = cam.eye.x;
  c2w[13] = cam.eye.y;
  c2w[14] = cam.eye.z;
  c2w[15] = 1.0f;

  float4 ray_dir_world_homo = normalize(matMul4x4(c2w, ray_dir_cam_homo));
  float3 ray_dir_world = make_float3(ray_dir_world_homo.x, ray_dir_world_homo.y,
                                     ray_dir_world_homo.z);

  ray ray_obj = {.origin = cam.eye, .direction = ray_dir_world};

  //  hit hit_data = intersect(m, ray_obj);

  hit closest_hit = {.hit = 0,
                     .point = make_float3(0.0f, 0.0f, 0.0f),
                     .normal = make_float3(0.0f, 0.0f, 0.0f),
                     .lambda = INFINITY,
                     .colour = make_float3(0.0f, 0.0f, 0.0f)};

  for (int i = 0; i < m.num_faces; i++) {
		printf("Checking face %d\n", i);
    int3 vector_indices = m.faces[i];
    float3 a = m.vertices[vector_indices.x];
    float3 b = m.vertices[vector_indices.y];
    float3 c = m.vertices[vector_indices.z];

    float3 edge1 = b - a;
    float3 edge2 = c - a;

    float3 h = cross(ray_obj.direction, edge2);
    float det = dot(edge1, h);

    if (fabs(det) < FP_TOLERANCE)
      continue;

    float inv_det = 1.0f / det;
    float3 s = ray_obj.origin - a;

    float u = inv_det * dot(s, h);
    if (u < 0.0f || u > 1.0f)
      continue;

    float3 q = cross(s, edge1);
    float v = inv_det * dot(ray_obj.direction, q);
    if (v < 0.0f || u + v > 1.0f)
      continue;

    float lambda = inv_det * dot(edge2, q);
    if (lambda < FP_TOLERANCE)
      continue;

    if (lambda < closest_hit.lambda) {
      closest_hit.hit = 1;
      closest_hit.lambda = lambda;
      closest_hit.point =
          make_float3(ray_obj.origin.x + lambda * ray_obj.direction.x,
                      ray_obj.origin.y + lambda * ray_obj.direction.y,
                      ray_obj.origin.z + lambda * ray_obj.direction.z);
      closest_hit.normal = normalize(cross(edge1, edge2));
      closest_hit.colour = make_float3((closest_hit.normal.x + 1) / 2,
                                       (closest_hit.normal.y + 1) / 2,
                                       (closest_hit.normal.z + 1) / 2);
    }
  }

  int pixel_index = y * cam.image_width + x;

  if (closest_hit.hit) {
		printf("Hit!\n");
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
  dim3 threadsPerBlock(16, 16);
  dim3 blocksPerGrid(
      (cam.image_width + threadsPerBlock.x - 1) / threadsPerBlock.x,
      (cam.image_height + threadsPerBlock.y - 1) / threadsPerBlock.y);
  render_ray_trace<<<blocksPerGrid, threadsPerBlock>>>(cam, m, r, g, b, vh, vw,
                                                       du, dv);
  cudaDeviceSynchronize();
}
