#include "renderer.cuh"

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
  getC2W(cam, c2w);
  float4 ray_dir_world_homo = normalize(matMul4x4(c2w, ray_dir_cam_homo));

  ray r = {.origin = cam.eye, .direction = ray_dir_world_homo};

  hit hit_data = intersect(m, r);

  int pixel_index = y * cam.image_width + x;

  if (hit_data.hit) {
    r[pixel_index] = hit_data.colour.x;
    g[pixel_index] = hit_data.colour.y;
    b[pixel_index] = hit_data.colour.z;
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
	render_ray_trace<<<blocksPerGrid, threadsPerBlock>>>(cam, m, r, g, b, vh, vw, du, dv);
	cudaDeviceSynchronize();
}
