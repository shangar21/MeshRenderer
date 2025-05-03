#include "Project.cuh"

__global__ void projectTrianglesKernel(CudaTriangle *cudaTriangles,
                                       CudaCamera *cam, size_t n) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (idx >= n)
    return;

  // If the triangle is not in view, the default proj value is infinity, will
  // check for this and skip the triangle during rasterization
  if (!triangleInView(&cudaTriangles[idx], cam))
    return;

  // Projecting to screen space
  float3 a = make_float3(
      matVecMult(cam->projMatrix, make_float4(cudaTriangles[idx].a, 1.0f)));
  float3 b = make_float3(
      matVecMult(cam->projMatrix, make_float4(cudaTriangles[idx].b, 1.0f)));
  float3 c = make_float3(
      matVecMult(cam->projMatrix, make_float4(cudaTriangles[idx].c, 1.0f)));

  // Pixel space with pseudo-depth
  cudaTriangles[idx].projA =
      make_float3(cam->imageWidth * ((a.x + 1.0f) / 2.0f),
                  cam->imageHeight * ((a.y + 1.0f) / 2.0f), a.z);
  cudaTriangles[idx].projB =
      make_float3(cam->imageWidth * ((b.x + 1.0f) / 2.0f),
                  cam->imageHeight * ((b.y + 1.0f) / 2.0f), b.z);
  cudaTriangles[idx].projC =
      make_float3(cam->imageWidth * ((c.x + 1.0f) / 2.0f),
                  cam->imageHeight * ((c.y + 1.0f) / 2.0f), c.z);

  cudaTriangles[idx].isProjected = true;
}

void projectTriangles(CudaTriangle *cudaTriangles, CudaCamera *cudaCam,
                      size_t n) {
  const int threadsPerBlock = 256;
  int blocks = (n + threadsPerBlock - 1) / threadsPerBlock;

  // Assume that the cudaTriangles pointer is a cudaMalloc pointer
  // Assume that cudaCam pointer is a cudaMalloc pointer
  projectTrianglesKernel<<<blocks, threadsPerBlock>>>(cudaTriangles, cudaCam,
                                                      n);

  // Wait for threads to finish before moving on
  cudaDeviceSynchronize();
}
