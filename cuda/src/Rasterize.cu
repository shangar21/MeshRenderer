#include "Rasterize.cuh"

__device__ inline float3 barycentric2D(float2 *p, float3 *a, float3 *b,
                                       float3 *c) {
  float det = (b->y - c->y) * (a->x - c->x) + (c->x - b->x) * (a->y - c->y);

  float lambda0 =
      ((b->y - c->y) * (p->x - c->x) + (c->x - b->x) * (p->y - c->y)) / det;
  float lambda1 =
      ((c->y - a->y) * (p->x - c->x) + (a->x - c->x) * (p->y - c->y)) / det;
  float lambda2 = 1.0f - lambda0 - lambda1;

  return make_float3(lambda0, lambda1, lambda2);
}

// Stolen from
// https://stackoverflow.com/questions/17399119/how-do-i-use-atomicmax-on-floating-point-values-in-cuda
__device__ __forceinline__ float fatomicMin(float *addr, float value) {
  float old;
  old = (value >= 0)
            ? __int_as_float(atomicMin((int *)addr, __float_as_int(value)))
            : __uint_as_float(
                  atomicMax((unsigned int *)addr, __float_as_uint(value)));

  return old;
}

__global__ void rasterizeTriangleKernel(CudaTriangle *cudaTriangles,
                                        CudaCamera *cam, size_t n, float *R,
                                        float *G, float *B, float *depthBuffer,
                                        int rows, int cols) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;

  if (idx >= n)
    return;

  if (!cudaTriangles[idx].isProjected)
    return;

  float3 v0 = cudaTriangles[idx].projA;
  float3 v1 = cudaTriangles[idx].projB;
  float3 v2 = cudaTriangles[idx].projC;

  // BBox (in pixel space)
  float minX = fminf(v0.x, fminf(v1.x, v2.x));
  float maxX = fmaxf(v0.x, fmaxf(v1.x, v2.x));
  float minY = fminf(v0.y, fminf(v1.y, v2.y));
  float maxY = fmaxf(v0.y, fmaxf(v1.y, v2.y));

  int xmin = max(0, (int)floorf(minX));
  int xmax = min(cols - 1, (int)ceilf(maxX));
  int ymin = max(0, (int)floorf(minY));
  int ymax = min(rows - 1, (int)ceilf(maxY));

  for (int y = ymin; y <= ymax; y++) {
    for (int x = xmin; x <= xmax; x++) {
      // center of pixel
      float2 p = make_float2(x + 0.5, y + 0.5);
      // Barycentric location of pixel relative to triangle
      float3 bary = barycentric2D(&p, &v0, &v1, &v2);
      // if not inside tri, we can skip
      if (bary.x < 0 || bary.y < 0 || bary.z < 0)
        continue;

      // depth estimate with bary-interpolation
      int bufIdx = y * cols + x;
      float depth = dot(bary, make_float3(v0.z, v1.z, v2.z));

      float oldDepth = fatomicMin(&depthBuffer[bufIdx], depth);

      // if behind other triangles, do not bother ggz
      if (depth > oldDepth)
        continue;

      R[bufIdx] = dot(bary, make_float3(cudaTriangles[idx].colA.x,
                                        cudaTriangles[idx].colB.x,
                                        cudaTriangles[idx].colC.x));
      G[bufIdx] = dot(bary, make_float3(cudaTriangles[idx].colA.y,
                                        cudaTriangles[idx].colB.y,
                                        cudaTriangles[idx].colC.y));
      B[bufIdx] = dot(bary, make_float3(cudaTriangles[idx].colA.z,
                                        cudaTriangles[idx].colB.z,
                                        cudaTriangles[idx].colC.z));
    }
  }
}

void rasterizeTriangles(CudaTriangle *cudaTriangles, CudaCamera *cam, size_t n,
                        Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                        Eigen::MatrixXf &B, Eigen::MatrixXf &depthMap) {
  const int threadsPerBlock = 256;
  int blocks = (n + threadsPerBlock - 1) / threadsPerBlock;

  float *r, *g, *b, *d;
  int totalPixels = R.size();
  int rows = R.rows();
  int cols = R.cols();

  cudaMalloc(&r, sizeof(float) * totalPixels);
  cudaMalloc(&g, sizeof(float) * totalPixels);
  cudaMalloc(&b, sizeof(float) * totalPixels);
  cudaMalloc(&d, sizeof(float) * totalPixels);

  cudaMemset(r, 0, sizeof(float) * totalPixels);
  cudaMemset(g, 0, sizeof(float) * totalPixels);
  cudaMemset(b, 0, sizeof(float) * totalPixels);

  // Depth needs to be initialzied at infinity
  std::vector<float> initDepth(totalPixels,
                               std::numeric_limits<float>::infinity());
  cudaMemcpy(d, initDepth.data(), sizeof(float) * totalPixels,
             cudaMemcpyHostToDevice);

  rasterizeTriangleKernel<<<blocks, threadsPerBlock>>>(cudaTriangles, cam, n, r,
                                                       g, b, d, rows, cols);

  cudaDeviceSynchronize();

  std::vector<float> hr(totalPixels);
  std::vector<float> hg(totalPixels);
  std::vector<float> hb(totalPixels);
  std::vector<float> hd(totalPixels);

  cudaMemcpy(hr.data(), r, totalPixels * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(hg.data(), g, totalPixels * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(hb.data(), b, totalPixels * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(hd.data(), d, totalPixels * sizeof(float), cudaMemcpyDeviceToHost);

  cudaFree(r);
  cudaFree(g);
  cudaFree(b);
  cudaFree(d);

  Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      mappedR(hr.data(), rows, cols);

  Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      mappedG(hg.data(), rows, cols);

  Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      mappedB(hb.data(), rows, cols);

  Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      mappedD(hd.data(), rows, cols);

  R = mappedR.colwise().reverse().eval();
  G = mappedG.colwise().reverse().eval();
  B = mappedB.colwise().reverse().eval();
  depthMap = mappedD.colwise().reverse().eval();
}
