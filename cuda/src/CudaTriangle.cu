#include <CudaTriangle.cuh>

CudaTriangle *triangleToCudaTriangle(const std::vector<Triangle> &triangles) {
  struct CudaTriangle *cudaTriangles = new CudaTriangle[triangles.size()];
  for (int i = 0; i < triangles.size(); i++) {
    cudaTriangles[i] = {
        .a = make_float3(triangles[i].a),
        .b = make_float3(triangles[i].b),
        .c = make_float3(triangles[i].c),

        .nA = make_float3(triangles[i].nA),
        .nB = make_float3(triangles[i].nB),
        .nC = make_float3(triangles[i].nC),

        .tA = make_float2(triangles[i].tA),
        .tB = make_float2(triangles[i].tB),
        .tC = make_float2(triangles[i].tC),

        .colA = make_float3(triangles[i].colA),
        .colB = make_float3(triangles[i].colB),
        .colC = make_float3(triangles[i].colC),

        .projA = make_float3(INFINITY),
        .projB = make_float3(INFINITY),
        .projC = make_float3(INFINITY),
    };
  }

  CudaTriangle *deviceTriangles;
  cudaMalloc(&deviceTriangles, triangles.size() * sizeof(CudaTriangle));
  cudaMemcpy(deviceTriangles, cudaTriangles,
             triangles.size() * sizeof(CudaTriangle), cudaMemcpyHostToDevice);

  delete[] cudaTriangles;
  return deviceTriangles;
}

void freeCudaTriangles(CudaTriangle *&deviceTriangles) {
  if (deviceTriangles == nullptr)
    return;
  cudaError_t err = cudaFree(deviceTriangles);
  deviceTriangles = nullptr;
}

__device__ float3 getBarycentric(CudaTriangle *triangle, float3 p) {
  float3 v0v1 = triangle->b - triangle->a;
  float3 v0v2 = triangle->c - triangle->a;
  float3 v0p = p - triangle->a;

  float d00 = dot(v0v1, v0v1);
  float d01 = dot(v0v1, v0v2);
  float d11 = dot(v0v2, v0v2);
  float d20 = dot(v0p, v0v1);
  float d21 = dot(v0p, v0v2);

  float denom = d00 * d11 - d01 * d01;
  float v = (d11 * d20 - d01 * d21) / denom;
  float w = (d00 * d21 - d01 * d20) / denom;
  float u = 1.0f - v - w;

  return make_float3(u, v, w);
}

__device__ bool isPointInside(CudaTriangle *triangle, float3 p) {
  float3 bary = getBarycentric(triangle, p);
  return bary.x >= 0 && bary.y >= 0 && bary.z >= 0;
}
