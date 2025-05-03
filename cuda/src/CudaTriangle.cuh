#pragma once

#include "MathHelpers.cuh"
#include "Triangle.h"
#include <cuda_runtime.h>
#include <vector>

struct CudaTriangle {
  float3 a;
  float3 b;
  float3 c;

  float3 nA;
  float3 nB;
  float3 nC;

  float2 tA;
  float2 tB;
  float2 tC;

  float3 colA;
  float3 colB;
  float3 colC;

  float3 projA;
  float3 projB;
  float3 projC;

	bool isProjected;
};

CudaTriangle *triangleToCudaTriangle(const std::vector<Triangle> &triangles);
void freeCudaTriangles(CudaTriangle *&deviceTriangles);
__device__ bool isPointInside(CudaTriangle *triangle, float3 p);
__device__ float3 getBarycentric(CudaTriangle *triangle, float3 p);
__device__ float3 getBarycentricNormal(CudaTriangle *triangle, float3 b);
__device__ float3 getBarycentricColour(CudaTriangle *triangle, float3 b);
