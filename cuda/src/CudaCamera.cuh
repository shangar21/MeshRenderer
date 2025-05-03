#pragma once

#include "Camera.h"
#include "CudaTriangle.cuh"
#include "MathHelpers.cuh"
#include <cuda_runtime.h>

struct CudaCamera {
  float4 projMatrix[4];
  float4 w2c[4];
  int imageHeight;
  int imageWidth;
  float near;
  float far;
  float fovx;
  float fovy;
};

CudaCamera *cameraToCudaCamera(const Camera &cam);
void freeCudaCamera(CudaCamera *&cudaCam);
__device__ bool triangleInView(CudaTriangle *triangle, CudaCamera *cam);
