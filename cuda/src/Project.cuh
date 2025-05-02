#pragma once

#include "CudaCamera.cuh"
#include "CudaTriangle.cuh"
#include <Eigen/Dense>
#include <cmath>
#include <cuda_runtime.h>

__global__ void projectTrianglesKernel(CudaTriangle *cudaTriangles,
                                       CudaCamera *cam, size_t n);

void projectTriangles(CudaTriangle *cudaTriangles, CudaCamera *cudaCam,
                       size_t n);
