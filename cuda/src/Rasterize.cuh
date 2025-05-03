#pragma once

#include "CudaCamera.cuh"
#include "CudaTriangle.cuh"
#include <Eigen/Dense>
#include <cmath>
#include <cuda_runtime.h>
#include <device_atomic_functions.h>
#include <math_constants.h>
#include <vector>
#include <iostream>

__global__ void rasterizeTriangleKernel(CudaTriangle *cudaTriangles,
                                        CudaCamera *cam, size_t n, float *R,
                                        float *G, float *B, float *depthBuffer,
                                        int rows, int cols);

void rasterizeTriangles(CudaTriangle *cudaTriangles, CudaCamera *cam, size_t n,
                        Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                        Eigen::MatrixXf &B, Eigen::MatrixXf &depthMap);
