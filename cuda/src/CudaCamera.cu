#include "CudaCamera.cuh"

CudaCamera *cameraToCudaCamera(const Camera &cam) {
  float4 projMatrix[4];
  float4 cudaW2C[4];

  Eigen::Matrix4f m = cam.getProjMatrix() * cam.getW2C();
  Eigen::Matrix4f w2c = cam.getW2C();

  projMatrix[0] = make_float4(m, 0);
  projMatrix[1] = make_float4(m, 1);
  projMatrix[2] = make_float4(m, 2);
  projMatrix[3] = make_float4(m, 3);

  cudaW2C[0] = make_float4(w2c, 0);
  cudaW2C[1] = make_float4(w2c, 1);
  cudaW2C[2] = make_float4(w2c, 2);
  cudaW2C[3] = make_float4(w2c, 3);

  CudaCamera cudaCam;
  for (int i = 0; i < 4; ++i) {
    cudaCam.projMatrix[i] = projMatrix[i];
    cudaCam.w2c[i] = cudaW2C[i];
  }

  cudaCam.imageHeight = cam.imageHeight;
  cudaCam.imageWidth = cam.imageWidth;
  cudaCam.near = cam.near;
  cudaCam.far = cam.far;
  cudaCam.fovx = cam.fovx;
  cudaCam.fovy = cam.fovy;

  CudaCamera *deviceCam;
  cudaMalloc(&deviceCam, sizeof(CudaCamera));
  cudaMemcpy(deviceCam, &cudaCam, sizeof(CudaCamera), cudaMemcpyHostToDevice);

  return deviceCam;
}

void freeCudaCamera(CudaCamera *&cudaCam) {
  if (cudaCam == nullptr)
    return;
  cudaError_t err = cudaFree(cudaCam);
  cudaCam = nullptr;
}


