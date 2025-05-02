#pragma once

#include "CudaTriangle.cuh"
#include "Camera.h"
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

CudaCamera *cameraToCudaCamera(const Camera& cam);
void freeCudaCamera(CudaCamera* &cudaCam); 

__device__ inline bool triangleInView(CudaTriangle *triangle, CudaCamera *cam) {
  float4 aCam = matVecMult(cam->w2c, make_float4(triangle->a, 1.0f));
  float4 bCam = matVecMult(cam->w2c, make_float4(triangle->b, 1.0f));
  float4 cCam = matVecMult(cam->w2c, make_float4(triangle->c, 1.0f));

  if ((aCam.z < cam->near && bCam.z < cam->near && cCam.z < cam->near) ||
      (aCam.z > cam->far && bCam.z > cam->far && cCam.z > cam->far))
    return false;

  float tanHalfFovx = tan(cam->fovx / 2.0f);
  float tanHalfFovy = tan(cam->fovy / 2.0f);

  if ((aCam.x < -aCam.z * tanHalfFovx && bCam.x < -bCam.z * tanHalfFovx &&
       cCam.x < -bCam.z * tanHalfFovx) ||
      (aCam.x > aCam.z * tanHalfFovx && bCam.x > bCam.z * tanHalfFovx &&
       cCam.x > cCam.z * tanHalfFovx)) {
    return false;
  }

  if ((aCam.y < -aCam.z * tanHalfFovy && bCam.y < -bCam.z * tanHalfFovy &&
       cCam.y < -cCam.z * tanHalfFovy) ||
      (aCam.y > aCam.z * tanHalfFovy && bCam.y > bCam.z * tanHalfFovy &&
       cCam.y > cCam.z * tanHalfFovy)) {
    return false;
  }

  return true;
}
