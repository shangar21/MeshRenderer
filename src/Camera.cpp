#include "Camera.h"
#include <iostream>

Camera::Camera(const Eigen::Vector3f &p, const Eigen::Vector3f &g,
               const Eigen::Vector3f &t, float fl, int imageWidth,
               int imageHeight, float near, float far)
    : eye(p), target(g), up(t), fl(fl), imageWidth(imageWidth),
      imageHeight(imageHeight) {
  aspectRatio = (float)imageWidth / (float)imageHeight;
  fovx = 2.0f * std::atan((float)imageWidth / (2 * fl));
  fovy = 2.0f * std::atan((float)imageHeight / (2 * fl));
}

Eigen::Matrix4f Camera::getC2W() const {
  // normalized gaze
  Eigen::Vector3f w = (target - eye).normalized();
  Eigen::Vector3f u = w.cross(up).normalized();
  Eigen::Vector3f v = w.cross(u).normalized();

  Eigen::Matrix4f c2w = Eigen::Matrix4f::Identity();

  c2w.block<3, 1>(0, 0) = u;
  c2w.block<3, 1>(0, 1) = v;
  c2w.block<3, 1>(0, 2) = -w;
  c2w.block<3, 1>(0, 3) = -eye;

  return c2w;
}

Eigen::Matrix4f Camera::getW2C() const { return getC2W().inverse(); }

// From Real-Time Rendering (Thomas Moller)
Eigen::Matrix4f Camera::getProjMatrix() const {
  float tanFovX = std::tan(fovx / 2.0f);
  float tanFovY = std::tan(fovy / 2.0f);

  Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();

  proj(0, 0) = 1.0f / tanFovX;
  proj(1, 1) = 1.0f / tanFovY;
  proj(2, 2) = -(far + near) / (far - near);
  proj(2, 3) = -(2.0f * far * near) / (far - near);
  proj(3, 2) = -1.0f;

  return proj;
}

std::vector<Eigen::Vector3f>
Camera::projectTriangle(const Triangle triangle) const {
  std::vector<Eigen::Vector3f> projectedVertices;

  std::vector<Eigen::Vector3f> triangleVertices = {triangle.a, triangle.b,
                                                   triangle.c};

  for (const Eigen::Vector3f vertex : triangleVertices) {
    Eigen::Vector4f vertexCamHomo =
        getProjMatrix() * getW2C() * vertex.homogeneous();
    Eigen::Vector3f vertexNDC = vertexCamHomo.hnormalized();
    float pixelX = ((vertexNDC.x() + 1.0f) / 2.0f) * imageWidth;
    float pixelY = (1.0f - ((vertexNDC.y() + 1.0f) / 2.0f)) * imageHeight;
    float pixelZ = vertexNDC.z();
    projectedVertices.emplace_back(pixelX, pixelY, pixelZ);
  }

  return projectedVertices;
}

bool Camera::triangleInView(const Triangle triangle) const {
  Eigen::Vector4f aCam = getW2C() * triangle.a.homogeneous();
  Eigen::Vector4f bCam = getW2C() * triangle.b.homogeneous();
  Eigen::Vector4f cCam = getW2C() * triangle.c.homogeneous();

  if ((aCam.z() < near && bCam.z() < near && cCam.z() < near) ||
      (aCam.z() > far && bCam.z() > far && cCam.z() > far))
    return false;

  float tanHalfFovx = std::tan(fovx / 2.0f);
  float tanHalfFovy = std::tan(fovy / 2.0f);

  if ((aCam.x() < -aCam.z() * tanHalfFovx &&
       bCam.x() < -bCam.z() * tanHalfFovx &&
       cCam.x() < -bCam.z() * tanHalfFovx) ||
      (aCam.x() > aCam.z() * tanHalfFovx && bCam.x() > bCam.z() * tanHalfFovx &&
       cCam.x() > cCam.z() * tanHalfFovx)) {
    return false;
  }

  if ((aCam.y() < -aCam.z() * tanHalfFovy &&
       bCam.y() < -bCam.z() * tanHalfFovy &&
       cCam.y() < -cCam.z() * tanHalfFovy) ||
      (aCam.y() > aCam.z() * tanHalfFovy && bCam.y() > bCam.z() * tanHalfFovy &&
       cCam.y() > cCam.z() * tanHalfFovy)) {
    return false;
  }

  return true;
}
