#include "Camera.h"

Camera::Camera(const Eigen::Vector3f &p, const Eigen::Vector3f &g,
               const Eigen::Vector3f &t, float fl, int imageWidth,
               int imageHeight)
    : eye(p), target(g), up(t), fl(fl), imageWidth(imageWidth),
      imageHeight(imageHeight) {
  aspectRatio = (float)imageWidth / (float)imageHeight;
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
  c2w.block<3, 1>(0, 3) = eye;

  return c2w;
}

Eigen::Matrix4f Camera::getW2C() const { return getC2W().inverse(); }
