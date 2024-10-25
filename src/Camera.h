#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class Camera {
public:
  Eigen::Vector3f eye;
  Eigen::Vector3f target;
  Eigen::Vector3f up;

  float fl;
  int imageWidth;
  int imageHeight;
  float aspectRatio;

  Camera(const Eigen::Vector3f &p, const Eigen::Vector3f &g,
         const Eigen::Vector3f &up, float fl, int imageWidth, int imageHeight);
  Eigen::Matrix4f getC2W() const;
  Eigen::Matrix4f getW2C() const;
  void move(const Eigen::Vector3f &delta);
  void rotate(float theta,
              const Eigen::Vector3f &axis = Eigen::Vector3f::UnitY());
};
