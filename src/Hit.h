#pragma once

#include <Eigen/Dense>
#include <limits>

class Hit {
public:
  bool hit;
  Eigen::Vector3f point;
  Eigen::Vector3f normal;
  float lambda;
  Eigen::Vector3f colour;

  Hit(bool hit = false, const Eigen::Vector3f &point = Eigen::Vector3f::Zero(),
      const Eigen::Vector3f &normal = Eigen::Vector3f::Zero(),
      float lambda = std::numeric_limits<float>::max(),
      const Eigen::Vector3f &colour = Eigen::Vector3f::Zero());
};
