#include "Ray.h"

Ray::Ray(const Eigen::Vector3f &orig, const Eigen::Vector3f &dir)
    : origin(orig), direction(dir.normalized()) {}

Eigen::Vector3f Ray::pointAt(float lambda) const {
  return origin + lambda * direction;
}
