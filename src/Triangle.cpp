#include "Triangle.h"

bool Triangle::intersect(const Ray &ray, Hit &hit) const {
  Eigen::Vector3f edge1 = b - a;
  Eigen::Vector3f edge2 = c - a;

  Eigen::Vector3f h = ray.direction.cross(edge2);
  float det = edge1.dot(h);

  if (fabs(det) < FP_TOLERANCE) {
    return false; 
  }

  float inv_det = 1.0 / det;
  Eigen::Vector3f s = ray.origin - a;
  float u = inv_det * s.dot(h);

  if (u < 0.0 || u > 1.0) {
    return false;
  }

  Eigen::Vector3f q = s.cross(edge1);
  float v = inv_det * ray.direction.dot(q);

  if (v < 0.0 || u + v > 1.0) {
    return false;
  }

  float t = inv_det * edge2.dot(q);

  if (t > FP_TOLERANCE && t < hit.lambda) {
    // Intersection detected, update hit information
    hit.hit = true;
    hit.lambda = t;
    hit.point = ray.origin + t * ray.direction;
    Eigen::Vector3f bary = getBarycentric(hit.point);
    hit.normal = getBarycentricNormal(bary);
    hit.colour = (hit.normal + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.0f;
    return true;
  }

  return false;
}

Eigen::Vector3f Triangle::getBarycentric(const Eigen::Vector3f &p) const {
  Eigen::Vector3f v0v1 = b - a;
  Eigen::Vector3f v0v2 = c - a;
  Eigen::Vector3f v0p = p - a;

  float d00 = v0v1.dot(v0v1);
  float d01 = v0v1.dot(v0v2);
  float d11 = v0v2.dot(v0v2);
  float d20 = v0p.dot(v0v1);
  float d21 = v0p.dot(v0v2);

  float denom = d00 * d11 - d01 * d01;
  float v = (d11 * d20 - d01 * d21) / denom;
  float w = (d00 * d21 - d01 * d20) / denom;
  float u = 1.0f - v - w;

  return Eigen::Vector3f(u, v, w);
}

Eigen::Vector3f
Triangle::getBarycentricNormal(const Eigen::Vector3f &bary) const {
  if (hasNormals) {
    return (bary.x() * nA + bary.y() * nB + bary.z() * nC).normalized();
  } else {
    Eigen::Vector3f edge1 = b - a;
    Eigen::Vector3f edge2 = c - a;
    return edge1.cross(edge2).normalized();
  }
}
