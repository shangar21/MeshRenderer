#pragma once

#include "Hit.h"
#include "Ray.h"
#include "Triangle.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>

class BVHNode {
public:
  Eigen::Vector3f bboxMin;
  Eigen::Vector3f bboxMax;

  std::shared_ptr<BVHNode> left;
  std::shared_ptr<BVHNode> right;

  std::vector<Triangle> triangles;

  BVHNode(const std::vector<Triangle> &triangles)
      : triangles(triangles), left(nullptr), right(nullptr) {
    setBbox();
  };

  BVHNode(std::shared_ptr<BVHNode> left, std::shared_ptr<BVHNode> right)
      : left(left), right(right) {
    setBbox();
  };

  void setBbox();
  bool checkIsLeaf() const;
  bool intersect(const Ray &ray, Hit &hit, float tMin, float tMax) const;
  bool intersectsBbox(const Ray &ray, float &tMin, float &tMax) const;
};
