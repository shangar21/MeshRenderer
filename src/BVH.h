#pragma once

#include "BVHNode.h"
#include "Hit.h"
#include "Mesh.h"
#include "Ray.h"
#include "Triangle.h"
#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

class BVH {
public:
  std::shared_ptr<BVHNode> root;
  int maxInLeaf;
  int maxDepth;
  int currentDepth;
  std::string uvMapPath;

  BVH(int maxInLeaf = 10) : maxInLeaf(maxInLeaf) {}
  BVH(std::string texPath, int maxInLeaf = 10)
      : maxInLeaf(maxInLeaf), uvMapPath(texPath) {}
  void buildFromMesh(const Mesh &mesh);
  std::shared_ptr<BVHNode> build(std::vector<Triangle> &triangles, int depth);
  bool intersect(const Ray &ray, Hit &hit) const;

private:
};
