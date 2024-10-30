#pragma once

#include "BVHNode.h"
#include "Hit.h"
#include "Mesh.h"
#include "Ray.h"
#include "Triangle.h"
#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <vector>

class BVH {
public:
  std::shared_ptr<BVHNode> root;
  int maxInLeaf;
  int maxDepth;
  int currentDepth;

  BVH(int maxInLeaf = 10) : maxInLeaf(maxInLeaf) {}
  void buildFromMesh(const Mesh &mesh);
  std::shared_ptr<BVHNode> build(std::vector<Triangle> &triangles, int depth);
  bool intersect(const Ray &ray, Hit &hit) const;

private:
  std::vector<Triangle> meshToTriangles(const Mesh &mesh) const;
  void sortTrianglesByAxis(std::vector<Triangle> &triangles, int axis);
};
