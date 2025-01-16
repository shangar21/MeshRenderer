#include "BVH.h"

void BVH::buildFromMesh(const Mesh &mesh) {
  std::vector<Triangle> triangles = mesh.meshToTriangles();
  root = build(triangles, 0);
}

std::shared_ptr<BVHNode> BVH::build(std::vector<Triangle> &triangles,
                                    int depth) {
  if (triangles.size() <= maxInLeaf) {
    return std::make_shared<BVHNode>(triangles);
  }

  Mesh::sortTrianglesByAxis(triangles, depth % 3);
  int mid = triangles.size() / 2;
  std::vector<Triangle> leftSubset(triangles.begin(), triangles.begin() + mid);
  std::vector<Triangle> rightSubset(triangles.begin() + mid, triangles.end());

  std::shared_ptr<BVHNode> left = build(leftSubset, depth + 1);
  std::shared_ptr<BVHNode> right = build(rightSubset, depth + 1);

  std::shared_ptr<BVHNode> node = std::make_shared<BVHNode>(left, right);
  return node;
}

bool BVH::intersect(const Ray &ray, Hit &hit) const {
  hit.lambda = std::numeric_limits<float>::infinity();
  return root->intersect(ray, hit, 0.0, hit.lambda);
}
