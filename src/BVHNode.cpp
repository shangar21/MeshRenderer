#include "BVHNode.h"

bool BVHNode::checkIsLeaf() const {
  return left == nullptr && right == nullptr;
}

void BVHNode::setBbox() {
  if (checkIsLeaf()) {
    bboxMin = triangles[0].a;
    bboxMax = triangles[0].a;

    for (const Triangle triangle : triangles) {
      bboxMin = bboxMin.cwiseMin(triangle.a)
                    .cwiseMin(triangle.b)
                    .cwiseMin(triangle.c);
      bboxMax = bboxMax.cwiseMax(triangle.a)
                    .cwiseMax(triangle.b)
                    .cwiseMax(triangle.c);
    }
  } else {
    bboxMin = left->bboxMin.cwiseMin(right->bboxMin);
    bboxMax = left->bboxMax.cwiseMax(right->bboxMax);
  }
}

bool BVHNode::intersect(const Ray &ray, Hit &hit, float tMin,
                        float tMax) const {
  if (!intersectsBbox(ray, tMin, tMax))
    return false;

  if (!checkIsLeaf()) {
    bool hitLeft = left && left->intersect(ray, hit, tMin, tMax);
    bool hitRight = right && right->intersect(ray, hit, tMin, tMax);
    return hitLeft || hitRight;
  }

  bool intersect = false;
  for (const Triangle &triangle : triangles) {
    Hit tmp;
    if (triangle.intersect(ray, tmp) && tmp.lambda < hit.lambda) {
      hit = tmp;
      intersect = true;
    }
  }

  return intersect;
}

bool BVHNode::intersectsBbox(const Ray &ray, float &tMin, float &tMax) const {
  for (int i = 0; i < 3; i++) {
    float invD = 1.0f / ray.direction[i];
    float t0 = (bboxMin[i] - ray.origin[i]) * invD;
    float t1 = (bboxMax[i] - ray.origin[i]) * invD;
    if (invD < 0.0f)
      std::swap(t0, t1);
    tMin = t0 > tMin ? t0 : tMin;
    tMax = t1 < tMax ? t1 : tMax;
    if (tMax <= tMin)
      return false;
  }
  return true;
}
