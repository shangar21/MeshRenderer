#pragma once

#include "Hit.h"
#include "Ray.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>

#define FP_TOLERANCE 1e-6

class Triangle {
public:
  Eigen::Vector3f a, b, c;
  Eigen::Vector3f nA, nB, nC;
  bool hasNormals = false;
  Eigen::Vector2f tA, tB, tC;
  bool hasTexCoords = false;
  Eigen::Vector3f colA, colB, colC;
	bool hasColours = false;

  Triangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b,
           const Eigen::Vector3f &c)
      : a(a), b(b), c(c) {}

  Triangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b,
           const Eigen::Vector3f &c, const Eigen::Vector3f &nA,
           const Eigen::Vector3f &nB, const Eigen::Vector3f &nC)
      : a(a), b(b), c(c), nA(nA), nB(nB), nC(nC), hasNormals(true) {}

  Triangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b,
           const Eigen::Vector3f &c, const Eigen::Vector3f &nA,
           const Eigen::Vector3f &nB, const Eigen::Vector3f &nC,
           const Eigen::Vector2f &tA, const Eigen::Vector2f &tB,
           const Eigen::Vector2f &tC)
      : a(a), b(b), c(c), nA(nA), nB(nB), nC(nC), tA(tA), tB(tB), tC(tC),
        hasNormals(true), hasTexCoords(true) {}

  Eigen::Vector3f centroid() const { return (a + b + c) / 3.0f; }

  bool intersect(const Ray &ray, Hit &hit) const;
  Eigen::Vector3f getBarycentric(const Eigen::Vector3f &point) const;

  Eigen::Vector3f
  getBarycentricNormal(const Eigen::Vector3f &barycentric) const;

  Eigen::Vector3f
  getBarycentricColour(const Eigen::Vector3f &barycentric) const;
	
	bool isPointInside(const Eigen::Vector3f& p) const;
};
