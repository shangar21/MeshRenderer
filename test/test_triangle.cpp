// test_triangle.cpp
#include "Hit.h"
#include "Ray.h"
#include "Triangle.h"
#include <Eigen/Dense>
#include <iostream>

void testTriangleIntersection() {
  // Define vertices of a triangle
  Eigen::Vector3f a(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f b(1.0f, 0.0f, 0.0f);
  Eigen::Vector3f c(0.0f, 1.0f, 0.0f);

  // Create a triangle
  Triangle triangle(a, b, c);

  // Define a ray that intersects the triangle
  Ray ray(Eigen::Vector3f(0.25f, 0.25f, -1.0f),
          Eigen::Vector3f(0.0f, 0.0f, 1.0f));

  ray.origin = Eigen::Vector3f(0.25f, 0.25f, -1.0f);
  ray.direction = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

  // Perform intersection test
  Hit hit;
  bool result = triangle.intersect(ray, hit);

  // Output results
  if (result) {
    std::cout << "Intersection detected at: " << hit.point.transpose()
              << std::endl;
    std::cout << "Intersection lambda (distance): " << hit.lambda << std::endl;
    std::cout << "Barycentric coordinates: "
              << triangle.getBarycentric(hit.point).transpose() << std::endl;
  } else {
    std::cout << "No intersection detected." << std::endl;
  }
}

int main() {
  testTriangleIntersection();
  return 0;
}
