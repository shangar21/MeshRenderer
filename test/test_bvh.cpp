// test_bvh.cpp
#include "BVH.h"
#include "Hit.h"
#include "Mesh.h"
#include "Ray.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

void testBVHConstruction() {
  // Create a mesh object and populate with some simple triangles
  Mesh mesh;
  mesh.vertices = Eigen::MatrixXf(4, 3);
  mesh.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1;

  mesh.faces.push_back(Eigen::Vector3i(0, 1, 2));
  mesh.faces.push_back(Eigen::Vector3i(1, 2, 3));

  // Construct BVH
  BVH bvh(1); // maxInLeaf = 1 to create more nodes
  bvh.buildFromMesh(mesh);

  // Print BVH structure
  std::cout << "BVH constructed successfully!" << std::endl;
}

void testBVHIntersection() {
  // Create a mesh object with some triangles
  Mesh mesh;
  mesh.vertices = Eigen::MatrixXf(3, 3);
  mesh.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0;

  mesh.faces.push_back(Eigen::Vector3i(0, 1, 2));

  // Construct BVH
  BVH bvh(1);
  bvh.buildFromMesh(mesh);

  // Define a ray that intersects the BVH
  Ray ray(Eigen::Vector3f(0.25f, 0.25f, -1.0f),
          Eigen::Vector3f(0.0f, 0.0f, 1.0f));

  // Check intersection with BVH
  Hit hit;
  bool result = bvh.intersect(ray, hit);

  // Output result
  if (result) {
    std::cout << "Intersection detected at: " << hit.point.transpose()
              << std::endl;
  } else {
    std::cout << "No intersection detected with BVH." << std::endl;
  }
}

int main() {
  testBVHConstruction();
  testBVHIntersection();
  return 0;
}
