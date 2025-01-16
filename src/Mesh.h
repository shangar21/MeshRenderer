#pragma once

#define FP_TOLERANCE 1e-6

#include "Hit.h"
#include "Ray.h"
#include "Triangle.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class Mesh {
public:
  Eigen::MatrixXf vertices;
  Eigen::MatrixXf normals;
  Eigen::MatrixXf texcoords;

  std::vector<Eigen::Vector3i> faces;
  std::vector<Eigen::Vector3i> faceNormals;
  std::vector<Eigen::Vector3i> faceTexture;

	std::string uvMapPath;

  Mesh() = default;
	Mesh(std::string texPath) : uvMapPath(texPath){};

  bool loadFromObj(const std::string &filename);
  void printMeshInfo() const;
  Hit intersect(const Ray &ray) const;
  std::vector<Triangle> meshToTriangles() const;
	void parseUVMap(std::vector<Triangle> &triangles) const; 
	static void sortTrianglesByAxis(std::vector<Triangle> &triangles, int axis);
};
