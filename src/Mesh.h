#pragma once

#define FP_TOLERANCE 1e-6

#include "Hit.h"
#include "Ray.h"
#include "../cuda/src/renderer.cuh"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <cuda_runtime.h>

class Mesh {
public:
  Eigen::MatrixXf vertices;
  Eigen::MatrixXf normals;
  Eigen::MatrixXf texcoords;

  std::vector<Eigen::Vector3i> faces;
  std::vector<Eigen::Vector3i> faceNormals;
  std::vector<Eigen::Vector3i> faceTexture;

  Mesh() = default;

  bool loadFromObj(const std::string &filename);
  void printMeshInfo() const;
  Hit intersect(const Ray &ray) const;
	void renderWithCuda(const camera& cam);
};
