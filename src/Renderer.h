#pragma once

#include "BVH.h"
#include "Camera.h"
#include "Hit.h"
#include "Mesh.h"
#include "Ray.h"
#include <opencv2/opencv.hpp>

class Renderer {
public:
  void renderRayTrace(const Camera &camera, const Mesh &mesh,
                      Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                      Eigen::MatrixXf &B);

  void renderRayTrace(const Camera &camera, const BVH &bvh, Eigen::MatrixXf &R,
                      Eigen::MatrixXf &G, Eigen::MatrixXf &B);

  void renderRasterize(const Camera &camera, const Mesh &mesh,
                       Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                       Eigen::MatrixXf &B);

  void saveAsPNG(const Eigen::MatrixXf &R, const Eigen::MatrixXf &G,
                 const Eigen::MatrixXf B, const std::string &filename) const;
};
