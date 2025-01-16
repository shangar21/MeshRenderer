#include "Renderer.h"
#include <omp.h>

void Renderer::renderRayTrace(const Camera &camera, const Mesh &mesh,
                              Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                              Eigen::MatrixXf &B) {
  float vh = 2.0f * camera.fl;
  float vw = vh * camera.aspectRatio;

  Eigen::Vector3f du = Eigen::Vector3f(vw / camera.imageWidth, 0.0f, 0.0f);
  Eigen::Vector3f dv = Eigen::Vector3f(0.0f, vh / camera.imageHeight, 0.0f);

#pragma omp parallel for
  for (int y = 0; y < camera.imageHeight; y++) {
    for (int x = 0; x < camera.imageWidth; x++) {
      float u = (x - camera.imageWidth / 2.0f) * du[0];
      float v = (y - camera.imageHeight / 2.0f) * dv[1];

      Eigen::Vector3f rayDirCamera(u, v, -camera.fl);
      rayDirCamera.normalize();
      Eigen::Vector4f rayDirCameraHomo(rayDirCamera.x(), rayDirCamera.y(),
                                       rayDirCamera.z(), 0.0f);
      Eigen::Vector3f rayDirWorld =
          (camera.getC2W() * rayDirCameraHomo).head<3>().normalized();

      Ray ray(camera.eye, rayDirWorld);

      Hit hit = mesh.intersect(ray);

      if (hit.hit) {
        R(y, x) = hit.colour.x();
        G(y, x) = hit.colour.y();
        B(y, x) = hit.colour.z();
      } else {
        R(y, x) = 0.0f;
        G(y, x) = 0.0f;
        B(y, x) = 0.0f;
      }
    }
  }
}

void Renderer::renderRayTrace(const Camera &camera, const BVH &bvh,
                              Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                              Eigen::MatrixXf &B) {
  float vh = 2.0f * camera.fl;
  float vw = vh * camera.aspectRatio;

  Eigen::Vector3f du = Eigen::Vector3f(vw / camera.imageWidth, 0.0f, 0.0f);
  Eigen::Vector3f dv = Eigen::Vector3f(0.0f, vh / camera.imageHeight, 0.0f);

#pragma omp parallel for
  for (int y = 0; y < camera.imageHeight; y++) {
    for (int x = 0; x < camera.imageWidth; x++) {
      float u = (x - camera.imageWidth / 2.0f) * du[0];
      float v = (y - camera.imageHeight / 2.0f) * dv[1];

      Eigen::Vector3f rayDirCamera(u, v, -camera.fl);
      rayDirCamera.normalize();
      Eigen::Vector4f rayDirCameraHomo(rayDirCamera.x(), rayDirCamera.y(),
                                       rayDirCamera.z(), 0.0f);
      Eigen::Vector3f rayDirWorld =
          (camera.getC2W() * rayDirCameraHomo).head<3>().normalized();

      Ray ray(camera.eye, rayDirWorld);

      Hit hit;
      bvh.intersect(ray, hit);

      if (hit.hit) {
        R(y, x) = hit.colour.x();
        G(y, x) = hit.colour.y();
        B(y, x) = hit.colour.z();
      } else {
        R(y, x) = 0.0f;
        G(y, x) = 0.0f;
        B(y, x) = 0.0f;
      }
    }
  }
}

Eigen::Vector3f computeBarycentric2D(const Eigen::Vector2f &p,
                                     const Eigen::Vector3f &a,
                                     const Eigen::Vector3f &b,
                                     const Eigen::Vector3f &c) {
  float det =
      (b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y());

  float lambda0 =
      ((b.y() - c.y()) * (p.x() - c.x()) + (c.x() - b.x()) * (p.y() - c.y())) /
      det;
  float lambda1 =
      ((c.y() - a.y()) * (p.x() - c.x()) + (a.x() - c.x()) * (p.y() - c.y())) /
      det;
  float lambda2 = 1.0f - lambda0 - lambda1;

  return Eigen::Vector3f(lambda0, lambda1, lambda2);
}

void Renderer::rasterizeTriangle(
    const std::vector<Eigen::Vector3f> &projectedVertices,
    const Triangle &triangle, Eigen::MatrixXf &R, Eigen::MatrixXf &G,
    Eigen::MatrixXf &B) {
  // Vertices (in pixel space)
  Eigen::Vector3f v0 = projectedVertices[0];
  Eigen::Vector3f v1 = projectedVertices[1];
  Eigen::Vector3f v2 = projectedVertices[2];

  // BBox (in pixel space)
  int xmin = std::max(0, (int)std::floor(std::min({v0.x(), v1.x(), v2.x()})));
  int xmax = std::min((int)R.cols() - 1,
                      (int)std::ceil(std::max({v0.x(), v1.x(), v2.x()})));
  int ymin = std::max(0, (int)std::floor(std::min({v0.y(), v1.y(), v2.y()})));
  int ymax = std::min((int)R.rows() - 1,
                      (int)std::ceil(std::max({v0.y(), v1.y(), v2.y()})));

  for (int y = ymin; y < ymax; y++) {
    for (int x = xmin; x < xmax; x++) {
      // center of pixel
      Eigen::Vector2f p(x + 0.5, y + 0.5);
      // Bary centric location of pixel center relative to triangle
      Eigen::Vector3f bary = computeBarycentric2D(p, v0, v1, v2);
      // check inside tri
      if (bary.x() < 0 || bary.y() < 0 || bary.z() < 0)
        continue;

      // barycentric depth estimate
      float depth = bary.dot(Eigen::Vector3f(v0.z(), v1.z(), v2.z()));
      // check depth is ahead of other triangle
      if (depth > depthBuffer(x, y))
        continue;

      // set depth map and pixel values
      depthBuffer(x, y) = depth;
      R(x, y) = bary.dot(Eigen::Vector3f(triangle.colA.x(), triangle.colB.x(),
                                         triangle.colC.x()));
      G(x, y) = bary.dot(Eigen::Vector3f(triangle.colA.y(), triangle.colB.y(),
                                         triangle.colC.y()));
      B(x, y) = bary.dot(Eigen::Vector3f(triangle.colA.z(), triangle.colB.z(),
                                         triangle.colC.z()));
    }
  }
}

void Renderer::renderRasterize(const Camera &camera, const Mesh &mesh,
                               Eigen::MatrixXf &R, Eigen::MatrixXf &G,
                               Eigen::MatrixXf &B) {

  // Reset depth buffer
  depthBuffer = Eigen::MatrixXf::Constant(camera.imageHeight, camera.imageWidth,
                                          std::numeric_limits<float>::max());

  // Reset frame buffer
  R = Eigen::MatrixXf::Zero(camera.imageHeight, camera.imageWidth);
  G = Eigen::MatrixXf::Zero(camera.imageHeight, camera.imageWidth);
  B = Eigen::MatrixXf::Zero(camera.imageHeight, camera.imageWidth);

  for (const Triangle &triangle : mesh.meshToTriangles()) {
    if (!camera.triangleInView(triangle))
      continue;
    std::vector<Eigen::Vector3f> projectedPoints =
        camera.projectTriangle(triangle);
    rasterizeTriangle(projectedPoints, triangle, R, G, B);
  }
}

void Renderer::saveAsPNG(const Eigen::MatrixXf &R, const Eigen::MatrixXf &G,
                         const Eigen::MatrixXf B,
                         const std::string &filename) const {
  int w = R.cols();
  int h = R.rows();

  cv::Mat image(h, w, CV_8UC3);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      image.at<cv::Vec3b>(y, x)[0] = static_cast<unsigned char>(
          std::min(255.0f, std::max(0.0f, B(y, x) * 255.0f))); // Blue
      image.at<cv::Vec3b>(y, x)[1] = static_cast<unsigned char>(
          std::min(255.0f, std::max(0.0f, G(y, x) * 255.0f))); // Green
      image.at<cv::Vec3b>(y, x)[2] = static_cast<unsigned char>(
          std::min(255.0f, std::max(0.0f, R(y, x) * 255.0f))); // Red
    }
  }

  cv::imwrite(filename, image);
}
