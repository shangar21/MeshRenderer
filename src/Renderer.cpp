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

      Eigen::Vector3f rayDirCamera (u, v, -camera.fl);
			rayDirCamera.normalize();
			Eigen::Vector4f rayDirCameraHomo(rayDirCamera.x(), rayDirCamera.y(), rayDirCamera.z(), 0.0f);
			Eigen::Vector3f rayDirWorld = (camera.getC2W() * rayDirCameraHomo).head<3>().normalized();

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
