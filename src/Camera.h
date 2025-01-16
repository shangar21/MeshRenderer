#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Triangle.h"

class Camera {
public:
  Eigen::Vector3f eye;
  Eigen::Vector3f target;
  Eigen::Vector3f up;

  float fl;
  int imageWidth;
  int imageHeight;
  float aspectRatio;
	float near;
	float far;
	float fovx;
	float fovy;

  Camera(const Eigen::Vector3f &p, const Eigen::Vector3f &g,
         const Eigen::Vector3f &up, float fl, int imageWidth, int imageHeight, float near = 0.1, float far = 10);
  Eigen::Matrix4f getC2W() const;
  Eigen::Matrix4f getW2C() const;
	Eigen::Matrix4f getProjMatrix() const;
  void move(const Eigen::Vector3f &delta);
  void rotate(float theta,
              const Eigen::Vector3f &axis = Eigen::Vector3f::UnitY());
	std::vector<Eigen::Vector3f> projectTriangle(const Triangle triangle) const;
	bool triangleInView(const Triangle triangle) const;
};
