#pragma once

#include <Eigen/Dense>

class Ray{
	public:
		Eigen::Vector3f origin;
		Eigen::Vector3f direction;

		Ray(const Eigen::Vector3f& orig, const Eigen::Vector3f& direction);

		Eigen::Vector3f pointAt(float lambda) const;
};
