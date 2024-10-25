#include "Hit.h"

Hit::Hit(bool hit, const Eigen::Vector3f &point, const Eigen::Vector3f &normal,
         float lambda, const Eigen::Vector3f &colour)
    : hit(hit), point(point), normal(normal), lambda(lambda), colour(colour) {}
