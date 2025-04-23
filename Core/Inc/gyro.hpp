#pragma once

#include <cmath>

#include <Eigen/Geometry>

inline float quaternion_to_yaw(const Eigen::Quaternionf &quat) {
  Eigen::Matrix3f mat = quat.toRotationMatrix();
  return -std::atan2(mat(1, 0), mat(0, 0));
}

inline float zero_to_2pi(float angle) {
  float res = std::fmod(angle, 2.0f * M_PI);
  if (res < 0) {
    return res + 2.0f * M_PI;
  }
  return res;
}