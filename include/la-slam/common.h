// std
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <unistd.h>
#include <unordered_map>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#pragma once

#define EPSIL 1e-6

using namespace std;

// Eigen typedefs
typedef Eigen::Transform<double, 3, Eigen::Affine> SE3d;

// helper functions
cv::Point2f eigen_to_pt(Eigen::Vector3d v);
bool is_zero(double n);

Eigen::Matrix3d skew_sym(Eigen::Vector3d v);
