#include <la-slam/common.h>
#include <la-slam/data_structures.h>
#include <la-slam/visualize.h>

using namespace std;

#pragma once

#ifndef VO_H
#define VO_H

#define MATCH_MAX_DIST 20
#define MAX_MATCHES 100

// correspondence
void generate_features(Image::Ptr im);

void generate_matches(Image::Ptr im1, Image::Ptr im2,
                      vector<Feature::Ptr> &feats1,
                      vector<Feature::Ptr> &feats2);

// poly mult for five_point alg
Eigen::Matrix<double, 10, 1> p1p1(Eigen::Vector4d a, Eigen::Vector4d b);

template <int len1, int len2>
Eigen::Matrix<double, len1 + len2 - 1, 1>
poly_mult(Eigen::Matrix<double, len1, 1> a, Eigen::Matrix<double, len2, 1> b);

template <int len>
double eval_poly_at(Eigen::Matrix<double, len, 1> p, double val);

Eigen::Matrix<double, 13, 1> sub_z3z4(Eigen::Matrix<double, 10, 1> a,
                                      Eigen::Matrix<double, 10, 1> b);

Eigen::Matrix<double, 20, 1> p2p1(Eigen::Matrix<double, 10, 1> a,
                                  Eigen::Vector4d b);

Eigen::Matrix<double, 10, 1> upper_tri_at(Eigen::Matrix<double, 9, 4> M, int i,
                                          int j);

Eigen::Matrix<double, 10, 1> lambda_at(Eigen::Matrix<double, 9, 4> M, int i,
                                       int j);

void gauss_jordan_partial(Eigen::Matrix<double, 10, 20> &A);

Eigen::Vector4d triangulate_planes(Eigen::Vector3d kp1, Eigen::Vector3d kp2,
                                   Eigen::Matrix3d E,
                                   Eigen::Matrix<double, 3, 4> P);

void five_point(Eigen::Matrix<double, 5, 3> &kps_norm1,
                Eigen::Matrix<double, 5, 3> &kps_norm2,
                vector<Eigen::Affine3d> &transf_hyps);

// Two-view visual odom via epipolar geometry
double epipolar_error(Eigen::Vector3d q1, Eigen::Vector3d q2,
                      Eigen::Affine3d T);

bool epipolar_constraints(Image::Ptr im1, Image::Ptr im2,
                          vector<Feature::Ptr> &feats1,
                          vector<Feature::Ptr> &feats2, Eigen::Affine3d &T);

#endif
