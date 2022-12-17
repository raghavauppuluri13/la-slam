#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <unistd.h>
#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>
#include <math.h>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/eigen.hpp>

using namespace std;

#ifndef VO_H 
#define VO_H

#define DATASET_PTH "data/undistorted"
#define IMGS_DIR DATASET_PTH "/images"

#define MAX_IMG_CNT 5
#define IMG_CFG DATASET_PTH "/rig_calibration_undistorted/images.txt"
#define CAMERA_CFG DATASET_PTH "/rig_calibration_undistorted/cameras.txt"

#define MATCH_MAX_DIST 30

typedef Eigen::Transform<double, 3, Eigen::Affine> SE3d;

struct Image;
struct Feature;

struct Image {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<Image> Ptr;
    int id;
    SE3d T_cam_global;
    SE3d T_global_cam;
    cv::Mat im;
    int cam_id;
    vector<shared_ptr<Feature>> features;
    cv::Mat descriptors; 
};


struct PinholeCameraModel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Matrix3d K;
    double fx,fy,cx,cy;

    PinholeCameraModel() {}
    PinholeCameraModel(vector<double> params) {
        assert(params.size() == 4);
        fx = params[0]; fy = params[1]; 
        cx = params[2]; cy = params[3];
        K << fx, 0, cx, 
             0, fy, cy, 
             0, 0, 1;
    }
    Eigen::Vector3d project(Eigen::Vector3d pt) {
        Eigen::Vector3d uv = K * pt;
        uv = uv / uv(2);
        return uv;
    }

    Eigen::Vector3d deproject(Eigen::Vector3d uv_norm) {
        return K.inverse() * uv_norm;
    }
}; 

struct CameraCfg {
    typedef shared_ptr<CameraCfg> Ptr;
    int id;
    int w;
    int h;
    PinholeCameraModel model;
};

struct Feature {
    typedef shared_ptr<Feature> Ptr;
    cv::KeyPoint kp;
    weak_ptr<Image> im;
    bool is_outlier;

    Feature() {}
    Feature(Image::Ptr im_, cv::KeyPoint kp_) : im(im_), kp(kp_) {}
};

typedef unordered_map<int, CameraCfg::Ptr> CameraCfgPtrMap;

void triangulate();

#endif
