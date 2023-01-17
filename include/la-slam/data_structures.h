#include <la-slam/common.h>

#pragma once

struct Image;
struct Feature;
struct PinholeCameraModel;
struct CameraCfg;

struct Image {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef shared_ptr<Image> Ptr;
    int id;
    SE3d T_cam_global;
    SE3d T_global_cam;
    cv::Mat im;
    shared_ptr<CameraCfg> cam_cfg;
    vector<shared_ptr<Feature>> features;
    cv::Mat descriptors;
};

struct PinholeCameraModel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Matrix3d K;
    double fx, fy, cx, cy;

    PinholeCameraModel() {}
    PinholeCameraModel(vector<double> params) {
        assert(params.size() == 4);
        fx = params[0];
        fy = params[1];
        cx = params[2];
        cy = params[3];
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
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

    Eigen::Vector3d to_hom() {
        shared_ptr<Image> im_shared = im.lock();
        assert(im_shared);
        return im_shared->cam_cfg->model.deproject(
            Eigen::Vector3d(kp.pt.x, kp.pt.y, 1));
    }
};

typedef unordered_map<int, CameraCfg::Ptr> CameraCfgPtrMap;
