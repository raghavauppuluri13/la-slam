#include <la-slam/data_structures.h>

#pragma once

#define DATASET_PTH "data/undistorted"
#define IMGS_DIR DATASET_PTH "/images"

#define MAX_IMG_CNT 2
#define IMG_CFG DATASET_PTH "/rig_calibration_undistorted/images.txt"
#define CAMERA_CFG DATASET_PTH "/rig_calibration_undistorted/cameras.txt"

bool parse_cameras(CameraCfgPtrMap &cam_cfg_map);
bool parse_images(vector<Image::Ptr> &im_vec, CameraCfgPtrMap &cam_cfg_map);
