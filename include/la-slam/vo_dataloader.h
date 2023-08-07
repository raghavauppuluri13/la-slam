#include <la-slam/data_structures.h>
#include <unordered_map>

#pragma once

#define MAX_IMG_CNT 20

struct DatasetConfig {
    DatasetConfig(const std::string &d, const std::string &img,
                  const std::string &cfg)
        : dataset_name(d), img_dir(img), cfg_dir(cfg) {}
    std::string dataset_name;
    std::string img_dir;
    std::string cfg_dir;
    const std::string img_cfg_name = "images.txt";
    const std::string cam_cfg_name = "cameras.txt";
};
const std::unordered_map<std::string, DatasetConfig> cfg_map = {
    {"delivery_area",
     {"delivery_area", "images", "dslr_calibration_undistorted"}},
    {"courtyard", {"courtyard", "images", "dslr_calibration_undistorted"}},
    {"sandbox",
     {"sandbox/undistorted", "images", "rig_calibration_undistorted"}}};

void dataset_get_camera_cfg(DatasetConfig &cfg, std::string &pth);
void dataset_get_image_cfg(DatasetConfig &cfg, std::string &pth);
void dataset_get_full_img_pth(DatasetConfig &cfg, std::string &rel_pth);

bool parse_cameras(CameraCfgPtrMap &cam_cfg_map, DatasetConfig &cfg);
bool parse_images(vector<Image::Ptr> &im_vec, CameraCfgPtrMap &cam_cfg_map,
                  DatasetConfig &cfg);
