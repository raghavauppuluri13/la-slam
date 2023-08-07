#include <la-slam/vo_dataloader.h>

void dataset_get_camera_cfg(DatasetConfig &cfg, std::string &pth) {
    std::string root = getenvstr("ROOT_DIR");
    pth = root + "/data/" + cfg.dataset_name + "/" + cfg.cfg_dir + "/" +
          cfg.cam_cfg_name;
}

void dataset_get_image_cfg(DatasetConfig &cfg, std::string &pth) {
    std::string root = getenvstr("ROOT_DIR");
    pth = root + "/data/" + cfg.dataset_name + "/" + cfg.cfg_dir + "/" +
          cfg.img_cfg_name;
}

void dataset_get_full_img_pth(DatasetConfig &cfg, std::string &rel_pth) {
    std::string root = getenvstr("ROOT_DIR");
    rel_pth = root + "/data/" + cfg.dataset_name + "/" + rel_pth;
}

bool parse_cameras(CameraCfgPtrMap &cam_cfg_map, DatasetConfig &data_cfg) {
    std::string camera_cfg_pth;
    dataset_get_camera_cfg(data_cfg, camera_cfg_pth);

    ifstream fin(camera_cfg_pth);
    if (!fin) {
        cout << "path doesn't exist: " << camera_cfg_pth << endl;
        return false;
    }
    while (!fin.eof() && !fin.bad()) {
        string line;
        getline(fin, line);

        if (line.size() == 0 || line[0] == '#') {
            continue;
        }

        string model_str;
        auto cfg = make_shared<CameraCfg>();
        istringstream cam_stream(line);
        cam_stream >> cfg->id >> model_str >> cfg->w >> cfg->h;
        vector<double> params;
        while (!cam_stream.eof() && !cam_stream.bad()) {
            params.emplace_back();
            cam_stream >> params.back();
        }
        cfg->model = PinholeCameraModel(params);
        // cout << cfg->id << endl;
        cam_cfg_map.insert(make_pair(cfg->id, cfg));
    }
    return true;
}

bool parse_images(vector<Image::Ptr> &im_vec, CameraCfgPtrMap &cam_cfg_map,
                  DatasetConfig &data_cfg) {

    std::string img_cfg_pth;
    dataset_get_image_cfg(data_cfg, img_cfg_pth);

    ifstream fin(img_cfg_pth);

    if (!fin) {
        cout << "path doesn't exist: " << img_cfg_pth << endl;
        return false;
    }
    int cnt = 0;
    while (!fin.eof() && !fin.bad() && im_vec.size() <= MAX_IMG_CNT) {
        string line;
        getline(fin, line);
        if (line.size() == 0 || line[0] == '#') {
            continue;
        }

        string img_file;
        auto im = make_shared<Image>();
        double qw, qx, qy, qz;
        int cam_id;
        istringstream img_stream(line);
        img_stream >> im->id >> qw >> qx >> qy >> qz >>
            im->T_cam_global.translation()[0] >>
            im->T_cam_global.translation()[1] >>
            im->T_cam_global.translation()[2] >> cam_id >> img_file;
        im->T_cam_global.linear() = Eigen::Quaterniond(qw, qx, qy, qz).matrix();
        im->T_global_cam = im->T_cam_global.inverse();
        stringstream img_file_stream;
        img_file_stream << data_cfg.img_dir << "/" << img_file;
        img_file = img_file_stream.str();
        dataset_get_full_img_pth(data_cfg, img_file);
        im->im = cv::imread(img_file);
        im->cam_cfg = cam_cfg_map[cam_id];

        if (im->im.data == nullptr) {
            cout << "file does not exist: " << img_file << endl;
            continue;
        }

        im_vec.push_back(im);
        cnt++;
        getline(fin, line); // skip observations
    }
    fin.close();

    return true;
}
