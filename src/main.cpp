#include <la-slam/visualize.h>
#include <la-slam/vo.h>
#include <la-slam/vo_dataloader.h>

int main(int argc, char **argv) {

    CameraCfgPtrMap cam_cfg_map;
    vector<Image::Ptr> im_vec;

    Trajectory3D poses;
    Trajectory3D poses_gt;

    parse_cameras(cam_cfg_map);
    parse_images(im_vec, cam_cfg_map);

    Eigen::Isometry3d Twr;
    Twr.translation() = im_vec[0]->T_global_cam.translation();
    Twr.linear() = im_vec[0]->T_global_cam.rotation();

    poses.push_back(Twr);

    for (int i = 0; i < im_vec.size() - 1; i++) {
        generate_features(im_vec[i]);
        generate_features(im_vec[i + 1]);
        Eigen::Affine3d T;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        vector<Feature::Ptr> feats1, feats2;
        generate_matches(im_vec[i], im_vec[i + 1], feats1, feats2);
        bool found_T =
            epipolar_constraints(im_vec[i], im_vec[i + 1], feats1, feats2, T);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used =
            chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        // cout << "epipolar: " << 1.0 / time_used.count() << " Hz" << endl;
        assert(found_T);

        Twr = Twr * T.matrix();
        poses.push_back(Twr);

        Eigen::Isometry3d Twr_gt;
        Twr_gt.translation() = im_vec[i]->T_global_cam.translation();
        Twr_gt.linear() = im_vec[i]->T_global_cam.rotation();
        poses_gt.push_back(Twr_gt);
        // cout << "T=" << endl << T << endl;
    }

    draw_trajectory({poses, poses_gt});

    return 0;
}
