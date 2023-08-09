#include <la-slam/visualize.h>
#include <la-slam/vo.h>
#include <la-slam/vo_dataloader.h>

int main(int argc, char **argv) {

    CameraCfgPtrMap cam_cfg_map;
    vector<Image::Ptr> im_vec;

    Trajectory3D poses;
    Trajectory3D poses_gt;

    std::string dataset_name = getenvstr("DATASET_NAME");

    DatasetConfig dataset_cfg = cfg_map.at(dataset_name);

    parse_cameras(cam_cfg_map, dataset_cfg);
    parse_images(im_vec, cam_cfg_map, dataset_cfg);

    Eigen::Isometry3d Twr;
    Twr.translation() = im_vec[0]->T_global_cam.translation();
    Twr.linear() = im_vec[0]->T_global_cam.rotation();

    poses.push_back(Twr);
    poses_gt.push_back(Twr);

    for (int i = 1; i < im_vec.size(); i++) {
        Image::Ptr F = im_vec[i - 1];
        Image::Ptr F_prime = im_vec[i];
        generate_features(F);
        generate_features(F_prime);
        Eigen::Affine3d T;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        vector<Feature::Ptr> feats1, feats2;
        generate_matches(F, F_prime, feats1, feats2);

        Eigen::Isometry3d Twr_gt;
        Eigen::Affine3d Twr_gt_rel;
        Twr_gt.translation() = F_prime->T_global_cam.translation();
        Twr_gt.linear() = F_prime->T_global_cam.rotation();
        Twr_gt_rel = poses_gt[poses_gt.size() - 1].inverse() * Twr_gt.matrix();
        poses_gt.push_back(Twr_gt);

        bool found_T =
            epipolar_constraints(F, F_prime, feats1, feats2, T, Twr_gt_rel);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used =
            chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "epipolar: " << 1.0 / time_used.count() << " Hz" << endl;
        assert(found_T);

        Twr = Twr.matrix() * T.matrix().inverse();
        poses.push_back(Twr);

        // cout << "T=" << endl << T << endl;
    }

    draw_trajectory(poses_gt, poses);

    return 0;
}
