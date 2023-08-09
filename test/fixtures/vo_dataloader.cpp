#include <la-slam/data_structures.h>
#include <la-slam/simpletest.h>
#include <la-slam/visualize.h>
#include <la-slam/vo_dataloader.h>

class VO_Dataloader : TestFixture {
  public:
    CameraCfgPtrMap cam_cfg_map;
    vector<Image::Ptr> im_vec;
    DatasetConfig data_cfg;
    Trajectory3D poses;
    Trajectory3D poses_gt;

    void Setup() override {
        parse_cameras(cam_cfg_map, data_cfg);
        parse_images(im_vec, cam_cfg_map, data_cfg);
    }
    void TearDown() override { draw_trajectory(poses_gt, poses); }
};
