#include <la-slam/data_structures.h>
#include <la-slam/simpletest.h>
#include <la-slam/visualize.h>
#include <la-slam/vo_dataloader.h>

class VO_Dataloader : TestFixture {
  public:
    CameraCfgPtrMap cam_cfg_map;
    vector<Image::Ptr> im_vec;
    Trajectory3D poses;
    Trajectory3D poses_gt;

    void Setup() override {
        parse_cameras(cam_cfg_map);
        parse_images(im_vec, cam_cfg_map);
    }
    void TearDown() override { draw_trajectory({poses, poses_gt}); }
};
