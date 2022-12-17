#include <initializer_list>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;

typedef vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> Trajectory3D;
void draw_trajectory(std::initializer_list<Trajectory3D>);
