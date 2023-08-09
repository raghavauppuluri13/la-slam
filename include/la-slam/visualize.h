#include <Eigen/Core>
#include <initializer_list>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;

typedef vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> Trajectory3D;
void draw_trajectory(Trajectory3D, Trajectory3D);
