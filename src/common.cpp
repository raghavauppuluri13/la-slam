#include <la-slam/common.h>

bool is_zero(double n) { return abs(n) < EPSIL; }

cv::Point2f eigen_to_pt(Eigen::Vector3d v) { return cv::Point2f(v(0), v(1)); }

Eigen::Matrix3d skew_sym(Eigen::Vector3d v) {
    Eigen::Matrix3d t_skew;
    t_skew << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return t_skew;
}

std::string getenvstr(std::string const &key) {
    char *val = getenv(key.c_str());
    return val == NULL ? std::string("") : std::string(val);
}
