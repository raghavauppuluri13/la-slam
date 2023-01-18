#include <la-slam/simpletest.h>
#include <la-slam/vo.h>

DEFINE_TEST_G(TestPolynomialMult4x4, VO) {
    Eigen::Vector4d p1, p2;
    // x+2y+3z+4
    p1 << 1, 2, 3, 4;
    // 5x+6y+7z+8
    p2 << 5, 6, 7, 8;

    // 5 x^2 + 16 x y + 22 x z + 28 x + 12 y^2 + 32 y z + 40 y + 21 z^2 + 52 z +
    // 32
    Eigen::Matrix<double, 10, 1> corr;
    corr << 5, 12, 21, 16, 22, 32, 28, 40, 52, 32;
    auto out = p1p1(p1, p2);
    TEST_MATd_EQ(out, corr, 10, 1, 10, 1);
}

DEFINE_TEST_G(TestPolynomialMult10x4, VO) {
    Eigen::Matrix<double, 10, 1> p1;
    Eigen::Vector4d p2;

    // x2+2y2+3z2+4xy+5xz+6yz+7x+8y+9z+10
    p1 << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
    // 11x+12y+13z+14
    p2 << 11, 12, 13, 14;

    // 11 x^3 + 24 y^3 + 39 z^3 + 56 x^2 y + 70 x y^2 + 68 x^2 z + 98 x z^2 +
    // 98 y^2 z + 114 y z^2 + 178 x y z + 91 x^2 + 124 y^2 + 159 z^2 + 228 x y
    //  + 260 x z + 296 y z + 208 x + 232 y + 256 z + 140
    //
    Eigen::Matrix<double, 20, 1> corr;
    corr << 11, 24, 39, 56, 70, 68, 98, 98, 114, 178, 91, 124, 159, 228, 260,
        296, 208, 232, 256, 140;
    auto out = p2p1(p1, p2);
    TEST_MATd_EQ(out, corr, 20, 1, 20, 1);
}

DEFINE_TEST_G(TestPolynomialSubtractAzB, VO) {
    Eigen::Matrix<double, 10, 1> a, b;

    // 5z2x+zx+16x+13z2y+12zy+4y+2z3+12z2+2z+10
    a << 5, 1, 16, 13, 12, 4, 2, 12, 2, 10;
    // 1z2x+2zx+3x+4z2y+5zy+6y+7z3+8z2+9z+20
    b << 1, 2, 3, 4, 5, 6, 7, 8, 9, 20;

    // -x z^3 + 3 x z^2 - 2 x z + 16 x - 4 y z^3 + 8 y z^2 + 6 y z + 4 y - 7 z^4
    // - 6 z^3 + 3 z^2 - 18 z + 10
    Eigen::Matrix<double, 13, 1> corr;
    corr << -1, 3, -2, 16, -4, 8, 6, 4, -7, -6, 3, -18, 10;
    auto out = sub_z3z4(a, b);
    TEST_MATd_EQ(out, corr, 13, 1, 13, 1);
}
