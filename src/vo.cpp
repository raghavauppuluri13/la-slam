// VO implementation of:
// Nistér, D. (2004). An efficient solution to the five-point relative pose
// problem. IEEE Transactions on Pattern Analysis and Machine Intelligence, 26,
// 756-770.

#include "la-slam/vo.h"
#include <cstdlib>

using namespace std;

void generate_features(Image::Ptr im) {
    vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    detector->detect(im->im, keypoints);
    descriptor->compute(im->im, keypoints, im->descriptors);

    for (int i = 0; i < keypoints.size(); i++) {
        Feature::Ptr f(new Feature(im, keypoints[i]));
        im->features.push_back(f);
    }
}

void generate_matches(Image::Ptr im1, Image::Ptr im2,
                      vector<Feature::Ptr> &feats1,
                      vector<Feature::Ptr> &feats2) {
    vector<cv::DMatch> matches, good_matches;
    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(im1->descriptors, im2->descriptors, matches);

    sort(matches.begin(), matches.end(),
         [](cv::DMatch l, cv::DMatch r) { return l.distance < r.distance; });

    for (auto match : matches) {
        if (match.distance <= max(2.0 * matches[0].distance, 30.0)) {
            good_matches.push_back(match);
        } else {
            im1->features[match.queryIdx]->is_outlier = true;
            im2->features[match.trainIdx]->is_outlier = true;
        }
    }

    random_shuffle(good_matches.begin(), good_matches.end());
    vector<cv::KeyPoint> kps1, kps2;
    for (auto match : good_matches) {
        feats1.push_back(im1->features[match.queryIdx]);
        feats2.push_back(im2->features[match.trainIdx]);
    }
    for (auto f : im1->features) {
        kps1.push_back(f->kp);
    }
    for (auto f : im2->features) {
        kps2.push_back(f->kp);
    }
    /*
    cv::Mat img_match;
    cv::drawMatches(im1->im, kps1, im2->im, kps2, good_matches, img_match);
    cv::imshow("all matches", img_match);
    cv::waitKey(0);
    */
}

// compute poly mult [x,y,z,1] x [x,y,z,1]
// out = [x2,y2,z2,xy,xz,yz,x,y,z,1]
Eigen::Matrix<double, 10, 1> p1p1(Eigen::Vector4d a, Eigen::Vector4d b) {
    Eigen::Matrix<double, 10, 1> out = Eigen::Matrix<double, 10, 1>::Zero();
    out(0) = a(0) * b(0);               // x2
    out(1) = a(1) * b(1);               // y2
    out(2) = a(2) * b(2);               // z2
    out(3) = a(0) * b(1) + a(1) * b(0); // xy
    out(4) = a(0) * b(2) + a(2) * b(0); // xz
    out(5) = a(1) * b(2) + a(2) * b(1); // yz
    out(6) = a(3) * b(0) + a(0) * b(3); // x
    out(7) = a(3) * b(1) + a(1) * b(3); // y
    out(8) = a(3) * b(2) + a(2) * b(3); // z
    out(9) = a(3) * b(3);               // 1
    return out;
}

// compute poly mult [z3,z2,z,1] x [z3,z2,z,1]
// z2,z,1
// z2,z,1
// z4,z3,z2
// z3,z2,z
// z2,z,1
template <int len1, int len2>
Eigen::Matrix<double, len1 + len2 - 1, 1>
poly_mult(Eigen::Matrix<double, len1, 1> a, Eigen::Matrix<double, len2, 1> b) {
    Eigen::Matrix<double, len1 + len2 - 1, 1> out =
        Eigen::Matrix<double, len1 + len2 - 1, 1>::Zero();
    for (int i = 0; i < len1; i++) {
        for (int j = 0; j < len2; j++) {
            out(len1 + len2 - 2 - i - j) += a(len1 - 1 - i) * b(len2 - 1 - j);
        }
    }
    return out;
}

template <int len>
double eval_poly_at(Eigen::Matrix<double, len, 1> p, double val) {
    double out = 0;
    for (int i = 0; i < len; i++) {
        out += p(len - 1 - i) * pow(val, i);
    }
    return out;
}

// Computes out = a - z * b
// a,b = [z2x,zx,x,z2y,zy,y,z3,z2,z,1]
// zb = [z3x,z2x,zx,z3y,z2y,zy,z4,z3,z2,z]
// out = [z3x,z2x,zx,x,z3y,z2y,zy,y,z4,z3,z2,z,1]
Eigen::Matrix<double, 13, 1> sub_z3z4(Eigen::Matrix<double, 10, 1> a,
                                      Eigen::Matrix<double, 10, 1> b) {
    Eigen::Matrix<double, 13, 1> out = Eigen::Matrix<double, 13, 1>::Zero();
    out(0) = -b(0);        // z3x
    out(1) = a(0) - b(1);  // z2x
    out(2) = a(1) - b(2);  // zx
    out(3) = a(2);         // x
    out(4) = -b(3);        // z3y
    out(5) = a(3) - b(4);  // z2y
    out(6) = a(4) - b(5);  // zy
    out(7) = a(5);         // y
    out(8) = -b(6);        // z4
    out(9) = a(6) - b(7);  // z3
    out(10) = a(7) - b(8); // z2
    out(11) = a(8) - b(9); // z
    out(12) = a(9);        // 1
    return out;
}

// compute poly mult [x2,y2,z2,xy,xz,yz,x,y,z,1] * [x,y,z,1]
// out = [x3,y3,z3,x2y,y2x,x2z,z2x,z2y,y2z,xyz,x2,y2,z2,xy,xz,yz,x,y,1]
Eigen::Matrix<double, 20, 1> p2p1(Eigen::Matrix<double, 10, 1> a,
                                  Eigen::Vector4d b) {
    Eigen::Matrix<double, 20, 1> out = Eigen::Matrix<double, 20, 1>::Zero();
    out(0) = a(0) * b(0);                              // x3
    out(1) = a(1) * b(1);                              // y3
    out(2) = a(2) * b(2);                              // z3
    out(3) = a(0) * b(1) + a(3) * b(0);                // x2y
    out(4) = a(1) * b(0) + a(3) * b(1);                // y2x
    out(5) = a(0) * b(2) + a(4) * b(0);                // x2z
    out(6) = a(2) * b(0) + a(4) * b(2);                // z2x
    out(7) = a(1) * b(2) + a(5) * b(1);                // y2z
    out(8) = a(2) * b(1) + a(5) * b(2);                // z2y
    out(9) = a(3) * b(2) + a(4) * b(1) + a(5) * b(0);  // xyz
    out(10) = a(0) * b(3) + a(6) * b(0);               // x2
    out(11) = a(1) * b(3) + a(7) * b(1);               // y2
    out(12) = a(2) * b(3) + a(8) * b(2);               // z2
    out(13) = a(3) * b(3) + a(6) * b(1) + a(7) * b(0); // xy
    out(14) = a(4) * b(3) + a(6) * b(2) + a(8) * b(0); // xz
    out(15) = a(5) * b(3) + a(7) * b(2) + a(8) * b(1); // yz
    out(16) = a(6) * b(3) + a(9) * b(0);               // x
    out(17) = a(7) * b(3) + a(9) * b(1);               // y
    out(18) = a(8) * b(3) + a(9) * b(2);               // z
    out(19) = a(9) * b(3);                             // 1
    return out;
}

Eigen::Matrix<double, 10, 1> upper_tri_at(Eigen::Matrix<double, 9, 4> M, int i,
                                          int j) {
    Eigen::Matrix<double, 10, 1> out = Eigen::Matrix<double, 10, 1>::Zero();
    for (int k = 0; k < 3; k++) {
        out += p1p1(M.row(i * 3 + k), M.row(j * 3 + k));
    }
    return out;
}

Eigen::Matrix<double, 10, 1> lambda_at(Eigen::Matrix<double, 9, 4> M, int i,
                                       int j) {
    Eigen::Matrix<double, 10, 1> out = Eigen::Matrix<double, 10, 1>::Zero();
    if (i == j) {
        Eigen::Matrix<double, 10, 1> diag_sum =
            Eigen::Matrix<double, 10, 1>::Zero();
        for (int k = 0; k < 3; k++) {
            diag_sum += upper_tri_at(M, k, k);
        }
        out = upper_tri_at(M, i, j) - 0.5 * diag_sum;
    } else {
        out = upper_tri_at(M, i, j);
    }
    return out;
}

// Gauss Jordan Elimination with partial pivoting (stopping 4 rows before end)
void gauss_jordan_partial(Eigen::Matrix<double, 10, 20> &A) {
    Eigen::FullPivLU<Eigen::Matrix<double, 10, 20>> lu(A);
    // upper tri matrix of A
    Eigen::Matrix<double, 10, 20> U =
        lu.matrixLU().triangularView<Eigen::Upper>();

    // backsubstitution
    for (int i = 9; i > 3; i--) {
        for (int j = 9; j > i; j--) {
            U.row(i) -= U.row(j) * U(i, j);
        }
        U.row(i) /= U(i, i);
    }
}

Eigen::Vector4d triangulate_planes(Eigen::Vector3d kp1, Eigen::Vector3d kp2,
                                   Eigen::Matrix3d E,
                                   Eigen::Matrix<double, 3, 4> P) {
    Eigen::Vector3d a, b, c, d;
    Eigen::Vector4d C, Q;
    a = E.transpose() * kp2;
    Eigen::Vector3d diag;
    diag << 1, 1, 0;
    b = kp1.cross(diag.asDiagonal() * a);
    c = kp2.cross(diag.asDiagonal() * E * kp1);
    C = P.transpose() * c;
    d = a.cross(b);
    Q.head<3>() = d.transpose() * C(3);
    Q(3) = -1 * (d.dot(C.head<3>()));
    return Q;
}

void five_point(Eigen::Matrix<double, 5, 3> &kps_norm1,
                Eigen::Matrix<double, 5, 3> &kps_norm2,
                vector<Eigen::Affine3d> &transf_hyps) {
    Eigen::Matrix<double, 5, 9> input_mat;

    for (int i = 0; i < 5; i++) {
        Eigen::Vector3d q = kps_norm1.row(i), q_prime = kps_norm2.row(i);
        input_mat.row(i) << q(0) * q_prime(0), q(1) * q_prime(0),
            q(2) * q_prime(0), q(0) * q_prime(1), q(1) * q_prime(1),
            q(2) * q_prime(1), q(0) * q_prime(2), q(1) * q_prime(2),
            q(2) * q_prime(2);
    }

    // Extract right nullspace of input_mat
    Eigen::MatrixXd Q;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 9, 5>> qr(
        input_mat.transpose());

    Q = qr.householderQ();
    Eigen::Matrix<double, 9, 4> Q_nul = Q.rightCols<4>();

    Eigen::Matrix3d X(Q_nul.col(0).data()), Y(Q_nul.col(1).data()),
        Z(Q_nul.col(2).data()), W(Q_nul.col(3).data());
    Eigen::Matrix<double, 9, 4> E_hat;

    int E_i = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            E_hat.row(E_i) << X(i, j), Y(i, j), Z(i, j), W(i, j);
            E_i++;
        }
    }

    Eigen::Matrix<double, 10, 20> A = Eigen::Matrix<double, 10, 20>::Zero();

    // compute epipolar constraint
    int A_i = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                A.row(A_i) +=
                    p2p1(lambda_at(E_hat, i, k), E_hat.row(k * 3 + j));
            }
            A_i++;
        }
    }

    // compute det(E) = 0 constraint
    A.row(9) = p2p1(p1p1(E_hat.row(1), E_hat.row(5)) -
                        p1p1(E_hat.row(2), E_hat.row(4)),
                    E_hat.row(6)) +
               p2p1(p1p1(E_hat.row(2), E_hat.row(3)) -
                        p1p1(E_hat.row(0), E_hat.row(5)),
                    E_hat.row(7)) +
               p2p1(p1p1(E_hat.row(0), E_hat.row(4)) -
                        p1p1(E_hat.row(1), E_hat.row(3)),
                    E_hat.row(8));

    Eigen::Matrix<double, 10, 20> A_hat;

    // move x * deg(z,2), y * deg(z,2), 1 * deg(z,3) to end of matrix
    // from =
    // [0-x3,1-y3,2-z3,3-x2y,4-y2x,5-x2z,6-z2x,7-y2z,8-z2y,9-xyz,10-x2,11-y2,12-z2,13-xy,14-xz,15-yz,16x,17y,18z,19-1]
    // to   =
    // [x3,y3,|x2y,y2x,x2z,y2z,xyz|,|x2,y2,xy|,|z2x,zx,x|,|z2y,zy,y|,|z3,z2,z,1|]
    A_hat = A(Eigen::indexing::all, {0, 1,  3,  4, 5,  7,  9, 10, 11, 13,
                                     6, 14, 16, 8, 15, 17, 2, 12, 18, 19});
    gauss_jordan_partial(A_hat);

    Eigen::Matrix<double, 3, 13> B;
    Eigen::Matrix<double, 3, 4> Bx, By;
    Eigen::Matrix<double, 3, 5> B1;

    B.row(0) = sub_z3z4(A_hat.row(4).tail<10>(), A_hat.row(5).tail<10>());
    B.row(1) = sub_z3z4(A_hat.row(6).tail<10>(), A_hat.row(7).tail<10>());
    B.row(2) = sub_z3z4(A_hat.row(8).tail<10>(), A_hat.row(9).tail<10>());

    for (int i = 0; i < 3; i++) {
        Bx.row(i) = B.row(i).segment<4>(0);
        By.row(i) = B.row(i).segment<4>(1);
        B1.row(i) = B.row(i).tail<5>();
    }

    // n = det(B)
    Eigen::Matrix<double, 8, 1> p1 = poly_mult<4, 5>(By.row(0), B1.row(1)) -
                                     poly_mult<5, 4>(B1.row(0), By.row(1));
    Eigen::Matrix<double, 8, 1> p2 = poly_mult<5, 4>(B1.row(0), Bx.row(1)) -
                                     poly_mult<4, 5>(Bx.row(0), B1.row(1));
    Eigen::Matrix<double, 7, 1> p3 = poly_mult<4, 4>(Bx.row(0), By.row(1)) -
                                     poly_mult<4, 4>(By.row(0), Bx.row(1));
    Eigen::Matrix<double, 11, 1> n = poly_mult<8, 4>(p1, Bx.row(2)) +
                                     poly_mult<8, 4>(p2, By.row(2)) +
                                     poly_mult<7, 5>(p3, B1.row(2));

    // normalize
    n /= n(0);

    // Solve for roots
    Eigen::Matrix<double, 10, 10> comp_mat =
        Eigen::Matrix<double, 10, 10>::Zero();

    comp_mat.row(0) = -1 * n.tail<10>();
    comp_mat.block<9, 9>(1, 0) = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::ComplexEigenSolver<Eigen::Matrix<double, 10, 10>> eigensolver(
        comp_mat);
    if (eigensolver.info() != Eigen::Success) {
        return;
    }
    Eigen::Matrix<complex<double>, 10, 1> roots = eigensolver.eigenvalues();

    // generate transformation hypotheses
    for (int i = 0; i < 10; i++) {
        if (!is_zero(roots(i).imag())) {
            continue;
        }
        double z = roots(i).real();
        double x = eval_poly_at<8>(p1, z) / eval_poly_at<7>(p3, z);
        double y = eval_poly_at<8>(p2, z) / eval_poly_at<7>(p3, z);

        // Calculate E
        Eigen::Matrix3d E = x * X + y * Y + z * Z + W;
        Eigen::IOFormat fmt(3, 0, ", ", "\n", "[", "]");

        // svd decomp to get U,V, enforce diag(U) = 1,1,0
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU |
                                                     Eigen::ComputeFullV);

        Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();

        if (U.determinant() < 0) {
            U.col(2) *= -1;
        }
        if (V.determinant() < 0) {
            V.col(2) *= -1;
        }

        Eigen::Vector3d diag;
        diag << 1, 1, 0;
        E = U * diag.asDiagonal() * V.transpose();

        // Calculate Pa = [Ra|t]
        Eigen::Matrix3d D;
        Eigen::Matrix<double, 3, 4> P;
        D << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        P.topLeftCorner<3, 3>() = U * D * V.transpose(); // rot
        P.col(3) = U.col(2);                             // trans

        Eigen::Vector4d Q =
            triangulate_planes(kps_norm1.row(0), kps_norm2.row(0), E, P);

        double c1 = Q(2) * Q(3);
        double c2 = (P * Q)(2) * Q(3);

        Eigen::Matrix4d Hr = Eigen::Matrix4d::Identity(); // reflection
        Hr(3, 3) *= -1;

        Eigen::Matrix4d Ht =
            Eigen::Matrix4d::Identity(); // twisted pair transformation for R
        Eigen::Vector3d ht_diag;
        ht_diag << -1, -1, 1;

        Ht.topLeftCorner<3, 3>() = V * ht_diag.asDiagonal() * V.transpose();

        Eigen::Matrix4d Ht_q =
            Eigen::Matrix4d::Identity(); // twisted pair transformation for Q
        Ht_q.row(3).head<3>() = -2 * V.col(2);

        // select valid transformation which results in pnts in front of both
        // img planes, initially P = Pa
        if (c1 < 0 && c2 < 0) {
            // add reflection Pb = (Hr * Pa.T).T
            P = (Hr * P.transpose()).transpose();
        } else if (c1 * c2 < 0) {
            // twisted pair Pc = Pb * Ht
            P = P * Ht;
            Q = Ht_q * Q;
            if (Q(2) * Q(3) < 0) {
                // add reflection Pd = (Hr * Pc.T).T
                P = (Hr * P.transpose()).transpose();
            }
        }
        Eigen::Affine3d T;
        T.linear() = P.topLeftCorner<3, 3>();
        T.translation() = P.col(3);
        transf_hyps.push_back(T);
    }
}

double epipolar_error(Eigen::Vector3d q1, Eigen::Vector3d q2,
                      Eigen::Affine3d T) {
    Eigen::Matrix3d t_skew = skew_sym(T.translation().head<3>());
    Eigen::Vector3d e3;
    e3 << 0, 0, 1;
    Eigen::Matrix3d e3_skew_sym = skew_sym(e3);
    double cost_numerator = q2.dot(t_skew * T.linear() * q1);
    Eigen::Vector3d cost_denom_1 = e3_skew_sym * t_skew * T.linear() * q1;
    Eigen::Vector3d cost_denom_2 =
        q2.transpose() * t_skew * T.linear() * e3_skew_sym.transpose();
    return cost_numerator * cost_numerator / cost_denom_1.squaredNorm() +
           cost_numerator * cost_numerator / cost_denom_2.squaredNorm();
}

#define N 100
#define B 15
#define get_stage(i, M) int(M * pow(2, -1 * (i / B)))

bool epipolar_constraints(Image::Ptr im1, Image::Ptr im2,
                          vector<Feature::Ptr> &feats1,
                          vector<Feature::Ptr> &feats2, Eigen::Affine3d &T) {
    assert(feats1.size() >= N);
    vector<Eigen::Vector3d> kps_norm1, kps_norm2;
    vector<Eigen::Affine3d> tfs;
    double time_used_av = 0;
    for (int i = 1; i < N; i++) {
        Eigen::Matrix<double, 5, 3> kps1, kps2;
        for (int j = 0; j < 5; j++) {
            kps1.row(j) = feats1[i + j]->to_hom();
            kps2.row(j) = feats2[i + j]->to_hom();
        }
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        five_point(kps1, kps2, tfs);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used =
            chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        time_used_av += time_used.count();
    }

    vector<int> obs_inds(N);
    std::iota(begin(obs_inds), end(obs_inds), 0);
    random_shuffle(begin(obs_inds), end(obs_inds));

    vector<pair<double, int>> sorted_hyps;
    for (int i = 0; i < tfs.size(); i++) {
        pair<double, int> p;
        p = make_pair(0, i);
        sorted_hyps.push_back(p);
    }

    for (int i = 0; i < N && get_stage(i, tfs.size()) != 1; i++) {
        int obs_ind = obs_inds[i];
        int curr_block = get_stage(i, tfs.size());
        for (int h = 0; h < curr_block; h++) {
            Eigen::Vector3d q6 = feats1[obs_ind]->to_hom();
            Eigen::Vector3d q6_prime = feats2[obs_ind]->to_hom();
            double err = epipolar_error(q6, q6_prime, tfs[h]);
            sorted_hyps[h].first = err;
        }
        sort(sorted_hyps.begin(), sorted_hyps.begin() + curr_block,
             [](pair<double, int> l, pair<double, int> r) {
                 return l.first < r.first;
             });
    }
    T = tfs[sorted_hyps[0].second];

    time_used_av /= N;
    cout << "Error: " << sorted_hyps[0].first << endl;
    // cout << "five_point avg: " << time_used_av << "s" << endl;

    return true;
}
#undef N
#undef B
#undef get_stage
