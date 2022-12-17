#include "graphslam/vo.h"
#include "graphslam/visualize.h"

using namespace std;

bool parse_cameras(CameraCfgPtrMap& cfg_ptr) {
    ifstream fin(CAMERA_CFG);
    if(!fin) {
        cout << "path doesn't exist: " << CAMERA_CFG << endl; 
        return false;
    }
    while(!fin.eof() && !fin.bad()) {
        string line;
        getline(fin,line);

        if(line.size() == 0 || line[0] == '#') {
            continue;
        }

        string model_str;
        auto cfg = make_shared<CameraCfg>();
        istringstream cam_stream(line);
        cam_stream >> cfg->id >> model_str
                    >> cfg->w >> cfg->h;
        vector<double> params;
        while (!cam_stream.eof() && !cam_stream.bad()) {
              params.emplace_back();
              cam_stream >> params.back();
        }
        cfg->model = PinholeCameraModel(params);
        // cout << cfg->id << endl;
        cfg_ptr.insert(std::make_pair(cfg->id, cfg));
    }
    return true;
}


bool parse_images(vector<Image::Ptr>& im_vec) { 

    ifstream fin(IMG_CFG);

    if(!fin) {
        cout << "path doesn't exist: " << IMG_CFG << endl; 
        return false;
    } 
    int cnt = 0;
    while(!fin.eof() && !fin.bad() && im_vec.size() <= MAX_IMG_CNT) {
        string line; getline(fin,line); 
        if(line.size() == 0 || line[0] == '#') {
            continue;
        }

        string img_file;
        auto im = make_shared<Image>();
        double qw, qx, qy, qz;
        istringstream img_stream(line);
        img_stream >> im->id
            >> qw >> qx >> qy >> qz
            >> im->T_cam_global.translation()[0] 
            >> im->T_cam_global.translation()[1] 
            >> im->T_cam_global.translation()[2] 
            >> im->cam_id >> img_file;
        im->T_cam_global.linear() = Eigen::Quaterniond(qw, qx, qy, qz).matrix();
        im->T_global_cam = im->T_cam_global.inverse(); 
        stringstream img_file_stream;
        img_file_stream << IMGS_DIR << "/" << img_file;
        img_file = img_file_stream.str();
        im->im = cv::imread(img_file);

        if(im->im.data == nullptr) {
            cout << "file does not exist" << img_file << endl;
            continue;
        }

        im_vec.push_back(im);
        cnt++;
        getline(fin,line); // skip observations
    }
    fin.close();

    return true;
}


void generate_features(Image::Ptr im) {
    vector<cv::KeyPoint> keypoints;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();

    detector->detect(im->im, keypoints);
    descriptor->compute(im->im, keypoints, im->descriptors);

    for(int i = 0; i < keypoints.size(); i++) {
        Feature::Ptr f(new Feature(im, keypoints[i]));
        im->features.push_back(f);
    }
}

void triangulate_dlt(vector<cv::Point2f>& pts1, vector<cv::Point2f>& pts2, Eigen::Matrix<double,3,4>& P1, Eigen::Matrix<double,3,4>& P2, Eigen::Vector4d& X) {
    const int MATCHES = 20;
    pts1.resize(MATCHES);
    pts2.resize(MATCHES);

    Eigen::Matrix<double, MATCHES * 4, 4> A = Eigen::Matrix<double, MATCHES * 4, 4>::Zero();
    cv::Point2f com1(0,0), com2(0,0);
    double scale1 = 0, scale2 = 0;

    for(int i = 0; i < pts1.size(); i++) {
        com1 += pts1[i];
        com2 += pts2[i];
    }
    com1 *= 1./pts1.size();
    com2 *= 1./pts2.size();
    
    for(int i = 0; i < pts1.size(); i++) {
        cv::Point2f centered_pt = pts1[i] - com1;
        scale1 += sqrt(centered_pt.x * centered_pt.x + centered_pt.y * centered_pt.y);

        centered_pt = pts2[i] - com2;
        scale2 += sqrt(centered_pt.x * centered_pt.x + centered_pt.y * centered_pt.y);
    }

    scale1 *= 1./pts1.size();
    scale2 *= 1./pts2.size();

    scale1 = sqrt(2)/scale1;
    scale2 = sqrt(2)/scale2;

    for(int i = 0; i < pts1.size(); i++) {
        cv::Point2d p1 = pts1[i] - com1, p2 = pts2[i] - com2;
        p1 *= scale1;
        p2 *= scale2;
        //cv::Point2d p1 = pts1[i], p2 = pts2[i];

        Eigen::Matrix4d A_i;
        A_i << p1.x * P1.row(2) - P1.row(0),
                            p1.y * P1.row(2) - P1.row(1), 
                            p2.x * P2.row(2) - P2.row(0), 
                            p2.y * P2.row(2) - P2.row(1); 
        A.block<4,4>(i,0) = A_i;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    Eigen::Matrix3d T_denorm_1, T_denorm_2;
    T_denorm_1 << scale1, 0, -scale1*com1.x, 0, scale1, -scale1*com1.y, 0, 0, 1;
    T_denorm_2 << scale2, 0, -scale2*com2.x, 0, scale2, -scale2*com2.y, 0, 0, 1;

    X = svd.matrixV().rightCols<1>();
    X /= X(3);
    //cout << "x" << endl << X.head<3>() << endl;
    //X.head<3>() << T_denorm_2.inverse() *  * T_denorm_1 X.head<3>();

}

bool epipolar_constraints(CameraCfgPtrMap& cam_cfg_map, Image::Ptr im1, Image::Ptr im2, Eigen::Matrix4d& T) {
    std::vector<cv::DMatch> matches, good_matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->match(im1->descriptors, im2->descriptors, matches);

    sort(matches.begin(), matches.end(), 
        [](cv::DMatch l, cv::DMatch r) {
            return l.distance < r.distance;
        }
    );

    vector<cv::Point2f> pts1, pts2;
    for(int i = 0; i < matches.size(); i++) {
        int p1_i = matches[i].trainIdx, p2_i = matches[i].queryIdx;
        if(matches[i].distance <= std::max(2.0 * matches[0].distance, 30.0)) {
            pts1.push_back(im1->features[p1_i]->kp.pt);
            pts2.push_back(im2->features[p2_i]->kp.pt);
            good_matches.push_back(matches[i]);
        }
    }

    /*
    vector<cv::KeyPoint> kps1,kps2;
    for(int i = 0; i < im1->features.size(); i++) kps1.push_back(im1->features[i]->kp);
    for(int i = 0; i < im2->features.size(); i++) kps2.push_back(im2->features[i]->kp);
    cv::Mat img_match;
    cv::drawMatches(im1->im, kps1, im2->im, kps2, good_matches, img_match); 
    cv::imshow("all matches", img_match);
    cv::waitKey(0);
    */

    PinholeCameraModel K1 = cam_cfg_map[im1->cam_id]->model;
    PinholeCameraModel K2 = cam_cfg_map[im1->cam_id]->model;

    cv::Point2d principal_pt(K1.cx,K1.cy);
    cv::Mat E_cv = cv::findEssentialMat(pts1,pts2,K1.fx,principal_pt);
    Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> > E( (double*)E_cv.data ); 

    //cout << "E =" << std::endl << E << endl;
    //cout << "det(E) =" << std::endl << E.determinant() << endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
    sigma.diagonal() << svd.singularValues();
    //cout << "sigma =" << std::endl << sigma << endl;
    
    double POSSIBLE_ANGLES[2] = {-M_PI/2,M_PI/2};
    Eigen::Vector3d axis(0,0,1);
    Eigen::Matrix4d T_11 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_mat, t_skew;
    Eigen::Vector3d t;

    bool found = false;
    for(double R_angle : POSSIBLE_ANGLES) {
        Eigen::AngleAxis<double> R_rot(R_angle, axis);
        for(double t_angle : POSSIBLE_ANGLES) {
            Eigen::AngleAxis<double> t_rot(t_angle, axis);

            R_mat = svd.matrixU() * (R_rot.toRotationMatrix().transpose() 
                                    * svd.matrixV().transpose());
            t_skew = svd.matrixU() * (t_rot.toRotationMatrix() 
                                    * (sigma * svd.matrixU().transpose()));
            t << t_skew(1,2) * -1, t_skew(0,2), t_skew(1,0);

            T_12 = Eigen::Matrix4d::Identity();
            T_12.topLeftCorner<3,3>() = R_mat;
            T_12.topRightCorner<3,1>() = t;

            Eigen::Matrix<double,3,4> K1_hat = Eigen::Matrix<double,3,4>::Zero();
            Eigen::Matrix<double,3,4> K2_hat = Eigen::Matrix<double,3,4>::Zero();
            K1_hat.topLeftCorner<3,3>() = K1.K;
            K2_hat.topLeftCorner<3,3>() = K2.K;

            Eigen::Matrix<double,3,4> P1 = K1_hat * T_11, P2 = K2_hat * T_12;
            Eigen::Vector4d X = Eigen::Vector4d::Ones();
            triangulate_dlt(pts1,pts2,P1, P2, X);

            //cout << "X=" << endl << X << endl;

            int positive_cnt = 0;
            for(int i = 0; i < 3; i++) if(X(i) > 0) positive_cnt++;
            if(positive_cnt == 2) {
                T = T_12;
                found = true; 
                break;
            } 
        }
        if(found) {
            break;
        }
    }

    //cout << "T=" << endl << T << endl;

    for(int i = 0; i < pts1.size(); i++) {
       cv::Point2d p1 = pts1[i], p2 = pts2[i];
       Eigen::Vector3d uv1, uv2;
       uv1 << p1.x,p1.y,1;
       uv2 << p2.x,p2.y,1;
       Eigen::Vector3d uv_norm_1, uv_norm_2;
       uv_norm_1 = K1.deproject(uv1);
       uv_norm_2 = K1.deproject(uv2);
       //cout << "epipolar_constraints: " << uv_norm_2.transpose() * t_skew * R_mat * uv_norm_1 << endl; 
    } 

    return found;
}



int main(int argc, char** argv) {

    CameraCfgPtrMap cam_cfg_map;
    vector<Image::Ptr> im_vec;

    Trajectory3D poses;
    Trajectory3D poses_gt;

    parse_cameras(cam_cfg_map);
    parse_images(im_vec);
    
    Eigen::Isometry3d Twr;
    Twr.translation() = im_vec[0]->T_global_cam.translation();
    Twr.linear() = im_vec[0]->T_global_cam.rotation();

    poses.push_back(Twr);

    for(int i = 0; i < im_vec.size() - 1; i++) {
        generate_features(im_vec[i]);
        generate_features(im_vec[i+1]);
        Eigen::Matrix4d T;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        bool found_T = epipolar_constraints(cam_cfg_map, im_vec[i], im_vec[i+1], T);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
        //cout << "epipolar: " << 1.0 / time_used.count() << " Hz" << endl;
        assert(found_T);

        Twr = Twr * T;
        poses.push_back(Twr);

        Eigen::Isometry3d Twr_gt;
        Twr_gt.translation() = im_vec[i]->T_global_cam.translation(); 
        Twr_gt.linear() = im_vec[i]->T_global_cam.rotation(); 
        poses_gt.push_back(Twr_gt);
        //cout << "T=" << endl << T << endl;
    }


    draw_trajectory({poses, poses_gt});
    
    return 0;
}
