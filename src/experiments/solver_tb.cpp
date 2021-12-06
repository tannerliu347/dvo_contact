#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


// #include <filesystem>
#include <iostream>
#include <vector>

#include "rgbd_image.h"

// 1341839846.7712 -2.3142 -2.2665 1.9327 0.9424 0.0850 -0.0028 -0.3235

// 1341839847.2712 -2.3366 -2.1170 1.9314 0.9437 0.0751 0.0092 -0.3221

// 1341839846.769925.png==img1.png 1341839847.273912.png==img2.png 


using namespace cv;
using namespace std;
// namespace fs = std::filesystem;
// using namespace cv::xfeatures2d;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;

void extract_orb_features(Mat& img1, vector<KeyPoint>& keypoints1, Mat& descriptors1)
{
    Ptr<ORB> orb = ORB::create(MAX_FEATURES);
    orb->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
}


Mat read_image(const char* filename)
{
    std::string image_path = filename;
    Mat img = imread(image_path, IMREAD_COLOR);
    return img;
}



int main()
{
    Mat img1_cvmat = read_image("../dvo_contact/experiments/test_img/img1.png");
    Mat img2_cvmat = read_image("../dvo_contact/experiments/test_img/img1.png");
    if(img1_cvmat.empty() || img2_cvmat.empty())
    {
        std::cout << "Image load failed!" << std::endl;
        return -1;
    }

    float fx = 535.4;
    float fy = 539.2;
    float ox = 320.1;
    float oy = 247.6;


    Eigen::Matrix3f intrinsic_mat;
    intrinsic_mat << fx, ox, 0.f, 0.f, fy, oy, 0.f, 0.f, 0.f;
    
    dvo::Intrinsic cam_intrinsic(intrinsic_mat);
    // cam_intrinsic =  dvo::Intrinsic::Intrinsic(intrinsic_mat);


    size_t cam_width = 640;
    size_t cam_height = 480;
    dvo::RgbdCamera camera(cam_width, cam_height, cam_intrinsic);
    
    dvo::RgbdImage img1(camera);
    dvo::RgbdImage img2(camera);

    img1.intensity = img1_cvmat;

    vector<KeyPoint> kps;
    Mat des;
    extract_orb_features(img1_cvmat, kps, des);

    cout << "DEBUG, check keypoints " << kps[0].pt <<endl;


    
    
    return 0;
}
