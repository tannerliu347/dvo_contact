#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

#include "rgbd_image.h"

// 1341839846.7712 -2.3142 -2.2665 1.9327 0.9424 0.0850 -0.0028 -0.3235

// 1341839847.2712 -2.3366 -2.1170 1.9314 0.9437 0.0751 0.0092 -0.3221

// 1341839846.769925.png==img1.png 1341839847.273912.png==img2.png 


using namespace cv;
using namespace std;
using namespace cv::features2d;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;

void extract_orb_features(Mat& img1, vector<KeyPoint>& keypoints1, Mat& descriptors1)
{
    Ptr<ORB> orb = ORB::create(MAX_FEATURES);
    orb->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
}



Mat read_image(const char* filename)
{
    std::string image_path = samples::findFile(filename);
    Mat img = imread(image_path, IMREAD_GRAYSCALE);
    return img;
}



int main()
{
    Mat img1 = read_image("img1.png");
    Mat img2 = read_image("img2.png");
    if(img1.empty() || img2.empty())
    {
        std::cout << "Image load failed!" << std::endl;
        return -1;
    }

    dvo::Intrinsic cam_intrinsic = new dvo::Intrinsic();
    cam_intrinsic.


    size_t cam_width = 640;
    size_t cam_height = 480;
    camera = dvo::RgbdCamera()

    
    
    return 0;
}