
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>



#include <iostream>
#include <vector>
#include <experimental/filesystem>
#include <string>
#include <unistd.h>

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

// define color space
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar CYAN(255, 255, 0);


void extract_orb_features(Mat& img1, vector<KeyPoint>& keypoints1, Mat& descriptors1) 
{
    Ptr<ORB> orb = ORB::create(MAX_FEATURES);
    orb->detectAndCompute(img1, Mat(), keypoints1, descriptors1);
}

void read_image(const char* filename, Mat& canvas) 
{
    std::string cwd = get_current_dir_name();
    std::string image_path =  cwd + "/../src/experiments/test_img/" + std::string(filename);
    // cout << "DEBUG file dir = " << image_path << endl;
    canvas = cv::imread(image_path, IMREAD_COLOR);
    return ;
}

void read_depth(const char* filename, Mat& canvas) 
{
    std::string cwd = get_current_dir_name();
    std::string image_path =  cwd + "/../src/experiments/test_img/" + std::string(filename);
    canvas = cv::imread(image_path, IMREAD_GRAYSCALE);
    return ; 
}

void keep_points(float discard_precent, size_t width, size_t height, const vector<KeyPoint>& in_kps, vector<Point2i>& out_kps) 
{
    float dw = discard_precent * (float) width; // size of discarding width
    float dh = discard_precent * (float) height; // size of discarding height
    
    //debug 
    cout << "DEBUG dw, dh = " << dw << dh << endl; 
    
    int w_lower, w_upper, h_lower, h_upper;
    w_lower = dw;
    w_upper = width - dw;
    h_lower = dh ;
    h_upper = height - dh;

    cout << "DEBUG dw, width-dw = " << w_lower << " "<< w_upper << endl;
    cout << "DEBUG dh, height-dh = " << h_lower << " " << h_upper << endl;
    //int dummy_counter = 0;
    for(int i = 0; i < in_kps.size(); i++) 
    {   
        Point2i uv = in_kps[i].pt;
        //debug
        // cout << "DEBUG if a,b are true  " << a << "  " << b << endl;
        
        if (((w_lower < uv.x) && (uv.x < w_upper)) && ((h_lower < uv.y) && (uv.y < h_upper))) 
        {
            
            out_kps.push_back(uv);
            //dummy_counter++;
        }
    }
    //cout<<"DEBUG dummy counter " << dummy_counter <<endl;
    return  ;
}

template<typename T>
void keypoint_plotter(Mat& img, vector<T>& points,  
                                char color = 'g', 
                                int radius = 5,
                                int thickness = 2,
                                bool cv_keypoints = false
                                )
{
    
    cv::Scalar plot_color;
    switch (color)
    {
    case 'g': plot_color = GREEN;   break;
    case 'b': plot_color = BLUE;    break;
    case 'r': plot_color = RED;     break;
    case 'c': plot_color = CYAN;    break;
    case 'y': plot_color = YELLOW;  break;
    case 'm': plot_color = MAGENTA; break;
    default:  plot_color = GREEN;   break;
    }
    
    string cwd = get_current_dir_name();
    if (cv_keypoints) {
        for (int i = 0; i < points.size(); i++)
        {
            cv::circle(img, points[i].pt, radius, plot_color, thickness);
        }
    }
    else 
    {
        for (int i = 0; i < points.size(); i++) {
        cv::circle(img, points[i], radius, plot_color, thickness);
        }
    }
    // imwrite(cwd + relative_path, img);
}

int main()
{
    string cwd = get_current_dir_name();
    cout << "cwd = " << cwd <<endl;
    
    // read rgb image
    Mat img1_cvmat, img2_cvmat;
    read_image("img1.png", img1_cvmat);
    read_image("img2.png", img2_cvmat);

    // read depth image
    Mat img1_depth, img2_depth;
    read_depth("img1_depth.png", img1_depth);
    read_depth("img2_depth.png", img2_depth);


    if(img1_cvmat.empty() || img2_cvmat.empty() || img1_depth.empty() || img2_depth.empty())
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

    img1.rgb = img1_cvmat;
    img1.depth = img1_depth;

    img2.rgb = img2_cvmat;
    img2.depth = img2_depth;


    img1.initialize()

    vector<KeyPoint> kps;
    Mat des;
    extract_orb_features(img1_cvmat, kps, des);

    vector<Point2i> kept_kps;
    keep_points(0.4, cam_width, cam_height, kps, kept_kps);

    cout << "num of kept points = " << kept_kps.size() << endl;
    cout << "num of original points = " << kps.size() << endl;
    
    Mat im_display = img1_cvmat;
    
    keypoint_plotter(im_display, kps, 'b', 5, 2, true);
    keypoint_plotter(im_display, kept_kps, 'g');

    //debug
    //bool a = (20<30<40);
    //cout<< "DEBUG " << a << endl;
    
    imwrite(cwd+"/../src/experiments/features.png", im_display);

    return 0;
}
