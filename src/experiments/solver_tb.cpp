#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>


#include <iostream>
#include <vector>
#include <experimental/filesystem>
#include <string>
#include <unistd.h>

#include "rgbd_image.h"
#include "FrontendSolver.h"
#include "utils.hpp"
// img1.png 1341839846.7712 // tx = -2.3142 ty = -2.2665 tz = 1.9327   
                            // qx = 0.9424 qy = 0.0850 qz = -0.0028 qw = -0.3235

// img2.png 1341839847.2712 // tx = -2.3366 ty = -2.1170 tz = 1.9314   
                            // qx = 0.9437 qy = 0.0751 qz = 0.0092 qw = -0.3221

// 1341839846.769925.png==img1.png 1341839847.273912.png==img2.png 


using namespace cv;
using namespace std;

// namespace fs = std::filesystem;
// using namespace cv::xfeatures2d;

const int MAX_FEATURES = 500; // for orb_features 
const float GOOD_MATCH_PERCENT = 0.15f;


// define color space
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar CYAN(255, 255, 0);

// define scaling factor
const float DEPTH_SCALE = 1.f / 5000;

// define image size
const size_t cam_width = 640;
const size_t cam_height = 480;

void extract_orb_features(Mat& img1, vector<KeyPoint>& keypoints1, Mat& descriptors1);

void read_image(const char* filename, Mat& canvas);

void read_intensity(const char* filename, Mat& canvas);

void read_depth(const char* filename, Mat& canvas);

void keep_points(float discard_precent, size_t width, size_t height, const vector<KeyPoint>& in_kps, vector<Point2i>& out_kps) ;

template<typename T>
void keypoint_plotter(Mat& img, vector<T>& points,  
                                char color = 'g', 
                                int radius = 5,
                                int thickness = 2);

void feature_point_to_point_cloud (const std::vector<cv::Point2i>& kept_pts,
                                        const dvo::PointCloud& img_pc,
                                        dvo::PointCloud& kept_pc,
                                        const size_t camera_width, 
                                        const size_t camera_height);

void feature_point_to_intensity (const std::vector<cv::Point2i>& kept_pts, 
                                    const cv::Mat& img_intensity,
                                    Eigen::VectorXd& intensity) ;


int main()
{
    //compute ground truth transform  [qw, qx, qy, qz, tx, ty, tz]
    // img1.png 1341839846.7712 // tx = -2.3142 ty = -2.2665 tz = 1.9327   
                                // qx = 0.9424 qy = 0.0850 qz = -0.0028 qw = -0.3235

    // img2.png 1341839847.2712 // tx = -2.3366 ty = -2.1170 tz = 1.9314   
                                // qx = 0.9437 qy = 0.0751 qz = 0.0092 qw = -0.3221
    
    //double q_t1[7] = {-0.3235, 0.9424, 0.0850, -0.0028, -2.3142, -2.2665, 1.9327};
    //double q_t2[7] = {-0.3221, 0.9437, 0.0751, 0.0092, -2.3366, -2.1170, 1.9314};

    double q_t1[7] = {1, 0, 0, 0, 1, 2, 3};
    double q_t2[7] = {1, 0, 0, 0, 0.5, 1.5, 2.5};
    // double q_tdiff;
    double q_21[7] = {};
    calc_transform_from_quaternion(q_t1, q_t2, q_21, true);

    string cwd = get_current_dir_name();
    cout << "cwd = " << cwd <<endl;
    
    // read rgb image
    Mat img1_cvmat, img2_cvmat;
    read_image("img1.png", img1_cvmat);
    read_image("img2.png", img2_cvmat);

    //read intensity image
    Mat img1_intensity_discard, img2_intensity_discard;
    read_intensity("img1.png", img1_intensity_discard);
    read_intensity("img2.png", img2_intensity_discard);

    cv::Mat img1_intensity, img2_intensity;
    img1_intensity_discard.convertTo(img1_intensity, CV_32FC1);
    img2_intensity_discard.convertTo(img2_intensity, CV_32FC1);
    

    // read depth image
    // Mat img1_depth(cam_width, cam_height, CV_32FC3), img2_depth(cam_width, cam_height, CV_32FC3); //seg fault declaration
    // Mat img1_depth_prescale, img2_depth_prescale; // faultless declaration
    Mat img1_depth_unscale, img2_depth_unscale;
    // read_depth("img1_depth.png", img1_depth_prescale);
    // read_depth("img2_depth.png", img2_depth_prescale);

    read_depth("img1_depth.png", img1_depth_unscale);
    read_depth("img2_depth.png", img2_depth_unscale);

    // scale the depth
    // cv::Mat img1_depth(cam_width, cam_height, CV_64F), img2_depth(cam_width, cam_height, CV_64F);
    // img1_depth = img1_depth_prescale;
    // img2_depth = img2_depth_prescale;
    cv::Mat img1_depth; // = img1_depth_unscale * DEPTH_SCALE;
    cv::Mat img2_depth; // = img2_depth_unscale * DEPTH_SCALE;
    img1_depth_unscale.convertTo(img1_depth, CV_32FC1);
    img2_depth_unscale.convertTo(img2_depth, CV_32FC1);

    std::cout << "DEBUG another inspection for depth, before scaling " << img1_depth.at<float>(200, 300) << std::endl;
    img1_depth = img1_depth * DEPTH_SCALE;
    img2_depth = img2_depth * DEPTH_SCALE;
    std::cout << "DEBUG another inspection for depth, after scaling " << img1_depth.at<float>(200, 300) << std::endl;
    
    
    if(img1_cvmat.empty() || img2_cvmat.empty() || img1_depth.empty() || img2_depth.empty())
    {
        std::cout << "Image load failed!" << std::endl;
        return -1;
    }

    // camera intrinsics 
    float fx = 535.4;
    float fy = 539.2;
    float ox = 320.1;
    float oy = 247.6;


    Eigen::Matrix3f intrinsic_mat;
    intrinsic_mat << 
                    fx, 0.f, ox, 
                    0.f, fy, oy, 
                    0.f, 0.f, 1.f;
    
    dvo::Intrinsic cam_intrinsic(intrinsic_mat);
    // cam_intrinsic =  dvo::Intrinsic::Intrinsic(intrinsic_mat);
    
    // build dvo::RgbdCamera class
    dvo::RgbdCamera camera(cam_width, cam_height, cam_intrinsic);
    
    dvo::RgbdImage img1(camera);
    dvo::RgbdImage img2(camera);

    img1.rgb = img1_cvmat;
    img1.depth = img1_depth;

    // DEBUG 
    // cv::Mat dst;
    // cv::normalize(img1.depth, dst, 255, 230, cv::NORM_MINMAX, -1, cv::noArray());
    // cv::imwrite(cwd+"/../src/experiments/norm_depth.png", img1.depth);

    /*
    double min, max;
    cv::minMaxIdx(img1.depth, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(img1.depth, adjMap, 255/max);
    // DEBUG
    std::cout << "DEBUG min, max of depth " << min << " " << max << std::endl;
    cv::imwrite(cwd+"/../src/experiments/norm_depth.png", adjMap);
    */



    std::cout << "DEBUG try inspect depth " << img1.depth.at<double>(300, 200)<< std::endl;
    // std::cout << "DEBUG inspect intensity " << img1_intensity.at<float>(200, 200) << std::endl;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> img1_depth_eigen, img1_intensity_eigen;
    cv2eigen(img1.depth, img1_depth_eigen);
    cv2eigen(img1_intensity, img1_intensity_eigen);
    //std::cout << "DEBUG more thorough inspection on depth" << img1_depth_eigen.col(200) << std::endl;
    // std::cout << "DEBUG more thorough inspection on intensity" << tmp_intensity.col(200) <<std::endl;

    img1.intensity = img1_intensity;

    img2.rgb = img2_cvmat;
    img2.depth = img2_depth;
    img2.intensity = img2_intensity;

    std::cout<< "DEBUG img1 and img2 depth stuff -> img2 " << img2.depth.at<float>(300, 200) << std::endl;
    std::cout<< "DEBUG img1 and img2 depth stuff -> img2 " << img2_depth.at<float>(300, 200) << std::endl;
    

    img1.initialize();
    img2.initialize();

    dvo::PointCloud img1_pc;

    std::cout << "DEBUG before build point cloud" << std::endl;
    camera.buildPointCloud(img1_depth, img1_pc);
    std::cout << "DEBUG build point cloud DONE" << std::endl;
    
    vector<KeyPoint> kps;
    Mat des;
    extract_orb_features(img1_cvmat, kps, des);

    vector<Point2i> kept_kps;
    // discard all surrounding key points
    // keep_points() first argument is a hyper parameter between 0-0.5
    keep_points(0.4, cam_width, cam_height, kps, kept_kps);

    int kept_kps_size = kept_kps.size();
    dvo::PointCloud kept_pts_pc(4, kept_kps_size);
    Eigen::VectorXd img1_kp_intensity(kept_kps_size); 

    feature_point_to_point_cloud(kept_kps, img1_pc, kept_pts_pc, cam_width, cam_height);
    feature_point_to_intensity(kept_kps, img1_intensity, img1_kp_intensity); 

    vector<size_t> non_zero_idx;
    for (size_t i = 0; i < kept_kps.size(); i++)
    {
        Eigen::Vector4f tmp_col = kept_pts_pc.col(i);
        if (tmp_col(0) * tmp_col(1) * tmp_col(2) != 0)
        {
            non_zero_idx.push_back(i);
        }
    }

    int non_zero_size = non_zero_idx.size();
    vector<Point2i> non_zero_kps(non_zero_size);
    dvo::PointCloud non_zero_pc(4, non_zero_size);
    Eigen::VectorXd non_zero_intensity(non_zero_size);

    for (int i = 0; i < non_zero_size; i++) 
    {
        non_zero_kps[i] = kept_kps[non_zero_idx[i]];
        non_zero_pc.col(i) = kept_pts_pc.col(non_zero_idx[i]);
        non_zero_intensity(i) = img1_kp_intensity(non_zero_idx[i]);
    }

    std::cout << "DEBUG intensity prob has problem? " << non_zero_intensity(10) << std::endl;


    std::cout << "num of kept points = " << kept_kps.size() << std::endl; 
    std::cout << "num of original points = " << kps.size() << std::endl; 
    std::cout << "num of non zero points = " << non_zero_size << std::endl; 
    
    

    // solver testing starts here 
    
    FrontendSolver solver;
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    // cout << img1_pc.col(5000).format(CleanFmt) << endl;
    // cout << kept_pts_pc.format(CleanFmt) << endl;
    //cout << img1_kp_intensity.format(CleanFmt) << endl;
    // std::cout << "width: " << img2.width << std::endl;
    // std::cout << "height: " << img2.height << std::endl;
    //solver.solve(kept_pts_pc, img1_kp_intensity, img2);
    //solver.solve(non_zero_pc, non_zero_intensity, img2);
    


    // for plotting verifications
    /*
    Mat im_display = img1_cvmat;
    
    std::vector<cv::Point2i> kps_vec;
    for (int i = 0; i < kps.size(); i++) 
    {
        kps_vec.push_back(kps[i].pt);
    }

    std::cout << "DEBUG num of kps_vec " << kps_vec.size() << std::endl;

    keypoint_plotter(im_display, kps_vec, 'b');
    keypoint_plotter(im_display, kept_kps, 'g');
    
    imwrite(cwd+"/../src/experiments/features.png", im_display);
    std::cout << "Write image completed" <<std::endl;
    */
    return 0;
}



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


void read_intensity(const char* filename, Mat& canvas) 
{
    std::string cwd = get_current_dir_name();
    std::string image_path =  cwd + "/../src/experiments/test_img/" + std::string(filename);
    canvas = cv::imread(image_path, IMREAD_GRAYSCALE);
    return ; 
}

void read_depth(const char* filename, Mat& canvas) 
{
    std::string cwd = get_current_dir_name();
    std::string image_path =  cwd + "/../src/experiments/test_img/" + std::string(filename);
    canvas = cv::imread(image_path, cv::IMREAD_ANYDEPTH);
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
                                char color, 
                                int radius,
                                int thickness
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
    
    for (int i = 0; i < points.size(); i++) 
    {
    cv::circle(img, points[i], radius, plot_color, thickness);
    }
    
    // imwrite(cwd + relative_path, img);
}


void feature_point_to_point_cloud (const std::vector<cv::Point2i>& kept_pts,
                                        const dvo::PointCloud& img_pc,
                                        dvo::PointCloud& kept_pc,
                                        const size_t camera_width, 
                                        const size_t camera_height)
{
    for (int i = 0; i < kept_pts.size(); i++) 
    {
        cv::Point2i tmp_pt = kept_pts[i];
        int tmp_idx = tmp_pt.y * camera_width  + tmp_pt.x;
        kept_pc.col(i) = img_pc.col(tmp_idx);
    }
    return ;
}


void feature_point_to_intensity (const std::vector<cv::Point2i>& kept_pts, 
                                    const cv::Mat& img_intensity,
                                    Eigen::VectorXd& intensity) 
{
    for (int i = 0; i < kept_pts.size(); i++)
    {
        cv::Point2i tmp_pt = kept_pts[i];
        intensity(i) = img_intensity.at<float>(tmp_pt);
    }
    return ;
}


