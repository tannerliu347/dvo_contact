// solver_wrapper.cpp

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include "yaml-cpp/yaml.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>


#include <iostream>
#include <vector>
#include <experimental/filesystem>
#include <string>
// #include <unistd.h>

#include "rgbd_image.h"
#include "FrontendSolver.h"
#include "utils.hpp"
#include "TUM_loader.h"
#include "point_selection.h"

size_t PYRAMID_LEVEL = 3;
size_t cam_width = 640;
size_t cam_height = 480;

int main ()
{
    std::string cwd = get_current_dir_name();
    std::string TUM_dir = cwd;  // place holder, not sure about what associated.txt is 
    
    dvo::TUMLoader loader(TUM_dir);

    char resolved_path[PATH_MAX];
    char* tmp = realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");

    // size_t cam_width, cam_height;
    // need cam_width and cam_height from loader

    dvo::Intrinsic cam_intrinsic = loader.getIntrinsic(); // did not see cam_calib_ in other parts of the code

    dvo::RgbdCamera camera(cam_width, cam_height, cam_intrinsic);
    dvo::CameraPyramid cam_pyramid(camera);
    
    // template <typename T>
    // std::vector<T> solved_quat_results;

    cv::Mat previous_intensity, current_intensity;
    cv::Mat previous_depth, current_depth;
    
    std::vector<cv::Mat> previous_img(2);
    std::vector<cv::Mat> current_img(2);

    previous_img = loader.getImgs();
    previous_intensity = previous_img[0];
    previous_depth = previous_img[1];

    dvo::PtAndGradVerify pt_verifier;
    pt_verifier.intensity_threshold = config_setting["dvo"]["intensity_threshold"].as<float>();
    pt_verifier.depth_threshold = config_setting["dvo"]["depth_threshold"].as<float>();

    while (loader.hasNext())
    {
        loader.step();
        current_img = loader.getImgs();
        current_intensity = current_img[0];
        current_depth = current_img[1];

        dvo::RgbdImage current_rgbd(camera);
        current_rgbd.intensity = current_intensity;
        current_rgbd.depth = current_depth;
        current_rgbd.initialize();

        // solver(PointCloud pc, Eigen::VectorXd intensity, RgbdImage im2)
        
        dvo::AffineTransform R = loader.getPose(); // not sure for what 
        dvo::ImagePyramid previous_pyramid(camera, previous_intensity, previous_depth);
        // img_pyramid.build(PYRAMID_LEVEL); // no need
        
        dvo::PtSelection point_selecter(previous_pyramid, pt_verifier);

        for (size_t op_level = PYRAMID_LEVEL-1; op_level >= 0 ; op_level--)
        {
            dvo::PtSelection::PtIterator start_pointer, end_pointer;
            point_selecter.select(op_level, start_pointer, end_pointer);

            dvo::PointCloud pc;
            Eigen::VectorXd intensity;

            size_t idx = 0;
            for (auto it = start_pointer; it != end_pointer; it++)
            {
                float x, y, z;
                x = it->x;  y = it->y; z = it->z;
                Eigen::Vector4f tmp_col {{x, y, z, 1.f}};
                pc.col(idx) = tmp_col;
                intensity(idx) = it->i;
                idx++;
            }

            (point_selecter.getImagePyramid()->level(op_level)); //place holder, this is the img2 parameter prepared for solver


        }

        

        


    }
    
    
    return 0;
}
