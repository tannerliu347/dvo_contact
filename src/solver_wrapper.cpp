// solver_wrapper.cpp
#define DEBUG 

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
#include <assert.h>

#include "rgbd_image.h"
#include "FrontendSolver.h"
#include "utils.hpp"
#include "TUM_loader.h"
#include "point_selection.h"

size_t PYRAMID_LEVEL = 3;
size_t cam_width = 640;
size_t cam_height = 480;

int main (int argc, char* argv[])
{
    std::string dataset_dir(argv[1]);
    std::string cwd = get_current_dir_name();
<<<<<<< HEAD
    std::string TUM_dir = cwd + '/' + dataset_dir;  // place holder, not sure about what associated.txt is 
    
=======
    std::string TUM_dir = cwd + '/' + dataset_dir;  

    std::cout << "DEBUG current dataset directory = " << cwd+dataset_dir << std::endl;

>>>>>>> solver_wrapper
    dvo::TUMLoader loader(TUM_dir);
    std::cout << "DEBUG loader initialized SUCCESSFUL" << std::endl;

    char resolved_path[PATH_MAX];
    char* tmp = realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");
    std::cout << "DEBUG loading yaml config SUCCESSFUL" << std::endl;


    // size_t cam_width, cam_height;
    // need cam_width and cam_height from loader

    dvo::Intrinsic cam_intrinsic = loader.getIntrinsic(); // did not see cam_calib_ in other parts of the code
    dvo::RgbdCamera camera(cam_width, cam_height, cam_intrinsic);

    std::cout << "DEBUG RgbdCamera object built SUCCESSFUL" << std::endl;
    
    // template <typename T>
    // std::vector<T> solved_quat_results;
    dvo::CameraPyramid cam_pyramid(camera);
    dvo::CameraPyramid* cam_pointer = &cam_pyramid; // non-elegant fix for const reference


    cv::Mat previous_intensity, current_intensity;
    cv::Mat previous_depth, current_depth;
    
    std::vector<cv::Mat> previous_img(2);
    std::vector<cv::Mat> current_img(2);

    previous_img = loader.getImgs();

    if (previous_img.size() == 0)
    {
        std::cout << "\033[1;31mRuntimeERROR: no image was found by TUM_loader, program exiting\033[0m" << std::endl;
        assert(false);
    }

    previous_intensity = previous_img[0];
    previous_depth = previous_img[1];

    std::cout << "DEBUG previous image read SUCCESSFUL" << std::endl;

    dvo::PtAndGradVerify pt_verifier;
    pt_verifier.intensity_threshold = config_setting["dvo"]["intensity_threshold"].as<float>();
    pt_verifier.depth_threshold = config_setting["dvo"]["depth_threshold"].as<float>();
    
    std::cout << "DEBUG all essential parameters build SUCCESSFUL" << std::endl;
<<<<<<< HEAD

    FrontendSolver solver;
    solver.vis_ = true;
=======
    int temp_counter = 0;
>>>>>>> solver_wrapper
    while (loader.hasNext())
    {
        
        loader.step();

        // std::cout << "DEBUG loader.step() complete SUCCESSFUL" << std::endl;
        // std::cout << "DEBUG timestamp of the current image" << loader.getTimestamp() << std::endl;

        current_img = loader.getImgs();
        current_intensity = current_img[0];
        current_depth = current_img[1];

        if (temp_counter % 10 == 0)
        {
            std::cout << "DEBUG in while loop, current iteration = " << temp_counter << ", " 
            << "\nloading image " << std::to_string(loader.getTimestamp()) <<std::endl;
            cv::imwrite(cwd+"/../src/experiments/run_img/read_img"+std::to_string(temp_counter)+".png", current_intensity);
        }


        dvo::RgbdImage current_rgbd(camera);
        current_rgbd.intensity = current_intensity;
        current_rgbd.depth = current_depth;
        current_rgbd.initialize();

        // solver(PointCloud pc, Eigen::VectorXd intensity, RgbdImage im2)
        
        dvo::AffineTransform R = loader.getPose(); // not sure for what // I believe it is the ground truth transform?
        dvo::ImagePyramid previous_pyramid((*cam_pointer), previous_intensity, previous_depth);
        // img_pyramid.build(PYRAMID_LEVEL); // no need 
        
        dvo::PtSelection point_selecter(previous_pyramid, pt_verifier);

        for (int op_level = PYRAMID_LEVEL-1; op_level >= 0 ; op_level--)
        {
            dvo::PtSelection::PtIterator start_pointer, end_pointer;

            int sp_size = end_pointer-start_pointer;
            dvo::PointCloud pc(4, sp_size);
            Eigen::VectorXd intensity(sp_size);

            size_t idx = 0;
            for (auto it = start_pointer; it != end_pointer; it++)
            {
                float x, y, z;
                x = it->x;  y = it->y; z = it->z;
                Eigen::Vector4f tmp_col;
                tmp_col << x, y, z, 1.f;
                pc.col(idx) = tmp_col;
                intensity(idx) = it->i;
                idx++;
            }
            double identity[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            solver.solve(pc, 
                         intensity, 
                         point_selecter.getImagePyramid().level(op_level),
                         identity); //place holder, this is the img2 parameter prepared for solver
            
            /*
            reserved space for iteratively calling the solver;
            using identity as initial guess for pyramid_level = 2;
<<<<<<< HEAD
            using result from pyramid_level = 2 for 
            */
            
=======
            using result from pyramid_level = 2 for pyramid_level = 1;
            using result from pyramid_level = 1 for original image i. e. pyramid_level = 0;
>>>>>>> solver_wrapper

            input parameters pc, intensity, img2 = (point_selecter.getImagePyramid().level(op_level));
            */
        
            temp_counter++;
        }
        // solver result recording
        // place holder here


        // house keeping for the next iteration of image pairs
        previous_intensity = current_intensity;
        previous_depth = current_depth;

    }
    
    // output all recorded solver results
    // place holder here
    
    return 0;
}
