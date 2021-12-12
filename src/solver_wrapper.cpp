// solver_wrapper.cpp

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
#include "TUM_loader.h"


size_t PYRAMID_LEVEL = 3;

int main ()
{
    std::string cwd = get_current_dir_name();
    std::string TUM_dir = cwd;  // place holder, not sure about what associate.txt is 
    
    dvo::TUMLoader loader(TUM_dir);

    size_t cam_width, cam_height;
    // need cam_width and cam_height from loader

    dvo::Intrinsic cam_intrisic = loader.getIntrinsic(); // did not see cam_calib_ in other parts of the code

    dvo::RgbdCamera camera(cam_width, cam_height, cam_intrinsic)
    dvo::CameraPyramid cam_pyramid(camera);
    
    template <typename T>
    std::vector<T> solved_quat_results;

    while (loader.hasNext())
    {
        
        vector<cv::Mat> img_pair[2];
        img_pair = loader.getImgs();
        
        cv::Mat img_intensity, img_depth;
        img_intensity = img_pair[0];
        img_depth = img_pair[1];

        dvo::AffineTransform R = loader.getPose();

        dvo::ImagePyramid img_pyramid(camera, img_intensity, img_depth);
        
        img_pyramid.build(PYRAMID_LEVEL); 

        dvo::PtSelectionVerify pt_verifier;
        dvo::PtSelection(img_pyramid, pt_verifier);

        

        


    }
    
    
    return 0;
}
