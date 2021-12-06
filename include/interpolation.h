#pragma once
#include "rgbd_image.h"

#include <opencv2/opencv.hpp>


namespace dvo {

class Interpolation {
public:    
    static float bilinearWithDepth(const cv::Mat& intensity, const cv::Mat& dept, const float& x, const float& y, const float& z); 
};


}/* namespace dvo */