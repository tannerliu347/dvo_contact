#pragma once

#include <opencv2/opencv.hpp>

namespace dvo {

// static const float Invalid = std::numeric_limits<float>::quiet_NaN();

class Interpolation {
public:    
    static float bilinearWithDepth(const cv::Mat& intensity, const cv::Mat& dept, const float& x, const float& y, const float& z); 
};


}/* namespace dvo */