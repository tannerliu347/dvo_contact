#include "interpolation.h"

namespace dvo
{

float Interpolation::bilinearWithDepth(const cv::Mat& intensity, const cv::Mat& depth, const float& x, const float& y, const float& z) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;

    if(x1 >= intensity.cols || y1 >= intensity.rows) return Invalid;
    
    const float x1_weight = x - x0;
    const float x0_weight = 1.0f - x1_weight;
    const float y1_weight = y - y0;
    const float y0_weight = 1.0f - y1_weight;
    const float z_eps = z - 0.5f;

    float val = 0.0f;
    float sum = 0.0f;

    if(std::isfinite(depth.at<float>(y0, x0)) && depth.at<float>(y0, x0) > z_eps) {
        val += x0_weight * y0_weight * intensity.at<float>(y0, x0);
        sum += x0_weight * y0_weight;
    }

    if(std::isfinite(depth.at<float>(y0, x1)) && depth.at<float>(y0, x1) > z_eps) {
        val += x1_weight * y0_weight * intensity.at<float>(y0, x1);
        sum += x1_weight * y0_weight;
    }

    if(std::isfinite(depth.at<float>(y1, x0)) && depth.at<float>(y1, x0) > z_eps) {
        val += x0_weight * y1_weight * intensity.at<float>(y1, x0);
        sum += x0_weight * y1_weight;
    }

    if(std::isfinite(depth.at<float>(y1, x1)) && depth.at<float>(y1, x1) > z_eps) {
        val += x1_weight * y1_weight * intensity.at<float>(y1, x1);
        sum += x1_weight * y1_weight;
    }

    if(sum > 0.0f) {
        val /= sum;
    } else {
        //val = Invalid;
        val = -1.0f;
        std::cout << "invalid depth after warping!\n";
        //std::cout << depth.at<float>(y0, x0)<<std::endl;
        //std::cout << depth.at<float>(y0, x0) - z_eps <<std::endl;
    }
    /*
    std::cout << "depth at x0 y0 " << depth.at<float>(y0, x0) << std::endl;
    std::cout << "DEBUG x0, y0 " << intensity.at<float>(y0, x0) << std::endl;
    std::cout << "DEBUG sum" << sum << std::endl;
    std::cout << "DEBUG val (should be close)" << val << std::endl;
    */
    return val;
}

} // namespace dvo
