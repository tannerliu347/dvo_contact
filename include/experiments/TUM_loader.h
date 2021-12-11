#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "intrinsic.h"


namespace dvo {

    
class TUMLoader {
public:
    TUMLoader(std::string TUM_dir);

    bool hasNext();

    /**
     * @brief return the next pair of intensity and depth images,
     * first is a vector of 0: grayscale img, 1: depth image; second is the timestamp
     */
    std::pair<std::vector<cv::Mat>, float> getNext();

    Intrinsic getIntrinsic();

private:
    /**
     * @brief converts depth to meters
     */
    cv::Mat depthRawToM(cv::Mat& depth_raw);

private:
    std::vector<float> timestamps_;
    std::vector<std::string> rgb_files_;
    std::vector<std::string> dep_files_;
    Intrinsic cam_calib_;
    size_t idx_;
};

}