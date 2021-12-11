#include "TUM_loader.h"

namespace dvo {


TUMLoader::TUMLoader(std::string TUM_dir) : idx_(0), cam_calib_(525.0, 525.0, 319.5, 239.5) {
    // read associated pairs
    std::string assoc_file = TUM_dir + "associated.txt";
    std::ifstream pair_file(assoc_file);
    if (!pair_file.is_open()) {
        std::cout << "Unable to open file: " << assoc_file << std::endl;
    }
    std::string line;
    while (getline(pair_file, line)) {
        std::stringstream ss(line);
        std::string ts, rgb_file, tmp, dep_file;
        getline(ss, ts, ' ');
        getline(ss, rgb_file, ' ');
        getline(ss, tmp, ' ');
        getline(ss, dep_file, ' ');
        // std::cout << ts << std::endl;
        // std::cout << TUM_dir + rgb_file << std::endl;
        // std::cout << TUM_dir + dep_file << std::endl;
        // std::cout << "=====================\n";
        timestamps_.push_back(std::stof(ts));
        rgb_files_.push_back(TUM_dir + rgb_file);
        dep_files_.push_back(TUM_dir + dep_file);
    }
    std::cout << "Successfully found " << timestamps_.size() << " image pairs\n";
    std::cout << "===========================================================\n";
}

bool TUMLoader::hasNext() {
    return idx_ < timestamps_.size();
}

std::pair<std::vector<cv::Mat>, float> TUMLoader::getNext() {
    std::pair<std::vector<cv::Mat>, float> res;
    if (!hasNext()) {
        std::cout << "No available image pairs; Exiting\n";
        return res;
    }
    // read images using the directory
    cv::Mat grey = cv::imread(rgb_files_[idx_], cv::IMREAD_GRAYSCALE);
    cv::Mat dep = cv::imread(dep_files_[idx_], cv::IMREAD_ANYDEPTH);
    // Both gray scale and depth use CV_32FC1
    cv::Mat gray;
    grey.convertTo(gray, CV_32FC1);
    cv::Mat dept = depthRawToM(dep);
    
    res.first.push_back(gray);
    res.first.push_back(dept);
    res.second = timestamps_[idx_];
    idx_++;
    return res;
}

Intrinsic TUMLoader::getIntrinsic() {
    return cam_calib_;
}

cv::Mat TUMLoader::depthRawToM(cv::Mat& depth_raw) {
    float scale = 1.0f / 5000.0f;
    float fNaN = std::numeric_limits<float>::quiet_NaN();
    cv::Mat res(cv::Size(depth_raw.cols, depth_raw.rows), CV_32FC1, cv::Scalar(fNaN));
    // depth_raw.convertTo(res, CV_32FC1, (double)scale, 0.0f);
    for( int y = 0; y < depth_raw.rows; y++ ) {
        for( int x = 0; x < depth_raw.cols; x++ ) {
            // std::cout << "Depth before: " << depth_raw.at<float>(y,x);
            // depth_raw.at<float>(y,x) = scale * depth_raw.at<float>(y,x);
            u_int16_t cur_dep = depth_raw.at<u_int16_t>(y, x);
            if (cur_dep != 0) {
                res.at<float>(y, x) = static_cast<float>(cur_dep) * scale;
            }
            // std::cout << "; Depth after: " << depth_raw.at<float>(y,x) << std::endl;
        }
    }
    return res;
}

}