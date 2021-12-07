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
    cv::Mat gray = cv::imread(rgb_files_[idx_], cv::IMREAD_GRAYSCALE);
    cv::Mat dept = cv::imread(dep_files_[idx_], cv::IMREAD_ANYDEPTH); //TODO: check if this is correct
    res.first.push_back(gray);
    res.first.push_back(dept);
    res.second = timestamps_[idx_];
    idx_++;
    return res;
}

Intrinsic TUMLoader::getIntrinsic() {
    return cam_calib_;
}

}