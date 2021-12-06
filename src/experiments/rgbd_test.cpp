#include "rgbd_image.h"
#include "TUM_loader.h"
#include "yaml-cpp/yaml.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main() {
    char resolved_path[PATH_MAX];
    realpath("../", resolved_path);
    std::cout << resolved_path << std::endl;
    YAML::Node config_setting = YAML::LoadFile(std::string(resolved_path) + "/config/config.yaml");
    
    std::string image_load_path = config_setting["dvo"]["image_load_path"] ? config_setting["dvo"]["image_load_path"].as<std::string>() 
                                                                : "/home/tingjun/code/dvo_contact/dataset/";

    dvo::TUMLoader tum_loader(image_load_path);
    // auto cur = tum_loader.getNext();
    while (tum_loader.hasNext()) {
        auto cur = tum_loader.getNext();
        cv::imshow("gray", cur.first[0]);
        cv::imshow("dept", cur.first[1]);
        std::cout << cur.second << std::endl;
        cv::waitKey(10);
    }
    return 0;
}