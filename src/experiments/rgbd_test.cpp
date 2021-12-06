#include "rgbd_image.h"
#include "TUM_loader.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main() {
    dvo::TUMLoader tum_loader("/home/tannerliu/dvo_contact/dataset/rgbd_dataset_freiburg1_xyz/");
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