#pragma once
#include "rgbd_image.h"
#include <eigen3/Eigen/Core>

class FrontendSolver {
public:
    Eigen::Matrix4d solve (const dvo::RgbdImage& img1, const dvo::RgbdImage& img2);
    void solve (const dvo::PointCloud& pc1, const Eigen::VectorXd& img1_intensity, const dvo::RgbdImage& img2);
};
