#pragma once
#include "intrinsic.h"
#include "rgbd_image.h"
#include <eigen3/Eigen/Core>

class FrontendSolver {
private:
    dvo::Intrinsic intrinsic_;
public:
    FrontendSolver(const dvo::Intrinsic & intrinsic);
    Eigen::Matrix4d solve (const dvo::RgbdImage& img1, const dvo::RgbdImage& img2, const double initial_guess[7]);
};
