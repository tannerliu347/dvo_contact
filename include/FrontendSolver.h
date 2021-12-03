#pragma once
#include "intrinsic.h"
#include "rgbd_image.h"
#include <eigen3/Eigen/Core>

class FrontendSolver {
private:
    dvo::Intrinsic intrinsic_;
public:
    FrontendSolver(const dvo::Intrinsic & intrinsic);
    Eigen::Matrix4d solve (const RGBDImage& img1, const RGBDImage& img2, Eigen::Matrix initial_guess=identity());
};
