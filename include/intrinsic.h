#pragma once

#include <eigen3/Eigen/Core>

namespace dvo {


class Intrinsic {
public:
    Intrinsic(); 
    Intrinsic(Eigen::Matrix3f cam_data);
    Intrinsic(const Intrinsic& other);

    float fx() const;
    float fy() const;

    float ox() const;
    float oy() const;

    Eigen::Matrix3f get_intrinsic_matrix() const;

    void invertOffset();
    void scale(float factor);

private:
    Eigen::Matrix3f data_;
};


} //namespace dvo