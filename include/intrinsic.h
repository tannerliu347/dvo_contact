#pragma once

#include <eigen3/Eigen/Core>

namespace dvo {


class Intrinsic {
public:
    Intrinsic() {}

    Intrinsic(float fx, float fy, float ox, float oy);

    Intrinsic(const Intrinsic& other);

    float fx() const;
    float fy() const;

    float ox() const;
    float oy() const;

    void invertOffset();
    void scale(float factor);

private:
    Eigen::Matrix3f data_;
};


} //namespace dvo