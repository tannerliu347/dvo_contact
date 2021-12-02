#include "intrinsic.h"

namespace dvo {


Intrinsic::Intrinsic(const Intrinsic& other) : data_(other.data_) {}

float Intrinsic::fx() const {
    return data_(0, 0);
}

float Intrinsic::fy() const {
    return data_(1, 1);
}

float Intrinsic::ox() const {
    return data_(0, 2);
}

float Intrinsic::oy() const {
    return data_(1, 2);
}

void Intrinsic::invertOffset() {
    data_(0, 2) *= -1;
    data_(1, 2) *= -1;
}

void Intrinsic::scale(float factor) {
    data_ *= factor;
}


} //namespace dvo