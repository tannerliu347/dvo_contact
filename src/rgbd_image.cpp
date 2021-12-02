#include "rgbd_image.h"

namespace dvo {

// ------------------------------ RgbdCamera ------------------------------------------
RgbdCamera::RgbdCamera(size_t width, size_t height, const dvo::Intrinsic& intrinsics) :
        width_(width),
        height_(height),
        intrinsics_(intrinsics) {
    pointcloud_template_.resize(Eigen::NoChange, width_ * height_);
    int idx = 0;
    // Project image pixels back to 3D, assuming depth is 1 for now
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++, idx++) {
            pointcloud_template_(0, idx) = (x - intrinsics_.ox()) / intrinsics_.fx();
            pointcloud_template_(1, idx) = (y - intrinsics_.oy()) / intrinsics_.fy();
            pointcloud_template_(2, idx) = 1.0;
            pointcloud_template_(3, idx) = 0.0;
        }
    }
}

size_t RgbdCamera::width() const {
    return width_;
}

size_t RgbdCamera::height() const {
    return height_;
}

const dvo::Intrinsic& RgbdCamera::intrinsics() const {
    return intrinsics_;
}

RgbdImagePtr RgbdCamera::create(const cv::Mat& intensity, const cv::Mat& depth) const {
    RgbdImagePtr result(new RgbdImage(*this)); //TODO: build constructor
    result->intensity = intensity;
    result->depth = depth;
    result->initialize();
    return result;
}

RgbdImagePtr RgbdCamera::create() const {
    return std::make_shared<RgbdImage>(*this);
}

bool RgbdCamera::hasSamesize(const cv::Mat& img) const {
    return img.cols == width_ && img.rows == height_;
}

// TODO:missing build point cloud
// ------------------------------ RgbdCamera ------------------------------------------


// ----------------------------- CameraPyramid ----------------------------------------

CameraPyramid::CameraPyramid(const RgbdCamera& base) {
    levels_.push_back(std::make_shared<RgbdCamera>(base));
}

CameraPyramid::CameraPyramid(size_t base_width, size_t base_height, const dvo::Intrinsic& base_intrinsics) {
    levels_.push_back(std::make_shared<RgbdCamera>(base_width, base_height, base_intrinsics));
}

CameraPyramid::~CameraPyramid() {}

ImagePyramidPtr CameraPyramid::create(const cv::Mat& base_intensity, const cv::Mat& base_depth) {
    return std::make_shared<ImagePyramid>(*this, base_intensity, base_depth);
}

void CameraPyramid::build(size_t levels) {
    size_t start = levels_.size();
    for (size_t i = start; i < levels; i++) {
        RgbdCameraPtr& prev = levels_[i - 1];
        dvo::Intrinsic intrinsics(prev->intrinsics());
        intrinsics.scale(0.5f);
        levels_.push_back(std::make_shared<RgbdCamera>(prev->width() / 2, prev->height() / 2, intrinsics));
    }
}

const RgbdCamera& CameraPyramid::level(size_t level) {
    build(level + 1);
    return *levels_[level];
}

// TODO: verify which one gets called when
const RgbdCamera& CameraPyramid::level(size_t level) const {
    return *levels_[level];
}

// ----------------------------- CameraPyramid ----------------------------------------


// ----------------------------- ImagePyramid ----------------------------------------

ImagePyramid::ImagePyramid(CameraPyramid& camera, const cv::Mat& intensity, const cv::Mat& depth) :
        camPyr_(camera) {
    levels_.push_back(camPyr_.level(0).create(intensity, depth));
}

ImagePyramid::~ImagePyramid() {}

void ImagePyramid::build(const size_t num_levels) {
    if (levels_.size() >= num_levels) return;
    size_t start = levels_.size();
    for (size_t i = start; i < num_levels; i++) {
        levels_.push_back(camPyr_.level(i).create());
        //TODO: add smooth and subsample for intensity and depth
        levels_[i]->initialize();
    }
}

RgbdImage& ImagePyramid::level(size_t idx) {
    assert(idx < levels_.size());
    return *levels_[idx];
}

double ImagePyramid::timestamp() const {
    return !levels_.empty() ? levels_[0]->timestamp : 0.0;
}

// static smooth and subsample
static void pyrMeanDownsample(const cv::Mat& in, cv::Mat& out) {
    out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());
    for (int i = 0; i < out.rows; i++) {
        for (int j = 0; j < out.cols; j++) {
            int j0 = j * 2;
            int j1 = j0 + 1;
            int i0 = i * 2;
            int i1 = i0 + 1;
            out.at<float>(i, j) = (in.at<float>(i0, j0) + in.at<float>(i0, j1) + in.at<float>(i1, j0) + in.at<float>(i1, j1)) / 4.0f;
        }
    }
}

static void pyrDownsample(const cv::Mat& in, cv::Mat& out) {
    out.create(cv::Size(in.size().width / 2, in.size().height / 2), in.type());
    for (int i = 0; i < out.rows; i++) {
        for (int j = 0; j < out.cols; j++) {
            out.at<float>(i, j) = in.at<float>(i * 2, j * 2);
        }
    }
}

// ----------------------------- ImagePyramid ----------------------------------------


} //namespace dvo