#include "interpolation.h"
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

void RgbdCamera::buildPointCloud(const cv::Mat &depth, PointCloud& pointcloud) const{
    
    assert(hasSamesize(depth));
    pointcloud.resize(Eigen::NoChange, width_ * height_);
    const float* depth_ptr = depth.ptr<float>();
    for (int index = 0; index < width_ * height_; index++, depth_ptr++) {
        pointcloud.col(index) = pointcloud_template_.col(index) * (*depth_ptr);
        pointcloud(3, index) = 1.0;
    }

}
// ------------------------------ RgbdCamera ------------------------------------------


// ----------------------------- CameraPyramid ----------------------------------------

CameraPyramid::CameraPyramid(const RgbdCamera& base) {
    levels_.push_back(std::make_shared<RgbdCamera>(base));
}

CameraPyramid::CameraPyramid(size_t base_width, size_t base_height, const dvo::Intrinsic& base_intrinsics) {
    levels_.push_back(std::make_shared<RgbdCamera>(base_width, base_height, base_intrinsics));
}

CameraPyramid::~CameraPyramid() {
    std::cout << "Destruct CameraPyramid" << std::endl;
}

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

ImagePyramid::~ImagePyramid() {
    std::cout << "Destruct Image Pyramid" << std::endl;
}

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
    return *levels_[idx]; // return image for current level
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


// ----------------------------- RgbdImage ----------------------------------------
RgbdImage::RgbdImage(const RgbdCamera& camera) :
    camera_(camera),
    intensity_requires_calculation_(true),
    depth_requires_calculation_(true),
    pointcloud_requires_build_(true),
    acceleration_requires_calculation_(true),
    width(0),
    height(0) {}

RgbdImage::~RgbdImage() {
    std::cout << "Destruct RgbdImage" << std::endl;
}

const RgbdCamera& RgbdImage::camera() const
{
  return camera_;
}

void RgbdImage::initialize()
{
    bool intensity_bool = hasIntensity();
    bool depth_bool = hasDepth();

    std::cout << "intensity_bool: " << intensity_bool << std::endl;
    std::cout << "depth_bool: " << depth_bool << std::endl;


    assert(hasIntensity() || hasDepth());

    intensity_requires_calculation_ = true;
    depth_requires_calculation_ = true;
    pointcloud_requires_build_ = true;

    if(hasIntensity() && hasDepth()) {
        assert(intensity.size() == depth.size());
        width = intensity.cols;
        height = intensity.rows;
        return;
    }

    if(hasIntensity()) {
        assert(intensity.type() == cv::DataType<float>::type && intensity.channels() == 1);
        width = intensity.cols;
        height = intensity.rows;
        return;
    }

    if(hasDepth()) {
        assert(depth.type() == cv::DataType<float>::type && depth.channels() == 1);
        width = depth.cols;
        height = depth.cols;
    }
    return;
}

bool RgbdImage::hasIntensity() const
{
  return !intensity.empty();
}

bool RgbdImage::hasRgb() const
{
  return !rgb.empty();
}

bool RgbdImage::hasDepth() const
{
  return !depth.empty();
}

void RgbdImage::calculateDerivativeX(const cv::Mat& img, cv::Mat& result) {
    result.create(img.size(), img.type());

    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            int prev = std::max(j - 1, 0);
            int next = std::min(j + 1, img.cols - 1);
            // TODO: why times 0.5 here?
            result.at<float>(i, j) = (float) (img.at<float>(i, next) - img.at<float>(i, prev)) * 0.5f;
        }
    }
}

void RgbdImage::calculateDerivativeY(const cv::Mat& img, cv::Mat& result) {
    result.create(img.size(), img.type());

    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            int prev = std::max(i - 1, 0);
            int next = std::min(i + 1, img.rows - 1);
            // TODO: why times 0.5 here?
            result.at<float>(i, j) = (float) (img.at<float>(next, j) - img.at<float>(prev, j)) * 0.5f;
        }
    }
}

void RgbdImage::calculateIntensityDerivatives() {
    if(!intensity_requires_calculation_) return;
    assert(hasIntensity());
    calculateDerivativeX(intensity, intensity_dx);
    calculateDerivativeY(intensity, intensity_dy);
    
    intensity_requires_calculation_ = false;
}

void RgbdImage::calculateDepthDerivatives() {
    if(!depth_requires_calculation_) return;
    assert(hasDepth());
    calculateDerivativeX(depth, depth_dx);
    calculateDerivativeY(depth, depth_dy);
    
    depth_requires_calculation_ = false;
}

void RgbdImage::calculateDerivatives() {
    calculateIntensityDerivatives();
    calculateDepthDerivatives();
}

// TODO: please check again to make sure this function is written correctly to combine RgbdCamera::buildPointCould() and RgbdImage::buildPointCould()
void RgbdImage::buildPointCloud() {
    if(!pointcloud_requires_build_) return;

    assert(hasDepth());

    camera_.buildPointCloud(depth, point_cloud);

    pointcloud_requires_build_ = false;
}

void RgbdImage::calculateNormals() {
    if(angles.total() == 0) {
        normals = cv::Mat::zeros(depth.size(), CV_32FC4); // CV_32FC4: 32-bit float matrix with 4 channels
        angles = cv::Mat::zeros(depth.size(), CV_32FC1);
        
        float *angle_ptr = angles.ptr<float>();
        cv::Vec4f *normal_ptr = normals.ptr<cv::Vec4f>();

        int i_max = depth.rows - 1;
        int j_max = depth.cols - 1;

        for(int i = 0; i < depth.rows; i++) {
            for(int j = 0; j < depth.cols; j++) {
                int idx1 = i * depth.cols + std::max(j-1, 0);
                int idx2 = i * depth.cols + std::min(j+1, j_max);
                int idx3 = std::max(i-1, 0) * depth.cols + j;
                int idx4 = std::min(i+1, i_max) * depth.cols + j;

                // Not using AligedMapType
                Eigen::Vector4f n;
                // cross3 returns the cross product of *this and other using only the x, y, and z coefficients
                // This function is especially useful when using 4D vectors instead of 3D ones 
                // each col of point_cloud has 4 elements
                n = point_cloud.col(idx2) - point_cloud.col(idx1).cross3(point_cloud.col(idx4)) - point_cloud.col(idx3);
                n.normalize();

                *angle_ptr = std::abs(n(2));
            }
        }
    }
}

void RgbdImage::buildAccelerationStructure() {
    if (acceleration_requires_calculation_) {
        calculateDerivatives();
        acceleration_requires_calculation_ = false;
    }

    // if(acceleration.total() == 0) {
    //     calculateDerivatives();
    //     cv::Mat zeros = cv::Mat::zeros(intensity.size(), CV_32FC1);
    //     ///TODO: Convert to a certain type, check again(cv::merge can only merge data with same type)
    //     /// TOIGNORE: change it to vector:
        
    //     cv::Mat intensity_converted;
    //     intensity.convertTo(intensity_converted, CV_32FC1);
    //     cv::Mat intensity_dx_converted;
    //     intensity_dx.convertTo(intensity_dx_converted, CV_32FC1);
    //     cv::Mat intensity_dy_converted;
    //     intensity_dy.convertTo(intensity_dy_converted, CV_32FC1);
    //     cv::Mat depth_converted;
    //     depth.convertTo(depth_converted, CV_32FC1);
    //     cv::Mat depth_dx_converted;
    //     depth_dx.convertTo(depth_dx_converted, CV_32FC1);
    //     cv::Mat depth_dy_converted;
    //     depth_dy.convertTo(depth_dy_converted, CV_32FC1);
    //     cv::Mat channels[8] = {intensity_converted, depth, intensity_dx_converted, intensity_dy_converted, depth_dx, depth_dy, zeros, zeros};
    //     cv::merge(channels, 8, acceleration);
        
    //     // cv::Mat channels[2] = {intensity_converted, depth};
    //     // cv::merge(channels, 2, acceleration); 
    // }
}

bool RgbdImage::inImage(const float& x, const float& y) const
{
  return x >= 0 && x < width && y >= 0 && y < height;
}

void RgbdImage::warpIntensity(const AffineTransform& transformation, const PointCloud& reference_pointcloud, 
                              const Intrinsic& intrinsics, RgbdImage& result, PointCloud& transformed_pointcloud){
    
    // the following line is comment out because AffineTransform is already define as 3 floats 
    // Eigen::Affine3f new_transformation = transformation.cast<float>();

    cv::Mat warped_image(intensity.size(), intensity.type());
    cv::Mat warped_depth(depth.size(), depth.type());

    float ox = intrinsics.ox();
    float oy = intrinsics.oy();

    float* warped_intensity_ptr = warped_image.ptr<float>();
    float* warped_depth_ptr = warped_depth.ptr<float>();  

    // int outliers = 0;
    // int total = 0;
    int idx = 0;

    transformed_pointcloud = transformation * reference_pointcloud;
    
    for(size_t i = 0; i < height; i++) {
        for(size_t j = 0; j < width; ++j, ++idx, ++warped_intensity_ptr, ++warped_depth_ptr) {
            const Eigen::Vector4f& p3d = transformed_pointcloud.col(idx);

            if(!std::isfinite(p3d(2))) {
                *warped_intensity_ptr = Invalid;
                *warped_depth_ptr = Invalid;
                continue;
            }

            float x_projected = static_cast<float>(p3d(0) * intrinsics.fx() / p3d(2) + ox);
            float y_projected = static_cast<float>(p3d(1) * intrinsics.fy() / p3d(2) + oy);
            if(inImage(x_projected, y_projected)) {
                float z = (float) p3d(2);

                *warped_intensity_ptr = Interpolation::bilinearWithDepth(this->intensity, this->depth, x_projected, y_projected, z);
                *warped_depth_ptr = z;
            } else {
                *warped_intensity_ptr = Invalid;
                *warped_depth_ptr = Invalid;
            }
        }
    }

    result.intensity = warped_image;
    result.depth = warped_depth;
    result.initialize();
}

void RgbdImage::warpDepthForward(const AffineTransform& transformation, const Intrinsic& intrinsics, RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud) {
    Eigen::Affine3d transformation_3d = transformation.cast<double>();

    bool identity = transformation.affine().isIdentity(1e-6);
    cloud = cv::Mat_<cv::Vec3d>(depth.size(), cv::Vec3d(0, 0, 0));
    cv::Mat warped_depth = cv::Mat::zeros(depth.size(), depth.type());
    warped_depth.setTo(Invalid);

    float ox = intrinsics.ox();
    float oy = intrinsics.oy();
    float fx = intrinsics.fx();
    float fy = intrinsics.fy();
    const float* depth_ptr = depth.ptr<float>();
    // int outliers = 0;
    // int total = 0;

    for (size_t i = 0; i < height; i++)
    {
        for (size_t j = 0; j < width; j++, depth_ptr++)
        {
            if(!std::isfinite(*depth_ptr)) {
                continue;
            }

            float depth = *depth_ptr;
            Eigen::Vector3d p3d((j - ox) * depth / fx, (i - oy) * depth / fy, depth);
            Eigen::Vector3d p3d_transformed = transformation_3d * p3d;

            float i_projected = static_cast<float>(p3d_transformed(0) * fx / p3d_transformed(2) + ox);
            float j_projected = static_cast<float>(p3d_transformed(1) * fy / p3d_transformed(2) + oy);

            if(inImage(i_projected, j_projected)) {
                int idx_i = static_cast<int>(i_projected), idx_j = static_cast<int>(j_projected);

                if(!std::isfinite(warped_depth.at<float>(idx_i, idx_j)) || (warped_depth.at<float>(idx_i, idx_j) - 0.05) > depth)
                warped_depth.at<float>(idx_i, idx_j) = depth;
            }
            cloud(i, j) = cv::Vec3d(p3d_transformed(0), p3d_transformed(1), p3d_transformed(2));
        }
    }
}

void RgbdImage::warpIntensityForward(const AffineTransform& transformation, const Intrinsic& intrinsics, RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud)
{
    Eigen::Affine3d transformation_3d = transformation.cast<double>();
    bool identity = transformation.affine().isIdentity(1e-6);
    cloud = cv::Mat_<cv::Vec3d>(intensity.size(), cv::Vec3d(0, 0, 0));
    cv::Mat warped_image = cv::Mat::zeros(intensity.size(), intensity.type());

    if(identity) {
        warped_image = intensity;
        result.intensity = warped_image;
        result.depth = depth;
        result.initialize();
        return;
    }

    float ox = intrinsics.ox();
    float oy = intrinsics.oy();
    float fx = intrinsics.fx();
    float fy = intrinsics.fy();

    const float* depth_ptr = depth.ptr<float>();

    for(size_t i = 0; i < height; i++) {
        for(size_t j = 0; j < width; j++, depth_ptr++) {
            if (*depth_ptr <= 1e-6f) continue;
            
            float depth = *depth_ptr;
            Eigen::Vector3d p3d((j - ox) * depth / fx, (i - oy) * depth / fy, depth);
            Eigen::Vector3d p3d_transformed = transformation_3d * p3d;

            float i_projected = static_cast<float>(p3d_transformed(0) * fx / p3d_transformed(2) + ox);
            float j_projected = static_cast<float>(p3d_transformed(1) * fy / p3d_transformed(2) + oy);
            if(inImage(i_projected, j_projected)) {
                int idx_i = static_cast<int>(i_projected), idx_j = static_cast<int>(j_projected);

                warped_image.at<float>(idx_i, idx_j) = intensity.at<float>(i, j);
            }
            cloud(i, j) = cv::Vec3d(p3d_transformed(0), p3d_transformed(1), p3d_transformed(2));
        }
    }
    result.intensity = warped_image;
    result.depth = depth;
    result.initialize();
}
// ----------------------------- RgbdImage ----------------------------------------

} //namespace dvo