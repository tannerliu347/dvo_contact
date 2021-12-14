#pragma once

#include <opencv2/opencv.hpp>

#include <memory>
#include <eigen3/Eigen/Geometry>
// #include <boost/smart_ptr.hpp>
#include "intrinsic.h"

namespace dvo {

typedef Eigen::Matrix<float, 8, 1> Vector8f;
typedef Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor> PointCloud;
typedef Eigen::Transform<float,3, Eigen::Affine> AffineTransform;
static const float Invalid = std::numeric_limits<float>::quiet_NaN();


struct PtIntensityDepth{
public:
    // PtIntensityDepth();
    // ~PtIntensityDepth(){};
    Eigen::Vector4f getPointVec() {
        Eigen::Vector4f res;
        res << x, y, z, 0.0f;
        return res;
    }

    Eigen::Vector2f getIntensityDepty() {
        Eigen::Vector2f res;
        res << i, d;
        return res;
    }

    Eigen::Vector2f getIntensityDeriv() {
        Eigen::Vector2f res;
        res << idx, idy;
        return res;
    }

    Eigen::Vector2f getDepthDeriv() {
        Eigen::Vector2f res;
        res << ddx, ddy;
        return res;
    }

    Vector8f getIntensityDepthAndDeriv() {
        Vector8f res;
        res << i, d, idx, idy, ddx, ddy, time_interp, 0.0f;
        return res;
    }

    float x, y, z;
    float i, d, idx, idy, ddx, ddy, time_interp;
};

class RgbdImage;
typedef std::shared_ptr<RgbdImage> RgbdImagePtr;

class ImagePyramid;
typedef std::shared_ptr<ImagePyramid> ImagePyramidPtr;

class RgbdCamera {
public:
    RgbdCamera(size_t width, size_t height, const dvo::Intrinsic& intrinsics);
    ~RgbdCamera() {
        // std::cout << "Destruct Rgbd Camera Object" << std::endl;
    }

    size_t width() const;
    size_t height() const;

    const dvo::Intrinsic& intrinsics() const;

    RgbdImagePtr create(const cv::Mat& intensity, const cv::Mat& depth) const;
    RgbdImagePtr create() const;
    void buildPointCloud(const cv::Mat &depth, PointCloud& pointcloud) const;

private:
    bool hasSamesize(const cv::Mat& img) const;

private:
    size_t width_, height_;
    dvo::Intrinsic intrinsics_;
    PointCloud pointcloud_template_;
};

typedef std::shared_ptr<RgbdCamera> RgbdCameraPtr;
typedef std::shared_ptr<const RgbdCamera> RgbdCameraConstPtr;

class CameraPyramid {
public:
    CameraPyramid(const RgbdCamera& base);
    CameraPyramid(size_t base_width, size_t base_height, const dvo::Intrinsic& base_intrinsics);
    ~CameraPyramid();

    ImagePyramidPtr create(const cv::Mat& base_intensity, const cv::Mat& base_depth);

    /**
     * @brief check current Pyramid levels_ and build upto provided levels;
     */
    void build(size_t levels);

    const RgbdCamera& level(size_t level);

    const RgbdCamera& level(size_t level) const;

private:
    std::vector<RgbdCameraPtr> levels_;
};

class ImagePyramid {
public:
    ImagePyramid(CameraPyramid& camera, const cv::Mat& intensity, const cv::Mat& depth);

    virtual ~ImagePyramid();

    void build(const size_t num_levels);

    RgbdImage& level(size_t idx);

    double timestamp() const;

private:
    CameraPyramid& camPyr_;
    std::vector<RgbdImagePtr> levels_;
};

class RgbdImage {
public:
    RgbdImage(const RgbdCamera& camera);
    virtual ~RgbdImage();

    const RgbdCamera& camera() const;

    cv::Mat intensity;
    cv::Mat intensity_dx;
    cv::Mat intensity_dy;

    cv::Mat depth;
    cv::Mat depth_dx;
    cv::Mat depth_dy;

    cv::Mat normals, angles;

    cv::Mat rgb;

    PointCloud point_cloud;
    // cv::Mat_<Vector8f> acceleration;
    bool acceleration_requires_calculation_;

    size_t width, height;
    double timestamp;

    bool hasIntensity() const;
    bool hasDepth() const;
    bool hasRgb() const;

    void initialize();

    void calculateDerivatives();
    void calculateIntensityDerivatives();
    void calculateDepthDerivatives();
    void calculateNormals();
    
    // TODO: need to include the point cloud part in camera!
    void buildPointCloud();

    void buildAccelerationStructure();
    
    // inverse warping
    // transformation is the transformation from reference to this image
    void warpIntensity(const AffineTransform& transformation, const PointCloud& reference_pointcloud, 
                       const Intrinsic& intrinsics, RgbdImage& result, PointCloud& transformed_pointcloud);
    // forward warping
    // transformation is the transformation from this image to the reference image
    void warpIntensityForward(const AffineTransform& transformation, const Intrinsic& intrinsics, RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud);
    void warpDepthForward(const AffineTransform& transformation, const Intrinsic& intrinsics, RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud);
    
    bool inImage(const float& x, const float& y) const;
    // TOIGNORE: Advanced didn't used in the code
    // void warpDepthForwardAdvanced(const AffineTransform& transformation, const Intrinsic& intrinsics, RgbdImage& result);

private:
    bool intensity_requires_calculation_, depth_requires_calculation_, pointcloud_requires_build_;
    const RgbdCamera& camera_;

    void calculateDerivativeX(const cv::Mat& img, cv::Mat& result);
    void calculateDerivativeY(const cv::Mat& img, cv::Mat& result);

    enum WarpIntensityOptions
    {
        WithPointCloud,
        WithoutPointCloud,
    };

    // TOIGNORE: ignore following functions because of SSE
    // void calculateDerivativeYSseFloat(const cv::Mat& img, cv::Mat& result);
    // template<int PointCloudOption>
    // void warpIntensitySseImpl(const AffineTransform& transformation, const PointCloud& reference_pointcloud, const IntrinsicMatrix& intrinsics, RgbdImage& result, PointCloud& transformed_pointcloud);
};

}//namespace dvo