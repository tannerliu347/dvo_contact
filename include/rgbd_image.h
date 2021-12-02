#include <opencv2/opencv.hpp>

#include <memory>

#include "intrinsic.h"

namespace dvo {

typedef Eigen::Matrix<float, 8, 1> Vector8f;
typedef Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor> PointCloud;
typedef std::shared_ptr<RgbdCamera> RgbdCameraPtr;
typedef std::shared_ptr<const RgbdCamera> RgbdCameraConstPtr;

class PtIntensityDepth{
public:
    Eigen::Vector4f getPointVec() {
        Eigen::Vector4f res;
        res << x_, y_, z_, 0.0f;
        return res;
    }

    Eigen::Vector2f getIntensityDepty() {
        Eigen::Vector2f res;
        res << i_, d_;
        return res;
    }

    Eigen::Vector2f getIntensityDeriv() {
        Eigen::Vector2f res;
        res << idx_, idy_;
        return res;
    }

    Eigen::Vector2f getDepthDeriv() {
        Eigen::Vector2f res;
        res << ddx_, ddy_;
        return res;
    }

    Vector8f getIntensityDepthAndDeriv() {
        Eigen::Vector2f res;
        res << i_, d_, idx_, idy_, ddx_, ddy_, time_interp_, 0.0f;
        return res;
    }

private:
    float x_, y_, z_;
    float i_, d_, idx_, idy_, ddx_, ddy_, time_interp_;
};

class RgbdImage;
typedef std::shared_ptr<RgbdImage> RgbdImagePtr;

class ImagePyramid;
typedef std::shared_ptr<ImagePyramid> ImagePyramidPtr;

class RgbdCamera {
public:
    RgbdCamera(size_t width, size_t height, const dvo::Intrinsic& intrinsics);
    ~RgbdCamera() {}

    size_t width() const;
    size_t height() const;

    const dvo::Intrinsic& intrinsics() const;

    RgbdImagePtr create(const cv::Mat& intensity, const cv::Mat& depth) const;
    RgbdImagePtr create() const;

private:
    bool hasSamesize(const cv::Mat& img) const;

private:
    size_t width_, height_;
    dvo::Intrinsic intrinsics_;
    PointCloud pointcloud_template_;
};

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

}