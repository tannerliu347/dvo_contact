#pragma once
#include "intrinsic.h"
#include "rgbd_image.h"

namespace dvo
{

// Point selection virtual function definition:
class PtSelectionVerify
{
public:
  virtual ~PtSelectionVerify() {}
  // (x, y): point position in image; z: depth; i: intensity;
  // idx (idy): intensity gradient along x (y);
  // ddx (ddy): intensity gradient along x (y);
  virtual bool isPointOk(const size_t& x, const size_t& y, const float& z, const float& idx, const float& idy, const float& ddx, const float& ddy) const = 0;
};

// Valid point verification:
class PtVerify : public PtSelectionVerify
{
public:
  virtual ~PtVerify() {}
  // (x, y): point position in image; z: depth; i: intensity;
  // idx (idy): intensity gradient along x (y);
  // ddx (ddy): intensity gradient along x (y);
  virtual bool isPointOk(const size_t& x, const size_t& y, const float& z, const float& idx, const float& idy, const float& ddx, const float& ddy) const
  {
    return 1;
  }
};

// Valid point and gradient verification:
class PtAndGradVerify : public PtSelectionVerify
{
public:
  float intensity_threshold;
  float depth_threshold;

  PtAndGradVerify() :
    intensity_threshold(0.0f),
    depth_threshold(0.0f)
  {
  }

  virtual ~PtAndGradVerify() {}

  // (x, y): point position in image; z: depth; i: intensity;
  // idx (idy): intensity gradient along x (y);
  // ddx (ddy): intensity gradient along x (y);
  virtual bool isPointOk(const size_t& x, const size_t& y, const float& z, const float& idx, const float& idy, const float& ddx, const float& ddy) const
  {
    return (std::abs(idx) > intensity_threshold || std::abs(idy) > intensity_threshold || std::abs(ddx) > depth_threshold ||  std::abs(ddy) > depth_threshold);
  }
};

// Select valid points:
class PtSelection
{
public:
  typedef std::vector<PtIntensityDepth, Eigen::aligned_allocator<PtIntensityDepth> > PtID_VectorType;
  typedef PtID_VectorType::iterator PtIterator;

  PtSelection(const PtSelectionVerify& verify);
  PtSelection(dvo::ImagePyramid& pyramid, const PtSelectionVerify& verify);
  virtual ~PtSelection();

  dvo::ImagePyramid& getImagePyramid();

  void setImagePyramid(dvo::ImagePyramid& pyramid);

  size_t getMaxNumOfPoints(const size_t& level);

  void select(const size_t& level, PtIterator& first_point, PtIterator& last_point);

  void recycle(dvo::ImagePyramid& pyramid);

  bool getDebugIndex(const size_t& level, cv::Mat& dbg_idx);

  void debug(bool v)
  {
    debug_ = v;
  }

  bool debug() const
  {
    return debug_;
  }

private:
  struct Storage
  {
  public:
    PtID_VectorType points;
    PtIterator points_end;
    bool is_cached;

    cv::Mat debug_idx;

    Storage();
    void allocate(size_t max_points);
  };

  dvo::ImagePyramid *pyramid_;
  std::vector<Storage> storage_;
  const PtSelectionVerify& verify_;

  bool debug_;

  PtIterator selectPointsFromImage(const dvo::RgbdImage& img, const PtIterator& first_point, const PtIterator& last_point, cv::Mat& debug_idx);
};

} /* namespace dvo */

