#include "point_selection.h"

typedef unsigned int uint;

namespace dvo
{

PtSelection::PtSelection(const PtSelectionVerify& verify) :
    /// TODO: try to use nullptr: (instead of using 0)
    pyramid_(nullptr), 
    verify_(verify),
    debug_(false)
{
}

PtSelection::PtSelection(dvo::ImagePyramid& pyramid, const PtSelectionVerify& verify) :
    pyramid_(&pyramid),
    verify_(verify),
    debug_(false)
{
}

PtSelection::~PtSelection()
{
}

void PtSelection::recycle(dvo::ImagePyramid& pyramid)
{
  setImagePyramid(pyramid);
}

void PtSelection::setImagePyramid(dvo::ImagePyramid& pyramid)
{
  pyramid_ = &pyramid;

  for(size_t idx = 0; idx < storage_.size(); ++idx)
  {
    storage_[idx].is_cached = false;
  }
}

dvo::ImagePyramid& PtSelection::getImagePyramid()
{
  assert(pyramid_ != 0);

  return *pyramid_;
}

size_t PtSelection::getMaxNumOfPoints(const size_t& level)
{
  /// TODO: Formula, use directly
  return size_t(pyramid_->level(0).intensity.total() * std::pow(0.25, double(level)));
}


bool PtSelection::getDebugIndex(const size_t& level, cv::Mat& dbg_idx)
{
  // dbg_idx: debug index
  if(debug_ && storage_.size() > level)
  {
    dbg_idx = storage_[level].debug_idx;

    return dbg_idx.total() > 0;
  }
  else
  {
    return false;
  }
}


// given current level
void PtSelection::select(const size_t& level, PtSelection::PtIterator& first_point, PtSelection::PtIterator& last_point)
{
  assert(pyramid_ != 0);

  pyramid_->build(level + 1);

  if(storage_.size() < level + 1)
    storage_.resize(level + 1);

  /// TODO: Change variable name
  Storage& storage_level = storage_[level];

  if(!storage_level.is_cached || debug_)
  {
    std::cout << "Ready to create img" << std::endl;
    dvo::RgbdImage& img = pyramid_->level(level);
    
    img.buildPointCloud();
    img.buildAccelerationStructure();

    if(debug_)
      storage_level.debug_idx = cv::Mat::zeros(img.intensity.size(), CV_8UC1);

    storage_level.allocate(img.intensity.total());
    storage_level.points_end = selectPointsFromImage(img, storage_level.points.begin(), storage_level.points.end(), storage_level.debug_idx);

    storage_level.is_cached = true;
  }

  first_point = storage_level.points.begin();
  last_point = storage_level.points_end;
}

PtSelection::PtIterator PtSelection::selectPointsFromImage(const dvo::RgbdImage& img, const PtSelection::PtIterator& first_point, const PtSelection::PtIterator& last_point, cv::Mat& debug_idx)
{
  uint w = img.width;
  uint h = img.height;
  std::vector<PtIntensityDepth> points_with_IandD(w * h);
  for (size_t idx = 0; idx < img.point_cloud.cols(); idx++) {
    PtIntensityDepth point;
    point.x = img.point_cloud(0, idx);
    point.y = img.point_cloud(1, idx);
    point.z = img.point_cloud(2, idx);

    // idx to i, j
    uint i = idx / w;
    uint j = idx % w;
    point.i = img.intensity.at<float>(i, j);
    point.d = img.depth.at<float>(i, j); 
    point.idx = img.intensity_dx.at<float>(i, j); 
    point.idy = img.intensity_dy.at<float>(i, j); 
    point.ddx = img.depth_dx.at<float>(i, j); 
    point.ddy = img.depth_dy.at<float>(i, j); 
    // point.time_interp = img.timestamp.at<double>(i, j); 
    point.time_interp = 0; 
    points_with_IandD[idx] = point;
  }

  PtIterator selected_points_it = first_point;

  for(int y = 0; y < h; ++y)
  {
    //float time_interpolation = 1 + (y - 0.5f * img.height) * dt;

    for(int x = 0; x < w; ++x)
    {
      size_t idx = y * w + x;
      if(verify_.isPointOk(x, y, points_with_IandD[idx].z, 
       points_with_IandD[idx].idx, points_with_IandD[idx].idy, 
       points_with_IandD[idx].ddx, points_with_IandD[idx].ddy))
      {
        *selected_points_it = points_with_IandD[idx];

        ++selected_points_it;

        if(debug_)
          debug_idx.at<uint8_t>(y, x) = 1;

        if(selected_points_it == last_point)
          return selected_points_it;
      }
    }
  }

  return selected_points_it;
}

PtSelection::Storage::Storage() :
    points(),
    points_end(points.end()),
    is_cached(false)
{
}

void PtSelection::Storage::allocate(size_t max_points)
{
  if(points.size() < max_points)
  {
    points.resize(max_points);
  }
}

} /* namespace dvo */


