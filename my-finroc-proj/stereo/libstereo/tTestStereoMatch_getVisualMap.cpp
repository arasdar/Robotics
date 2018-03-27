/*
 * tTestStereoMatch_getVisualMap.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */



#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

using namespace finroc::stereo_traversability_experiments::aras::libstereo;

//////////////////////////////////////////////////////////////////////////////Opencv
void
tTestStereoMatching::getVisualMap(cv::Mat vMap)
{

  //vMap.create(height_, width_, CV_8UC3);

  float scale = 255.0f / (16.0f * static_cast<float>(max_disp_));

  for (unsigned int y = 0; y < height_; y++) //rows = y
  {
    for (unsigned int x = 0; x < width_; x++) //cols = x
    {
      if (disp_map_[y * width_ + x] <= 0)
      {
        vMap.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
      }
      else
      {
        unsigned char val = static_cast<unsigned char>(floor(scale * disp_map_[y * width_ + x]));
        vMap.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
void
tTestStereoMatching::getVisualMap(pcl::PointCloud<pcl::RGB>::Ptr vMap)
{

  if (vMap->width != width_ || vMap->height != height_)
  {
    vMap->resize(width_ * height_);
    vMap->width = width_;
    vMap->height = height_;
  }

  if (vMap->is_dense)
    vMap->is_dense = false;

  pcl::RGB invalid_val;
  invalid_val.r = 0;
  invalid_val.g = 255;
  invalid_val.b = 0;

  float scale = 255.0f / (16.0f * static_cast<float>(max_disp_));

  for (int y = 0; y < height_; y++)
  {
    for (int x = 0; x < width_; x++)
    {
      if (disp_map_[y * width_ + x] <= 0)
      {
        vMap->at(x, y) = invalid_val;
      }
      else
      {
        unsigned char val = static_cast<unsigned char>(floor(scale * disp_map_[y * width_ + x]));
        vMap->at(x, y).r = val;
        vMap->at(x, y).g = val;
        vMap->at(x, y).b = val;
      }
    }
  }
}
