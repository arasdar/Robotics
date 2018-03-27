//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/sensor_processing/libstereo/tStereoMatching.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tStereoMatching.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace libstereo
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tStereoMatching constructors
//----------------------------------------------------------------------
StereoMatching::StereoMatching()
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  disp_map_ = NULL;
  disp_map_trg_ = NULL;

  ref_img_ = NULL;
  trg_img_ = NULL;

  pp_ref_img_ = NULL;
  pp_trg_img_ = NULL;

  width_ = -1;
  height_ = -1;

  max_disp_ = -1;
  x_off_ = 0;

  ratio_filter_ = 0;
  peak_filter_ = 0;

  is_pre_proc_ = false;
  is_lr_check_ = false;
  lr_check_th_ = 1;
}

//----------------------------------------------------------------------
// tStereoMatching destructor
//----------------------------------------------------------------------
StereoMatching::~StereoMatching()
{
  if (disp_map_ != NULL)
  {
    delete [] disp_map_;
    //disp_map_ = NULL;
  }

  if (disp_map_trg_ != NULL)
  {
    delete [] disp_map_trg_;
    //disp_map_trg_ = NULL;
  }

  if (ref_img_ != NULL)
  {
    delete [] ref_img_;
    delete [] trg_img_;
  }

  if (pp_ref_img_ != NULL)
  {
    delete [] pp_ref_img_;
    delete [] pp_trg_img_;
  }
}

////----------------------------------------------------------------------
//// tStereoMatching postprocessing - median filter
////----------------------------------------------------------------------
void StereoMatching::medianFilter(int radius)
{

  //TODO: do median filter
  int side = radius * 2 + 1;

  short int *out = new short int [width_ * height_];
  memset(out, 0, width_ * height_ * sizeof(short int));

  short int *v = new short int [side * side];

  for (int y = radius; y < height_ - radius; y++)
  {
    for (int x = radius; x < width_ - radius; x++)
    {

      if (disp_map_[y * width_ + x] <= 0)
        out[y * width_ + x] = disp_map_[y * width_ + x];
      else
      {

        int n = 0;
        for (int j = -radius; j <= radius; j++)
        {
          for (int i = -radius; i <= radius; i++)
          {
            if (disp_map_[(y + j)*width_ + x + i] > 0)
            {
              v[n] = disp_map_[(y + j) * width_ + x + i];
              n++;
            }
          }
        }

        std::sort(v, v + n);
        out[y * width_ + x] = v[n / 2];
      }
    }
  }

  short int* temp_ptr = out;
  out = disp_map_;
  disp_map_ = temp_ptr;

  delete [] out;
  delete [] v;
}

//////////////////////////////////////////////////////////////////////////////
void
StereoMatching::getVisualMap(pcl::PointCloud<pcl::RGB>::Ptr vMap)
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

//////////////////////////////////////////////////////////////////////////////
void
StereoMatching::leftRightCheck()
{
  short int p1, p2, p2i;

  for (int y = 0; y < height_; y++)
  {
    for (int x = 0; x < width_; x++)
    {
      if (disp_map_[y * width_ + x] > 0)
      {
        p1 = disp_map_[y * width_ + x] / 16;

        p2i = static_cast<short int>(x - p1 - x_off_);

        if (p2i >= 0)
        {
          p2 = disp_map_trg_[y * width_ + p2i] / 16;

          if (abs(p1 - p2) > lr_check_th_)
            disp_map_[y * width_ + x] = -8;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
bool
StereoMatching::getPointCloud(
  float u_c, float v_c, float focal, float baseline,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::RGB>::Ptr texture)
{
  //disp map has not been computed yet..
  if (disp_map_ == NULL)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n");

    return (false);
  }

  if (texture->width != width_ || texture->height != height_)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: the size of the texture cloud does not match that of the computed range map. The resulting cloud can not be computed..\n");
    return (false);
  }

  //cloud needs to be re-allocated
  if (cloud->width != width_ || cloud->height != height_)
  {
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  //Loop
  pcl::PointXYZRGB temp_point;
  /*pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.b = std::numeric_limits<unsigned char>::quiet_NaN();*/

  //all disparities are multiplied by a constant equal to 16;
  //this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (int j = 0; j < height_; j++)
  {
    for (int i = 0; i < width_; i++)
    {
      if (disp_map_[ j * width_ + i] > 0)
      {
        temp_point.z = (depth_scale) / (disp_map_[ j * width_ + i]);
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;

        //temp_point.intensity = ( texture->at(j*width_+i).r +texture->at(j*width_+i).g + texture->at(j*width_+i).b) / 3.0f;
        temp_point.r = texture->at(j * width_ + i).r;
        temp_point.g = texture->at(j * width_ + i).g;
        temp_point.b = texture->at(j * width_ + i).b;

        (*cloud)[j * width_ + i] = temp_point;
      }
      //adding NaN value
      else
      {
        temp_point.x = std::numeric_limits<float>::quiet_NaN();
        temp_point.y = std::numeric_limits<float>::quiet_NaN();
        temp_point.z = std::numeric_limits<float>::quiet_NaN();
        temp_point.r = texture->at(j * width_ + i).r;
        temp_point.g = texture->at(j * width_ + i).g;
        temp_point.b = texture->at(j * width_ + i).b;
        (*cloud)[j * width_ + i] = temp_point;
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
StereoMatching::getPointCloud(
  float u_c, float v_c, float focal, float baseline,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  //disp map has not been computed yet..
  if (disp_map_ == NULL)
  {

    PCL_ERROR(
      "[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n"
    );

    return false;
  }

  //cloud needs to be re-allocated
  if (cloud->width != width_ || cloud->height != height_)
  {
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  if (cloud->is_dense)
    cloud->is_dense = false;

  //Loop
  pcl::PointXYZ temp_point;
  pcl::PointXYZ nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  //nan_point.intensity = std::numeric_limits<float>::quiet_NaN();

  //all disparities are multiplied by a constant equal to 16;
  //this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (int j = 0; j < height_; j++)
  {
    for (int i = 0; i < width_; i++)
    {
      if (disp_map_[ j * width_ + i] > 0)
      {

        temp_point.z = depth_scale / disp_map_[j * width_ + i];
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;
        //temp_point.intensity = 255;

        (*cloud)[j * width_ + i] = temp_point;
      }
      //adding NaN value
      else
      {
        (*cloud)[j * width_ + i] = nan_point;
      }
    }
  }

  return (true);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
