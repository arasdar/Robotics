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
/*!\file    projects/icarus/sensor_processing/libstereo/tGrayStereoMatching.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tGrayStereoMatching.h"

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
// tGrayStereoMatching constructors
//----------------------------------------------------------------------
GrayStereoMatching::GrayStereoMatching()
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{}

//----------------------------------------------------------------------
// tGrayStereoMatching destructor
//----------------------------------------------------------------------
GrayStereoMatching::~GrayStereoMatching()
{}

//////////////////////////////////////////////////////////////////////////////
void
GrayStereoMatching::preProcessing(unsigned char *img, unsigned char *pp_img)
{
  int radius = 4;              //default value, could be exported
  int n = 2 * radius + 1;
  int area = n * n;
  int threshold = 31;

  int sum = 0;
  int *v = new int[width_];
  memset(v, 0, sizeof(int) * width_);

  for (int x = 0; x < n; x++)
    for (int y = 0; y < n; y++)
      v[x] += img[y * width_ + x];

  for (int x = radius + 1; x < width_ - radius; x++)
    for (int y = 0; y < n; y++)
      v[x + radius] += img[y * width_ + x + radius];

  for (int y = 0; y <= radius; y++)
    for (int x = 0; x < width_; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

  for (int y = radius + 1; y < height_ - radius; y++)
  {
    for (int x = 0; x <= radius; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

    sum = 0;
    for (int x = 0; x < n; x++)
    {
      v[x] = v[x] + img[(y + radius) * width_ + x] - img[(y - radius - 1) * width_ + x];
      sum += v[x];
    }

    for (int x = radius + 1; x < width_ - radius; x++)
    {
      v[x + radius] = v[x + radius] + img[(y + radius) * width_ + x + radius] - img[(y - radius - 1) * width_ + x + radius];
      sum = sum + v[x + radius] - v[x - radius - 1];

      short int temp = static_cast<short int>(img[y * width_ + x] - (sum / area));

      if (temp < -threshold)
        pp_img[y * width_ + x] = 0;
      else if (temp >   threshold)
        pp_img[y * width_ + x] = static_cast<unsigned char>(threshold + threshold);
      else
        pp_img[y * width_ + x] = static_cast<unsigned char>(temp + threshold);

    }

    for (int x = width_ - radius; x < width_; x++)
    {
      pp_img[y * width_ + x] = img[y * width_ + x];
    }
  }

  for (int y = height_ - radius; y < height_; y++)
  {
    for (int x = 0; x < width_; x++)
    {
      pp_img[y * width_ + x] = img[y * width_ + x];
    }
  }

  delete [] v;
}

//////////////////////////////////////////////////////////////////////////////
void
GrayStereoMatching::imgFlip(unsigned char * & img)
{
  unsigned char *temp_row = new unsigned char[width_];

  for (int j = 0; j < height_; j++)
  {
    memcpy(temp_row, img + j * width_, sizeof(unsigned char) * width_);
    for (int i = 0; i < width_; i++)
    {
      img[j * width_ + i] = temp_row[width_ - 1 - i];
    }
  }

  delete [] temp_row;
}

//////////////////////////////////////////////////////////////////////////////
void
GrayStereoMatching::compute(pcl::PointCloud<pcl::RGB> &ref, pcl::PointCloud<pcl::RGB> &trg)
{

  if (ref.width != trg.width || ref.height != trg.height)
  {

    PCL_ERROR(
      "[pcl::GrayStereoMatching::compute] Error. The two input clouds have different sizes. Aborting..\n"
    );
    return;
  }

  if ((ref_img_ != NULL) && (width_ != ref.width || height_ != ref.height))
  {
    delete [] ref_img_;
    delete [] trg_img_;

    ref_img_ = NULL;
    trg_img_ = NULL;
  }

  if (ref_img_ == NULL)
  {
    ref_img_ = new unsigned char[ref.width * ref.height];
    trg_img_ = new unsigned char[ref.width * ref.height];
  }

  float divider = 1.0f / 3.0f;
  for (unsigned int j = 0; j < ref.height; j++)
  {
    for (unsigned int i = 0; i < ref.width; i++)
    {
      ref_img_[j * ref.width + i] = static_cast<unsigned char>(static_cast<float>(ref[j * ref.width + i].r + ref[j * ref.width + i].g + ref[j * ref.width + i].b) * divider);
      trg_img_[j * ref.width + i] = static_cast<unsigned char>(static_cast<float>(trg[j * ref.width + i].r + trg[j * ref.width + i].g + trg[j * ref.width + i].b) * divider);
      //ref_img_[ j*ref.width + i] = ( ref(j,i).r + ref(j,i).g + ref(j,i).b) / 3;
      //trg_img_[ j*ref.width + i] = ( trg(j,i).r + trg(j,i).g + trg(j,i).b) / 3;

    }
  }

  compute(ref_img_, trg_img_, ref.width, ref.height);

}

//////////////////////////////////////////////////////////////////////////////
void
GrayStereoMatching::compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height)
{

  //Check that a suitable value of max_disp has been selected
  if (max_disp_ <= 0)
  {
    PCL_ERROR(
      "[pcl::StereoMatching::compute] Error. A positive max_disparity value has not be correctly inserted. Aborting..\n"
    );
    return;
  }

  if ((disp_map_ != NULL) && (width_ != width || height_ != height))
  {
    delete [] disp_map_;
    disp_map_ = NULL;

    if (disp_map_trg_ != NULL)
    {
      delete [] disp_map_trg_;
      disp_map_trg_ = NULL;
    }

    if (pp_ref_img_ != NULL)
    {
      delete [] pp_ref_img_;
      delete [] pp_trg_img_;
      pp_ref_img_ = NULL;
      pp_trg_img_ = NULL;
    }
  }

  if (disp_map_ == NULL)
  {
    disp_map_ = new short int[width * height];

    width_ = width;
    height_ = height;
  }


  if (is_lr_check_ && disp_map_trg_ == NULL)
  {
    disp_map_trg_ = new short int[width * height];
  }

  if (!is_lr_check_ && disp_map_trg_ != NULL)
  {
    delete [] disp_map_trg_;
    disp_map_trg_ = NULL;
  }

  if (is_pre_proc_ && pp_ref_img_ == NULL)
  {
    pp_ref_img_ = new unsigned char[width_ * height_];
    pp_trg_img_ = new unsigned char[width_ * height_];
  }

  if (!is_pre_proc_ && pp_ref_img_ != NULL)
  {
    delete [] pp_ref_img_;
    delete [] pp_trg_img_;
    pp_ref_img_ = NULL;
    pp_trg_img_ = NULL;
  }

  memset(disp_map_, 0, sizeof(short int)*height_ * width_);

  if (is_pre_proc_)
  {
    preProcessing(ref_img, pp_ref_img_);
    preProcessing(trg_img, pp_trg_img_);
  }

  if (is_lr_check_)
  {

    if (is_pre_proc_)
    {
      imgFlip(pp_ref_img_);
      imgFlip(pp_trg_img_);

      compute_impl(pp_trg_img_, pp_ref_img_);

      imgFlip(pp_ref_img_);
      imgFlip(pp_trg_img_);
    }
    else
    {
      imgFlip(ref_img);
      imgFlip(trg_img);

      compute_impl(trg_img, ref_img);

      imgFlip(ref_img);
      imgFlip(trg_img);
    }

    for (int j = 0; j < height_; j++)
      for (int i = 0; i < width_; i++)
        disp_map_trg_[j * width_ + i] = disp_map_[j * width_ + width_ - 1 - i];

  }

  if (is_pre_proc_)
    compute_impl(pp_ref_img_, pp_trg_img_);
  else
    compute_impl(ref_img, trg_img);

  if (is_lr_check_)
  {
    leftRightCheck();
  }

  //at the end, x_offset (*16) needs to be added to all computed disparities,
  //so that each fixed point value of the disparity map represents the true disparity value multiplied by 16
  for (int j = 0; j < height_; j++)
    for (int i = 0; i < width_; i++)
      if (disp_map_[j * width_ + i] > 0)
        disp_map_[j * width_ + i] += static_cast<short int>(x_off_ * 16);

}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
