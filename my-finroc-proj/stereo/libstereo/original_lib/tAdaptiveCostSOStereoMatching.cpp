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
/*!\file    projects/icarus/sensor_processing/libstereo/tAdaptiveCostSOStereoMatching.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tAdaptiveCostSOStereoMatching.h"

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
// tAdaptiveCostSOStereoMatching constructors
//----------------------------------------------------------------------
AdaptiveCostSOStereoMatching::AdaptiveCostSOStereoMatching()
/*If you have some member variables, please initialize them here. Especially built - in types(like pointers!). Delete this line otherwise!*/
{
  radius_ = 5;

  gamma_s_ = 15;
  gamma_c_ = 25;

  smoothness_strong_ = 100;
  smoothness_weak_ = 20;

}

//----------------------------------------------------------------------
// tAdaptiveCostSOStereoMatching destructor
//----------------------------------------------------------------------
AdaptiveCostSOStereoMatching::~AdaptiveCostSOStereoMatching()
{}

//////////////////////////////////////////////////////////////////////////////
void
AdaptiveCostSOStereoMatching::compute_impl(unsigned char* ref_img, unsigned char* trg_img)
{
  //int n = radius_ * 2 + 1;
  //int sad_max = std::numeric_limits<int>::max ();



  //spatial distance init
  float *ds = new float[ 2 * radius_ + 1 ];
  for (int j = -radius_; j <= radius_; j++)
    ds[j + radius_] = static_cast<float>(exp(- abs(j) / gamma_s_));

  //LUT for color distance weight computation
  float lut[256];
  for (int j = 0; j < 256; j++)
    lut[j] = float(exp(-j / gamma_c_));

//  /*
//  * main loop to iterate through the entire left and right images
//  */
//  std::cout << "OpenMP version using " << omp_get_max_threads() << " threads" << std::endl;
//
//  #pragma omp parallel for

  for (int y = radius_ + 1; y < height_ - radius_; y++)
  {

    float **acc = new float *[width_];
    for (int d = 0; d < width_; d++)
    {
      acc[d] = new float[max_disp_];
      memset(acc[d], 0, sizeof(float) * max_disp_);
    }

    //data structures for Scanline Optimization
    float **fwd = new float *[width_];
    float **bck = new float *[width_];
    for (int d = 0; d < width_; d++)
    {
      fwd[d] = new float[max_disp_];
      memset(fwd[d], 0, sizeof(float)*max_disp_);
      bck[d] = new float[max_disp_];
      memset(bck[d], 0, sizeof(float)*max_disp_);
    }

    //left weight array alloc
    float *wl = new float [ 2 * radius_ + 1 ];

    for (int x = x_off_ + max_disp_ + 1; x < width_; x++)
    {
      for (int j = -radius_; j <= radius_; j++)
        wl[j + radius_] = lut[ abs(ref_img[(y + j) * width_ + x] - ref_img[y * width_ + x]) ] * ds[j + radius_];

      for (int d = 0; d < max_disp_; d++)
      {
        float sumw  = 0.0;
        float num = 0.0;

        for (int j = -radius_; j <= radius_; j++)
        {
          float weight_r = lut[ abs(trg_img[(y + j) * width_ + x - d - x_off_] - trg_img[y * width_ + x - d - x_off_]) ] * ds[j + radius_];
          int sad = abs(ref_img[(y + j) * width_ + x] - trg_img[(y + j) * width_ + x - d - x_off_]);
          num += wl[j + radius_] * weight_r * static_cast<float>(sad);
          sumw += wl[j + radius_] * weight_r;
        }

        acc[x][d] = num / sumw;

      }//d
    }//x

    //Forward
    for (int d = 0; d < max_disp_; d++)
      fwd[max_disp_ + 1][d] = acc[max_disp_ + 1][d];

    for (int x = x_off_ + max_disp_ + 2; x < width_; x++)
    {
      float c_min = fwd[x - 1][0];
      for (int d = 1; d < max_disp_; d++)
        if (fwd[x - 1][d] < c_min)
          c_min = fwd[x - 1][d];

      fwd[x][0] =  acc[x][0] - c_min + std::min(fwd[x - 1][0], std::min(fwd[x - 1][1] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));
      for (int d = 1; d < max_disp_ - 1; d++)
      {
        fwd[x][d] = acc[x][d] - c_min + std::min(std::min(fwd[x - 1][d], fwd[x - 1][d - 1] + static_cast<float>(smoothness_weak_)), std::min(fwd[x - 1][d + 1] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));
      }
      fwd[x][max_disp_ - 1] = acc[x][max_disp_ - 1] - c_min + std::min(fwd[x - 1][max_disp_ - 1], std::min(fwd[x - 1][max_disp_ - 2] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));
    }//x

    //Backward
    for (int d = 0; d < max_disp_; d++)
      bck[width_ - 1][d] = acc[width_ - 1][d];

    for (int x = width_ - 2; x > max_disp_ + x_off_; x--)
    {

      float c_min = bck[x + 1][0];
      for (int d = 1; d < max_disp_; d++)
        if (bck[x + 1][d] < c_min)
          c_min = bck[x + 1][d];

      bck[x][0] =  acc[x][0] - c_min + std::min(bck[x + 1][0], std::min(bck[x + 1][1] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));
      for (int d = 1; d < max_disp_ - 1; d++)
        bck[x][d] = acc[x][d] - c_min + std::min(std::min(bck[x + 1][d], bck[x + 1][d - 1] + static_cast<float>(smoothness_weak_)), std::min(bck[x + 1][d + 1] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));
      bck[x][max_disp_ - 1] = acc[x][max_disp_ - 1] - c_min + std::min(bck[x + 1][max_disp_ - 1], std::min(bck[x + 1][max_disp_ - 2] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));
    }//x

    //last scan
    for (int x = x_off_ + max_disp_ + 1; x < width_; x++)
    {
      float c_min = std::numeric_limits<float>::max();
      short int dbest = 0;

      for (int d = 0; d < max_disp_; d++)
      {
        acc[x][d] = fwd[x][d] + bck[x][d];
        if (acc[x][d] < c_min)
        {
          c_min = acc[x][d];
          dbest = static_cast<short int>(d);
        }
      }

      if (ratio_filter_ > 0)
        dbest = doStereoRatioFilter(acc[x], dbest, c_min, ratio_filter_, max_disp_);
      if (peak_filter_ > 0)
        dbest = doStereoPeakFilter(acc[x], dbest, peak_filter_, max_disp_);

      disp_map_[y * width_ + x] = static_cast<short int>(dbest * 16);

      //subpixel refinement
      if (dbest > 0 && dbest < max_disp_ - 1)
        disp_map_[y * width_ + x] = computeStereoSubpixel(dbest, acc[x][dbest - 1], acc[x][dbest], acc[x][dbest + 1]);
    } //x

    for (int x = 0; x < width_; x++)
    {
      delete [] fwd[x];
      delete [] bck[x];
      delete [] acc[x];
    }
    delete [] fwd;
    delete [] bck;
    delete [] acc;
    delete [] wl;

  }//y


  delete [] ds;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
