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
/*!\file    projects/icarus/sensor_processing/libstereo/tBlockBasedStereoMatching.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tBlockBasedStereoMatching.h"

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
namespace stereo_traversability_experiments
{
namespace aras
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
// tBlockBasedStereoMatching constructors
//----------------------------------------------------------------------
BlockBasedStereoMatching::BlockBasedStereoMatching()
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  radius_ = 5; //default value

}

//----------------------------------------------------------------------
// tBlockBasedStereoMatching destructor
//----------------------------------------------------------------------
BlockBasedStereoMatching::~BlockBasedStereoMatching()
{}

//////////////////////////////////////////////////////////////////////////////
void
BlockBasedStereoMatching::compute_impl(unsigned char* ref_img, unsigned char* trg_img)
{
  int n = radius_ * 2 + 1;
  int sad_min;
  short int dbest = 0;

  int sad_max = std::numeric_limits<int>::max();

  int *acc = new int[max_disp_];
  memset(acc, 0, sizeof(int) * max_disp_);

  int **v = new int *[width_];
  for (int d = 0; d < width_; d++)
  {
    v[d] = new int[max_disp_];
    memset(v[d], 0, sizeof(int) * max_disp_);
  }

  //First row
  for (int x = max_disp_ + x_off_; x < max_disp_ + x_off_ + n; x++)
  {
    for (int d = 0; d < max_disp_; d++)
    {
      for (int y = 0; y < n; y++)
        v[x][d] += abs(ref_img[(y * width_) + x] - trg_img[(y * width_) + x - d - x_off_]);
    }
  }

  //STAGE 2: other positions
  for (int x = max_disp_ + x_off_ + radius_ + 1; x < width_ - radius_; x++)
  {
    for (int d = 0; d < max_disp_; d++)
    {
      for (int y = 0; y < n; y++)
      {
        v[x + radius_][d] += abs(ref_img[(y * width_) + x + radius_] - trg_img[(y * width_) + x + radius_ - d - x_off_]);
      }
    }//d
  }//x

  //2
  unsigned char *lp, *rp, *lpp, *rpp;
  int ind1 = radius_ + radius_ + max_disp_ + x_off_;

  for (int y = radius_ + 1; y < height_ - radius_; y++)
  {
    //first position
    for (int d = 0; d < max_disp_; d++)
    {
      acc[d] = 0;
      for (int x = max_disp_ + x_off_; x < max_disp_ + x_off_ + n; x++)
      {
        v[x][d] +=  abs(ref_img[(y + radius_) * width_ + x] - trg_img[(y + radius_) * width_ + x - d - x_off_])
                    -
                    abs(ref_img[(y - radius_ - 1) * width_ + x] - trg_img[(y - radius_ - 1) * width_ + x - d - x_off_]);

        acc[d] += v[x][d];
      }
    }

    //all other positions
    lp  = static_cast<unsigned char*>(ref_img) + (y + radius_)    * width_ + ind1;
    rp  = static_cast<unsigned char*>(trg_img) + (y + radius_)    * width_ + ind1 - x_off_;
    lpp = static_cast<unsigned char*>(ref_img) + (y - radius_ - 1) * width_ + ind1;
    rpp = static_cast<unsigned char*>(trg_img) + (y - radius_ - 1) * width_ + ind1 - x_off_;

    for (int x = max_disp_ + x_off_ + radius_ + 1; x < width_ - radius_; x++)
    {
      sad_min = sad_max;

      lp++;
      rp++;
      lpp++;
      rpp++;

      for (int d = 0; d < max_disp_; d++)
      {
        v[x + radius_][d] += abs(*lp - *rp) - abs(*lpp - *rpp);

        acc[d] += v[x + radius_][d] - v[x - radius_ - 1][d];

        if (acc[d] < sad_min)
        {
          sad_min = acc[d];
          dbest = static_cast<short int>(d);
        }

        rp--;
        rpp--;
      }// d

      rp += max_disp_;
      rpp += max_disp_;

//      if (ratio_filter_ > 0)
//        dbest = doStereoRatioFilter(acc, dbest, sad_min, ratio_filter_, max_disp_);
//      if (peak_filter_ > 0)
//        dbest = doStereoPeakFilter(acc, dbest, peak_filter_, max_disp_);

      disp_map_[y * width_ + x] = static_cast<short int>(dbest * 16);

//      //subpixel refinement
//      if (dbest > 0 && dbest < max_disp_ - 1)
//        disp_map_[y * width_ + x] = computeStereoSubpixel(dbest, acc[dbest - 1], acc[dbest], acc[dbest + 1]);

    }//x
  }//y

  for (int d = 0; d < width_; d++)
    delete [] v[d];
  delete [] v;
  delete [] acc;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
