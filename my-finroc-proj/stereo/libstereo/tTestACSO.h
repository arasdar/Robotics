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
/*!\file    projects/icarus/sensor_processing/libstereo_test/tTestACSO.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 * \brief   Contains tTestACSO
 *
 * \b tTestACSO
 *
 * This is Adaptive Cost stereo matching technique class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__libstereo__tTestACSO_h__
#define __projects__stereo_traversability_experiments__aras__libstereo__tTestACSO_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tTimer.hpp"

#include <iomanip> //setprecision

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is Adaptive Cost stereo matching technique class.
 */
class tTestACSO : public tTestStereoMatching
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tTestACSO();

  ~tTestACSO();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  /** \brief setter for the radius (half length) of the column used for cost aggregation
    * \param[in] radius radius (half length) of the column used for cost aggregation; the total column length
    * is equal to 2*radius + 1
    */
  void
  setRadius(int radius)
  {
    radius_ = radius;
  };

  /** \brief setter for the spatial bandwith used for cost aggregation based on adaptive weights
    * \param[in] gamma_s spatial bandwith used for cost aggregation based on adaptive weights
    */
  void
  setGammaS(int gamma_s)
  {
    gamma_s_ = gamma_s;
  };

  /** \brief setter for the color bandwith used for cost aggregation based on adaptive weights
    * \param[in] gamma_c color bandwith used for cost aggregation based on adaptive weights
    */
  void
  setGammaC(int gamma_c)
  {
    gamma_c_ = gamma_c;
  };

  /** \brief "weak" smoothness penalty used within 2-pass Scanline Optimization
    * \param[in] smoothness_weak "weak" smoothness penalty cost
    */
  void
  setSmoothWeak(int smoothness_weak)
  {
    smoothness_weak_ = smoothness_weak;
  };

  /** \brief "strong" smoothness penalty used within 2-pass Scanline Optimization
    * \param[in] smoothness_strong "strong" smoothness penalty cost
    */
  void
  setSmoothStrong(int smoothness_strong)
  {
    smoothness_strong_ = smoothness_strong;
  };

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  void
  compute_impl(unsigned char* ref_img, unsigned char* trg_img);

  void compute_impl_parallel_openmp(const unsigned char* ref_img, const unsigned char* trg_img,
                                    const float *ds, const float* lut);

  void compute_impl_cuda(unsigned char* ref_img, unsigned char* trg_img);

  int radius_;

  //parameters for adaptive weight cost aggregation
  double gamma_c_;
  double gamma_s_;

  //Parameters for 2-pass SO optimization
  int smoothness_strong_;
  int smoothness_weak_;

  double lut_[256];

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
