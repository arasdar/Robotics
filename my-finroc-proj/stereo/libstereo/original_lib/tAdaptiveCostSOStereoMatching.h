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
/*!\file    projects/icarus/sensor_processing/libstereo/tAdaptiveCostSOStereoMatching.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 * \brief   Contains tAdaptiveCostSOStereoMatching
 *
 * \b tAdaptiveCostSOStereoMatching
 *
 * This is Adaptive Cost 2-pass Scanline Optimization Stereo Matching class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo__tAdaptiveCostSOStereoMatching_h__
#define __projects__icarus__sensor_processing__libstereo__tAdaptiveCostSOStereoMatching_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tGrayStereoMatching.h"

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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is Adaptive Cost 2-pass Scanline Optimization Stereo Matching class.
 *
  * This class implements an adaptive-cost stereo matching algorithm based on 2-pass Scanline Optimization.
  * The algorithm is inspired by the paper:
  * [1] L. Wang et al., "High Quality Real-time Stereo using Adaptive Cost Aggregation and Dynamic Programming", 3DPVT 2006
  * Cost aggregation is performed using adaptive weigths computed on a single column as proposed in [1].
  * Instead of using Dynamic Programming as in [1], the optimization is performed via 2-pass Scanline Optimization.
  * The algorithm is based on the Sum of Absolute Differences (SAD) matching function
  * Only works with grayscale (single channel) rectified images
  *
 */
class AdaptiveCostSOStereoMatching: public GrayStereoMatching
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  AdaptiveCostSOStereoMatching();

  ~AdaptiveCostSOStereoMatching();  //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

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
  virtual void
  compute_impl(unsigned char* ref_img, unsigned char* trg_img);

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
