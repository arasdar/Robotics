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
/*!\file    projects/icarus/sensor_processing/libstereo/tBlockBasedStereoMatching.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 * \brief   Contains tBlockBasedStereoMatching
 *
 * \b tBlockBasedStereoMatching
 *
 * This is Block based (or fixed window) Stereo Matching class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo__tBlockBasedStereoMatching_h__
#define __projects__icarus__sensor_processing__libstereo__tBlockBasedStereoMatching_h__

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
 * This is Block based (or fixed window) Stereo Matching class.
 *
  * This class implements the baseline Block-based - aka Fixed Window -  stereo matching algorithm.
  * The algorithm includes a running box filter so that the computational complexity is independent of
  * the size of the window ( O(1) wrt. to the size of window)
  * The algorithm is based on the Sum of Absolute Differences (SAD) matching function
  * Only works with grayscale (single channel) rectified images
 *
 */
class BlockBasedStereoMatching : public GrayStereoMatching
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  BlockBasedStereoMatching();

  ~BlockBasedStereoMatching();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  /** \brief setter for the radius of the squared window
    * \param[in] radius radius of the squared window used to compute the block-based stereo algorithm
    * the window side is equal to 2*radius + 1
    */
  void
  setRadius(int radius)
  {
    radius_ = radius;
  };

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  int radius_;

  virtual void
  compute_impl(unsigned char* ref_img, unsigned char* trg_img);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
