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
/*!\file    projects/icarus/sensor_processing/libstereo_test/tTestStereoMatching.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

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
// tTestStereoMatching constructors
//----------------------------------------------------------------------
tTestStereoMatching::tTestStereoMatching()
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  disp_map_ = NULL;

  ref_img_ = NULL;
  trg_img_ = NULL;

  pp_ref_img_ = NULL;
  pp_trg_img_ = NULL;

  width_ = -1;
  height_ = -1;

  max_disp_ = -1;
  x_off_ = 0;

  is_pre_proc_ = false;
}

//----------------------------------------------------------------------
// tTestStereoMatching destructor
//----------------------------------------------------------------------
tTestStereoMatching::~tTestStereoMatching()
{
  if (disp_map_ != NULL)
  {
    delete [] disp_map_;
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

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
