//
// You received this file as part of Finroc
// A Framework for intelligent robot control
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
/*!\file    projects/icarus/simulation_physx_ogre/mTrackVelocityController.h
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-11-28
 *
 * \brief Contains mTrackVelocityController
 *
 * \b mTrackVelocityController
 *
 * This module implements a feed-back controller for the left and right track speed.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__simulation_physx_ogre__mTrackVelocityController_h__
#define __projects__stereo_traversability_experiments__simulation_physx_ogre__mTrackVelocityController_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation_physx_ogre
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module implements a feed-back controller for the left and right track speed.
 */
class mTrackVelocityController : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<float> in_velocity_left;
  tInput<float> in_velocity_right;

  // actual value
  tInput<float> in_actual_velocity_left;
  tInput<float> in_actual_velocity_right;

  //Controller outputs
  tOutput<float> out_acc;
  tOutput<float> out_thrust_left;
  tOutput<float> out_thrust_right;
  tOutput<float> out_brake_left;
  tOutput<float> out_brake_right;

  tOutput<float> out_error_left;
  tOutput<float> out_error_right;
  tOutput<float> out_acc_i;
  tOutput<float> out_i_right;
  tOutput<float> out_i_left;

  // pi controller parameter
  tParameter<float> par_diff_Kp;
  tParameter<float> par_diff_Ki;
  tParameter<float> par_diff_antiWindup;
  tParameter<float> par_acc_Kp;
  tParameter<float> par_acc_Ki;
  tParameter<float> par_acc_antiWindup;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mTrackVelocityController(core::tFrameworkElement *parent, const std::string &name = "TrackVelocityController");
  ~mTrackVelocityController();
//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  float i_error_left, i_error_right;
  float i_error_diff;
  float i_error_acc;

  virtual void OnParameterChange();
  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
