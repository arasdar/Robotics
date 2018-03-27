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
/*!\file    projects/icarus/simulation_physx_ogre/mTrackVelocityController.cpp
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-11-28
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/mTrackVelocityController.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"
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
namespace simulation_physx_ogre
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mTrackVelocityController> cCREATE_ACTION_FOR_M_TRACKVELOCITYCONTROLLER("TrackVelocityController");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mTrackVelocityController constructor
//----------------------------------------------------------------------
mTrackVelocityController::mTrackVelocityController(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  i_error_left(0),
  i_error_right(0),
  i_error_diff(0),
  i_error_acc(0),
  par_diff_Kp(0.05),
  par_diff_Ki(0.007),
  par_diff_antiWindup(100),
  par_acc_Kp(0.05),
  par_acc_Ki(0.004),
  par_acc_antiWindup(80)
{}

//----------------------------------------------------------------------
// mTrackVelocityController destructor
//----------------------------------------------------------------------
mTrackVelocityController::~mTrackVelocityController()
{}

//----------------------------------------------------------------------
// mTrackVelocityController OnParameterChange
//----------------------------------------------------------------------
void mTrackVelocityController::OnParameterChange()
{
}

//----------------------------------------------------------------------
// mTrackVelocityController Update
//----------------------------------------------------------------------
void mTrackVelocityController::Update()
{
  float error_l = in_velocity_left.Get() - in_actual_velocity_left.Get();
  float error_r = in_velocity_right.Get() - in_actual_velocity_right.Get();

  out_error_left.Publish(error_l);
  out_error_right.Publish(error_r);

  if (rrlib::math::IsEqual(in_velocity_left.Get(), 0.0, 0.0001) && rrlib::math::IsEqual(in_velocity_right.Get(), 0.0, 0.0001))
  {
    out_brake_left.Publish(1.0);
    out_brake_right.Publish(1.0);
    out_acc.Publish(0.0);
    // since the error is near zero reset integral counter
    i_error_left = 0;
    i_error_right = 0;
    i_error_diff = 0;
    i_error_acc = 0;
  }
  else
  {

    i_error_left += error_l / 10.0;
    if (i_error_left > par_diff_antiWindup.Get())
      i_error_left = par_diff_antiWindup.Get();
    else if (i_error_left < -par_diff_antiWindup.Get())
      i_error_left = -par_diff_antiWindup.Get();

    i_error_right += error_r / 10.0;
    if (i_error_right > par_diff_antiWindup.Get())
      i_error_right = par_diff_antiWindup.Get();
    else if (i_error_right < -par_diff_antiWindup.Get())
      i_error_right = -par_diff_antiWindup.Get();

    out_i_right.Publish(i_error_right);
    out_i_left.Publish(i_error_left);

    float tl = (par_diff_Kp.Get() * error_l +
                par_diff_Ki.Get() * i_error_left);
    float tr = (par_diff_Kp.Get() * error_r +
                par_diff_Ki.Get() * i_error_right);

    if ((in_velocity_left.Get() + in_velocity_right.Get()) > 0.0)
    {
      tl += 0.1;
      tr += 0.1;
    }
    else
    {
      tl -= 0.1;
      tr -= 0.1;
    }

    /*if(!rrlib::math::IsEqual(tl,0.0,0.0001)||!rrlib::math::IsEqual(tr,0.0,0.0001)) {
    tl /= std::fabs(tl)+std::fabs(tr);
    tr /= std::fabs(tl)+std::fabs(tr);
    }*/

    // outer control loop concerning the overall engine torque
    float error_acc = std::fabs(in_velocity_left.Get()) - std::fabs(in_actual_velocity_left.Get()) +
                      std::fabs(in_velocity_right.Get()) - std::fabs(in_actual_velocity_right.Get());
    if (std::fabs(i_error_acc) < par_acc_antiWindup.Get())
      i_error_acc += error_acc;

    out_acc_i.Publish(i_error_acc);
    float acc = par_acc_Kp.Get() * error_acc + par_acc_Ki.Get() * i_error_acc;
    if (acc > 1.0)
      acc = 1.0;
    else if (acc < -1.0)
      acc = -1.0;

    out_acc.Publish(std::fabs(acc));
    out_thrust_left.Publish(tl > 1.0 ? 1.0 : (tl < -1.0 ? -1.0 : tl));
    out_thrust_right.Publish(tr > 1.0 ? 1.0 : (tr < -1.0 ? -1.0 : tr));
    out_brake_left.Publish(0.0);
    out_brake_right.Publish(0.0);
  }

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
