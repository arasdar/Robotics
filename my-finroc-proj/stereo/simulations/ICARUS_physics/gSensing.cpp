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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/gSensing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-18
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/gSensing.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/camera/test/mTestFrameGrabberCoin.h"
#include "libraries/localization/mOdometry.h"

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
namespace simulation_physics
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gSensing> cCREATE_ACTION_FOR_G_SENSING("Sensing");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gSensing constructor
//----------------------------------------------------------------------
gSensing::gSensing(core::tFrameworkElement *parent, const std::string &name,
                   const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
//  camera::mTestFrameGrabberCoin* frame_grabber =
  new camera::mTestFrameGrabberCoin(this, "stereo camera", &sp_shared_scene);
//   frame_grabber->so_image.

//  new localization::mOdometry(this, "OdometrySensors"); //IMU &
}

//----------------------------------------------------------------------
// gSensing destructor
//----------------------------------------------------------------------
gSensing::~gSensing()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
