//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    projects/petrover/gPetRover.cpp
 *
 * \author  Aras Dar
 *
 * \date    2018-05-04
 *
 */
//----------------------------------------------------------------------
#include "projects/petrover/gPetRover.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/DCMotorTestControl/mDCMotorControl.h"
#include "libraries/camera/mFrameGrabber.h"

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
namespace petrover
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gPetRover> cCREATE_ACTION_FOR_G_PETROVER("PetRover");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gPetRover constructor
//----------------------------------------------------------------------
gPetRover::gPetRover(core::tFrameworkElement *parent, const std::string &name,
                     const std::string &structure_config_file) :
  tGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
// Sensory input: getting the camera module running here.
///*finroc::DCMotorTestControl::mDCMotorControl* motor_control =*/ new finroc::DCMotorTestControl::mDCMotorControl(this);

// Motor output: getting the motor module running here.
/*finroc::camera::mFrameGrabber* frame_grabber =*/ new finroc::camera::mFrameGrabber(this);
}

//----------------------------------------------------------------------
// gPetRover destructor
//----------------------------------------------------------------------
gPetRover::~gPetRover()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
