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
/*!\file    projects/test/mRaspberryPiCamera.cpp
 *
 * \author  Aras Dar
 *
 * \date    2018-04-09
 *
 */
//----------------------------------------------------------------------
#include "projects/test/mRaspberryPiCamera.h"

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
namespace test
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<mRaspberryPiCamera> cCREATE_ACTION_FOR_M_RASPBERRYPICAMERA("RaspberryPiCamera");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mRaspberryPiCamera constructor
//----------------------------------------------------------------------
mRaspberryPiCamera::mRaspberryPiCamera(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name, false)//,
	//	change to 'true' to make module's ports shared (so that ports in other processes can connect to its sensor outputs and controller inputs)
  //  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{}

//----------------------------------------------------------------------
// mRaspberryPiCamera destructor
//----------------------------------------------------------------------
mRaspberryPiCamera::~mRaspberryPiCamera()
{}

//----------------------------------------------------------------------
// mRaspberryPiCamera OnStaticParameterChange
//----------------------------------------------------------------------
void mRaspberryPiCamera::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    As this static parameter has changed, do something with its value!
//  }
}

//----------------------------------------------------------------------
// mRaspberryPiCamera OnParameterChange
//----------------------------------------------------------------------
void mRaspberryPiCamera::OnParameterChange()
{
//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mRaspberryPiCamera Sense
//----------------------------------------------------------------------
void mRaspberryPiCamera::Sense()
{
//  if (this->SensorInputChanged())
//  {
//    //At least one of your sensor input ports has changed. Do something useful with its data.
//    //However, using the .HasChanged() method on each port you can check in more detail.
//  }
  
	//  Do something each cycle independent from changing ports.
	camera.GetData();
	camera.SaveData();

//  this->so_signal_1.Publish(some meaningful value); can be used to publish data via your sensor output ports.
}

//----------------------------------------------------------------------
// mRaspberryPiCamera Control
//----------------------------------------------------------------------
void mRaspberryPiCamera::Control()
{
//  if (this->ControllerInputChanged())
//  {
////    At least one of your controller input ports has changed. Do something useful with its data.
////    However, using the .HasChanged() method on each port you can check in more detail.
//  }
  
//  Do something each cycle independent from changing ports.

//  this->co_signal_1.Publish(some meaningful value); can be used to publish data via your sensor output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
