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
/*!\file    projects/test/mRaspberryPiCameraTest.cpp
 *
 * \author  Aras Dar
 *
 * \date    2018-04-08
 *
 */
//----------------------------------------------------------------------
#include "projects/test/mRaspberryPiCameraTest.h"

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
runtime_construction::tStandardCreateModuleAction<mRaspberryPiCameraTest> cCREATE_ACTION_FOR_M_RASPBERRYPICAMERATEST("RaspberryPiCameraTest");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mRaspberryPiCameraTest constructor
//----------------------------------------------------------------------
mRaspberryPiCameraTest::mRaspberryPiCameraTest(core::tFrameworkElement *parent, const std::string &name):
  tModule(parent, name, false)//, // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{}

//----------------------------------------------------------------------
// mRaspberryPiCameraTest destructor
//----------------------------------------------------------------------
mRaspberryPiCameraTest::~mRaspberryPiCameraTest()
{}

//----------------------------------------------------------------------
// mRaspberryPiCameraTest OnStaticParameterChange
//----------------------------------------------------------------------
void mRaspberryPiCameraTest::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//	  //    As this static parameter has changed, do something with its value!
//  }
}

//----------------------------------------------------------------------
// mRaspberryPiCameraTest OnParameterChange
//----------------------------------------------------------------------
void mRaspberryPiCameraTest::OnParameterChange()
{
	//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mRaspberryPiCameraTest Update
//----------------------------------------------------------------------
void mRaspberryPiCameraTest::Update()
{
//  if (this->InputChanged())
//  {
//	//    At least one of your input ports has changed. Do something useful with its data.
//	//    However, using the .HasChanged() method on each port you can check in more detail.
//  }
  
//	//  Do something each cycle independent from changing ports.
//	camera.GetData();
//	camera.SaveData();
  
//	Publishing data via output ports
//	Data can be published via output ports using the following pattern
// (we have an output port of type double in this example):

//	data_ports::tPortDataPointer<double> buffer_to_publish = output_port.GetUnusedBuffer();
//	*buffer_to_publish = 42;
//	buffer_to_publish.SetTimestamp(timestamp); // optional
//	output_port.Publish(buffer_to_publish);

//  this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
