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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/mSimPhysRobAndEnv.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2015-03-19
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/mSimPhysRobAndEnv.h"

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
namespace simulation_physics
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mSimPhysRobAndEnv> cCREATE_ACTION_FOR_M_SIMPHYSROBANDENV("SimPhysRobAndEnv");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mSimPhysRobAndEnv constructor
//----------------------------------------------------------------------
mSimPhysRobAndEnv::mSimPhysRobAndEnv(core::tFrameworkElement *parent, const std::string &name) :
  mPhysicsSimulationSimVis(parent, name)
//  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
//  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{
  scene_description_file.Set(core::GetFinrocFile("projects/stereo_traversability_experiments/simulation_physics/models/icarus_physics_simulation.descr"));
  shared_scene.AttachToOuterParameter();
}

//----------------------------------------------------------------------
// mSimPhysRobAndEnv destructor
//----------------------------------------------------------------------
mSimPhysRobAndEnv::~mSimPhysRobAndEnv()
{}

//----------------------------------------------------------------------
// mSimPhysRobAndEnv OnStaticParameterChange
//----------------------------------------------------------------------
void mSimPhysRobAndEnv::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    As this static parameter has changed, do something with its value!
//  }
  mPhysicsSimulationSimVis::OnStaticParameterChange();
}

//----------------------------------------------------------------------
// mSimPhysRobAndEnv OnParameterChange
//----------------------------------------------------------------------
void mSimPhysRobAndEnv::OnParameterChange()
{
//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
  mPhysicsSimulationSimVis::OnParameterChange();
}

//----------------------------------------------------------------------
// mSimPhysRobAndEnv Update
//----------------------------------------------------------------------
void mSimPhysRobAndEnv::Sense()
{
  mPhysicsSimulationSimVis::Sense();
  /*
  //  if (this->InputChanged())
  //  {
  //    At least one of your input ports has changed. Do something useful with its data.
  //    However, using the .HasChanged() method on each port you can check in more detail.
  //  }
  //
  //  Do something each cycle independent from changing ports.
  //
  //  this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
  */
}

void mSimPhysRobAndEnv::Control()
{
  mPhysicsSimulationSimVis::Control();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

