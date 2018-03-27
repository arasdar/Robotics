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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/gAction.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-18
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/gAction.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/structure_elements/mMultiply.h"

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
namespace test
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gAction> cCREATE_ACTION_FOR_G_ACTION("Action");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gAction constructor
//----------------------------------------------------------------------
gAction::gAction(core::tFrameworkElement *parent, const std::string &name,
                 const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{

  //converting the differential driving output left and right velocity to omega for action commmand
  //this part is usually done using FPGA or microprocessor to command the wheel motor to drive and move for locomotion
  structure_elements::mMultiply* multiply = new structure_elements::mMultiply(this, "VelocityToOmega");
  const float wheel_velocity_to_omega_wheel_diameter = 0.1;
  multiply->factor.Set(2.0 / wheel_velocity_to_omega_wheel_diameter);
  multiply->number_of_ports.Set(2);
  multiply->Init();

}

//----------------------------------------------------------------------
// gAction destructor
//----------------------------------------------------------------------
gAction::~gAction()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
