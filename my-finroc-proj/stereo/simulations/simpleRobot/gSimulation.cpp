//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    projects/simple_robot_simulation/gSimulation.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_simple_robot/gSimulation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_simple_robot/mMainSimulation.h"
#include "projects/stereo_traversability_experiments/simulation_simple_robot/mAddNoise.h"

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
namespace simulation_simple_robot
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gSimulation> cCREATE_ACTION_FOR_G_SIMULATION("Simulation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gSimulation constructor
//----------------------------------------------------------------------
gSimulation::gSimulation(core::tFrameworkElement *parent, const std::string &name,
                         const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its sensor outputs and controller inputs)
{
  // create modules
  mMainSimulation* main_sim = new mMainSimulation(this);
  mAddNoise* add_noise = new mAddNoise(this, "AddNoise Front");

  // connect some ports
  velocity.ConnectTo(main_sim->velocity);
  angular_velocity.ConnectTo(main_sim->angular_velocity);
  main_sim->pose.ConnectTo(pose);
  main_sim->ir_distance_front.ConnectTo(add_noise->input);
  add_noise->output.ConnectTo(ir_distance_front);
}

//----------------------------------------------------------------------
// gSimulation destructor
//----------------------------------------------------------------------
gSimulation::~gSimulation()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
