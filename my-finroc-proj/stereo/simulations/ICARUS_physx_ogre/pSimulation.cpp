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
/*!\file    libraries/simulation_physx_ogre/pSimulation.cpp
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-08-03
 *
 * \brief Contains mSimulation
 *
 * \b pSimulation
 *
 * This part is used for testing the simulation.
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <chrono>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//#include "projects/simulation/mSimulation.h"
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/gSimulation.h"
//#include "projects/icarus/control/manipulator/gManipulatorControl.h"
//#include "projects/icarus/control/joystick/gJoystickControl.h"


//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::stereo_traversability_experiments::simulation_physx_ogre;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the Simulation module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// InitMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  finroc::structure::tTopLevelThreadContainer<> *main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);
//  new finroc::simulation::mSimulation(main_thread);
  gSimulation *simulation = new gSimulation(main_thread);
//  finroc::icarus::control::manipulator::gManipulatorControl *manipulator_control = new finroc::icarus::control::manipulator::gManipulatorControl(main_thread);
//  finroc::icarus::control::joystick::gJoystickControl *joystick = new finroc::icarus::control::joystick::gJoystickControl(main_thread);

//  simulation->in_desired_manipulator_joint_angles.ConnectTo(manipulator_control->co_desired_manipulator_joint_angles);
//  simulation->in_desired_manipulator_joint_velocities.ConnectTo(manipulator_control->co_desired_manipulator_joint_velocities);
//  simulation->out_actual_manipulator_joint_angles.ConnectTo(manipulator_control->si_actual_manipulator_joint_angles);

//  manipulator_control->par_dh_parameter_vec.SetConfigEntry("control/LUGV manipulator/DH Parameters LUGV Simulation");

//  joystick->par_joystick_device_file.SetConfigEntry("control/joystick/joystick device file sugv");
//  joystick->co_desired_manipulator_joystick_set.ConnectTo(manipulator_control->ci_desired_manipulator_joystick_set);
//  manipulator_control->ci_manipulator_joystick_control_select.ConnectTo(joystick->co_manipulator_joystick_control_select);
//  manipulator_control->ci_manipulator_enable.ConnectTo(joystick->co_manipulator_enable);
//  joystick->co_desired_velocity.ConnectTo(simulation->in_velocity);
//  joystick->co_desired_angular_velocity.ConnectTo(simulation->in_angular_velocity);

  main_thread->SetCycleTime(std::chrono::milliseconds(10));
}
