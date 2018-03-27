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
/*!\file    projects/simple_robot_simulation/pSimulation.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 * \brief Contains mSimulation
 *
 * \b pSimulation
 *
 * this is a part to create a binary executable.
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
//#include "projects/simple_robot_simulation/mSimulation.h"
#include "projects/stereo_traversability_experiments/simulation_simple_robot/gSimulation.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the Simulation module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
const std::string cMAIN_THREAD_CONTAINER_NAME = "Main Thread";
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
//void InitMainGroup(finroc::structure::tThreadContainer *main_thread, const std::vector<std::string> &remaining_arguments)
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{

  finroc::structure::tTopLevelThreadContainer<> *main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);


//  new finroc::simple_robot_simulation::mSimulation(main_thread);
//  new finroc::simple_robot_simulation::gSimulation(main_thread);
  new finroc::stereo_traversability_experiments::simulation_simple_robot::gSimulation(main_thread);
//  new finroc::icarus::simulation::simple_robot_simulation::gSimulation(main_thread);

//  main_thread->SetCycleTime(std::chrono::milliseconds(50)); //original
  main_thread->SetCycleTime(40);
}
