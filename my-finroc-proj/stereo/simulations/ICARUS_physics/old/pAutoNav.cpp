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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/pAutoNav.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2015-03-19
 *
 * \brief Contains mAutoNav
 *
 * \b pAutoNav
 *
 * This is the part for gAutoNav module or Autonomous Navigation group.
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
//#include "projects/stereo_traversability_experiments/simulation_physics/mAutoNav.h"
//#include "projects/stereo_traversability_experiments/simulation_physics/gAutoNav.h"
#include "projects/stereo_traversability_experiments/simulation_physics/test/gAutoNav.h"

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
const std::string cPROGRAM_DESCRIPTION = "This program executes the AutoNav module/group.";
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
// CreateMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  finroc::structure::tTopLevelThreadContainer<> *main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

//  new finroc::stereo_traversability_experiments::simulation_physics::mAutoNav(main_thread);
//  new finroc::stereo_traversability_experiments::simulation_physics::gAutoNav(main_thread);
  new finroc::stereo_traversability_experiments::simulation_physics::gAutoNav(main_thread);

  main_thread->SetCycleTime(std::chrono::milliseconds(50));
}
