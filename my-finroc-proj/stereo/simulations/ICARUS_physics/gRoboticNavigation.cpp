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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/gRoboticNavigation.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-18
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/gRoboticNavigation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/gControl.h"
//#include "projects/stereo_traversability_experiments/simulation_physics/gAction.h"
#include "projects/stereo_traversability_experiments/simulation_physics/gActuation.h"
#include "projects/stereo_traversability_experiments/simulation_physics/mSimulationPhysics.h"
#include "projects/stereo_traversability_experiments/simulation_physics/gPlanning.h"
#include "projects/stereo_traversability_experiments/simulation_physics/gPerception.h"
#include "projects/stereo_traversability_experiments/simulation_physics/gSensing.h"

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
static runtime_construction::tStandardCreateModuleAction<gRoboticNavigation> cCREATE_ACTION_FOR_G_ROBOTICNAVIGATION("RoboticNavigation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gRoboticNavigation constructor
//----------------------------------------------------------------------
gRoboticNavigation::gRoboticNavigation(core::tFrameworkElement *parent, const std::string &name,
                                       const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its sensor outputs and controller inputs)
{

  mSimulationPhysics* sim = new mSimulationPhysics(this, "simulated robot & environment");

  gSensing* sensing = new gSensing(this); //, "simulated sensing"
  sim->shared_scene.AttachTo(sensing->sp_shared_scene);

  //  new gSensorDataProcessing(this);
  new gPerception(this);
//  new gMapping(this);

  new gPlanning(this);

  /*gControl* control =*/ new gControl(this);

//  /*gAction* action = */new gAction(this);
  new gActuation(this);

}

//----------------------------------------------------------------------
// gRoboticNavigation destructor
//----------------------------------------------------------------------
gRoboticNavigation::~gRoboticNavigation()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
