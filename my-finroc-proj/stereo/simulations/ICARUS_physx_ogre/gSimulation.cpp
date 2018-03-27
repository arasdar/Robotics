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
/*!\file    libraries/simulation_physx_ogre/gSimulation.cpp
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-08-05
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/gSimulation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/scheduling/tThreadContainerElement.h"

#include <chrono>

#include "libraries/simulation_physx_ogre/mSimulation.h"
#include "libraries/simulation_physx_ogre/elements/mTerrain.h"
#include "libraries/simulation_physx_ogre/elements/tTestPlane.h"
#include "libraries/simulation_physx_ogre/elements/mDamagedDownTownTile.h"
//#include "libraries/simulation_physx_ogre/elements/test/tTestWorld.h"
//#include "libraries/simulation_physx_ogre/elements/mRealisticSkySystem.h"
#include "libraries/mapping/handlers/display/mMapToCanvas.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//#include "projects/icarus/simulation_physx_ogre/gLUGV.h"
//#include "projects/icarus/simulation_physx_ogre/gSUGV.h"
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/gSUGV.h"
//#include "projects/icarus/simulation_physx_ogre/mIcarusControlInterface.h"
////#include "projects/icarus/simulation_physx_ogre/mProcessedSensorDataToMapping.h"

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
namespace simulation_physx_ogre
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
  tGroup(parent, name, structure_config_file, true, true)
{
  new finroc::simulation_physx_ogre::mSimulation(this, "Simulation");
  //new finroc::simulation_physx_ogre::mSimulation(this, "Simulation");
  RRLIB_LOG_PRINT(DEBUG, "mSimulation created");

  //new finroc::simulation_physx_ogre::elements::tTestPlane();
  //finroc::simulation_physx_ogre::elements::tFirstScene *firstScene = new finroc::simulation_physx_ogre::elements::tFirstScene();
  //new finroc::simulation_physx_ogre::elements::tTestWorld();
  new finroc::simulation_physx_ogre::elements::tTestPlane();
  //finroc::simulation_physx_ogre::elements::mDamagedDownTownTile *Damaged_world = new finroc::simulation_physx_ogre::elements::mDamagedDownTownTile(this, "Damaged World");




//  finroc::icarus::simulation_physx_ogre::mIcarusControlInterface *interface =
//    new finroc::icarus::simulation_physx_ogre::mIcarusControlInterface(this, "Interface");
//
//  interface->ci_angular_velocity.ConnectTo(this->in_angular_velocity);
//  interface->ci_velocity.ConnectTo(this->in_velocity);
//  interface->ci_desired_manipulator_joint_angles.ConnectTo(this->in_desired_manipulator_joint_angles);
//  interface->ci_desired_manipulator_joint_velocities.ConnectTo(this->in_desired_manipulator_joint_velocities);
//
//  interface->so_actual_manipulator_joint_angles.ConnectTo(this->out_actual_manipulator_joint_angles);
//  interface->so_velocity.ConnectTo(this->out_velocity);
//  interface->so_angular_velocity.ConnectTo(this->out_angular_velocity);
//  interface->so_current_pose.ConnectTo(this->out_current_pose);
//  interface->so_tcp_pose.ConnectTo(this->out_tcp_pose);
//
////  interface->GetControllerInputs().ConnectByName(this->GetInputs(),false);
////  interface->GetSensorOutputs().ConnectByName(this->GetOutputs(),false);
//
//  finroc::icarus::simulation_physx_ogre::gLUGV *lugv_group = new finroc::icarus::simulation_physx_ogre::gLUGV(this, "LUGV Group");
//
//  lugv_group->GetInputs().ConnectByName(interface->GetControllerOutputs(), false);
//  lugv_group->GetOutputs().ConnectByName(interface->GetSensorInputs(), false);


//  finroc::icarus::simulation_physx_ogre::gSUGV *sugv_group = new finroc::icarus::simulation_physx_ogre::gSUGV(this, "SUGV Group");
  new gSUGV(this, "SUGV group");

//  sugv_group->GetInputs().ConnectByName(interface->GetControllerOutputs(),false);
//  sugv_group->GetOutputs().ConnectByName(interface->GetSensorInputs(),false);


  /*sector mapping */
  //new finroc::icarus::simulation_physx_ogre::mProcessedSensorDataToMapping(this);
  //new scheduling::tThreadContainerElement<finroc::icarus::simulation_physx_ogre::mProcessedSensorDataToMapping>(this);
  //new finroc::mapping::handlers::display::mMapToCanvas<rrlib::mapping::tMapGridCartesian2DShiftable<double>, rrlib::canvas::tCanvas2D>(this); //, "grid map shiftable"
  //new scheduling::tThreadContainerElement<finroc::mapping::handlers::display::mMapToCanvas<rrlib::mapping::tMapGridCartesian2DShiftable<double>, rrlib::canvas::tCanvas2D>>(this);

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
