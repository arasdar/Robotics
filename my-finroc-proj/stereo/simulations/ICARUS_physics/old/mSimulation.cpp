//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    mSimulation.cpp
 *
 * \author  Aras
 *
 * \date    2012-10-15
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/mSimulation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/file_lookup.h"
#include "core/tRuntimeEnvironment.h"

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
runtime_construction::tStandardCreateModuleAction<mSimulation> cCREATE_ACTION_FOR_M_SIMULATION("ExtendedPhysicsSimulationNewton");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mSimulation constructors
//----------------------------------------------------------------------
mSimulation::mSimulation(core::tFrameworkElement *parent, const std::string &name)
  : mPhysicsSimulationSimVis(parent, name)
{
  scene_description_file.Set(core::GetFinrocFile("projects/stereo_traversability_experiments/simulation_physics/models/icarus_physics_simulation.descr"));
//    scene_description_file.Set(core::GetFinrocFile("projects/stereo_traversability_experiments/model/icarus_physics_simulation.descr"));
//                                              projects/stereo_traversability_experiments/simulation_physics/etc/icarus_physics_simulation.descr
//    projects/stereo_traversability_experiments/simulation_physics/etc/materials.mat
  shared_scene.AttachToOuterParameter();
}



void mSimulation::OnStaticParameterChange()
{
  mPhysicsSimulationSimVis::OnStaticParameterChange();
}

//----------------------------------------------------------------------
// mSimulation Control
//----------------------------------------------------------------------
void mSimulation::Control()
{
  mPhysicsSimulationSimVis::Control();
}

//----------------------------------------------------------------------
// mSimulation Sense
//----------------------------------------------------------------------
void mSimulation::Sense()
{
  mPhysicsSimulationSimVis::Sense();
}


}
}
}
