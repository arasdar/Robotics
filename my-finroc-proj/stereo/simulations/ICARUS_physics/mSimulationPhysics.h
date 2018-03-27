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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/mSimulationPhysics.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-18
 *
 * \brief Contains mSimulationPhysics
 *
 * \b mSimulationPhysics
 *
 * This is the simulated robot and environment using simvis3d.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__simulation_physics__test__mSimulationPhysics_h__
#define __projects__stereo_traversability_experiments__simulation_physics__test__mSimulationPhysics_h__

#include "plugins/structure/tSenseControlModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/physics_simulation/mPhysicsSimulationSimVis.h"
#include "rrlib/physics_simulation_newton/tEngineNewton.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the simulated robot and environment using simvis3d.
 */
//class mSimulationPhysics : public structure::tSenseControlModule
class mSimulationPhysics : public physics_simulation::mPhysicsSimulationSimVis
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mSimulationPhysics(core::tFrameworkElement *parent, const std::string &name = "SimulationPhysics");

  //----------------------------------------------------------------------
  // Protected methods (no fields/variables)
  //----------------------------------------------------------------------
protected:

  virtual rrlib::physics_simulation::tEngine* CreateClassicPhysicsEngine()
  {
    return new rrlib::physics_simulation::newton::tEngineNewton(core::GetFinrocFile("projects/stereo_traversability_experiments/simulation_physics/models/materials.mat"));
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mSimulationPhysics();

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Sense() override;

  virtual void Control() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
