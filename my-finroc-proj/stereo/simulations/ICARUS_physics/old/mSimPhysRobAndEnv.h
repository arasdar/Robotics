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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/mSimPhysRobAndEnv.h
 *
 * \author  Aras Dargazany
 *
 * \date    2015-03-19
 *
 * \brief Contains mSimPhysRobAndEnv
 *
 * \b mSimPhysRobAndEnv
 *
 * This is the module for simulation of robot and its environment using phyiscs engine as well.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__simulation_physics__test__mSimPhysRobAndEnv_h__
#define __projects__stereo_traversability_experiments__simulation_physics__test__mSimPhysRobAndEnv_h__

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
 * This is the module for simulation of robot and its environment using phyiscs engine as well.
 */
class mSimPhysRobAndEnv : public physics_simulation::mPhysicsSimulationSimVis
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  tStaticParameter<double> static_parameter_1;   Example for a static parameter. Replace or delete it!
//
//  tParameter<double> par_parameter_1;   Example for a runtime parameter named "Parameter 1". Replace or delete it!
//
//  tInput<double> in_signal_1;   Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tInput<double> in_signal_2;
//
//  tOutput<double> out_signal_1;   Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tOutput<double> out_signal_2;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mSimPhysRobAndEnv(core::tFrameworkElement *parent, const std::string &name = "SimPhysRobAndEnv");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  virtual rrlib::physics_simulation::tEngine* CreateClassicPhysicsEngine()
  {
    return new rrlib::physics_simulation::newton::tEngineNewton(core::GetFinrocFile("projects/stereo_traversability_experiments/simulation_physics/etc/material.mat"));
  }

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mSimPhysRobAndEnv();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!

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
