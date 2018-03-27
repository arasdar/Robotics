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
/*!\file    projects/icarus/simulation/mExtendedPhysicsSimulationNewton.h
 *
 * \icarus  Aras Dargazany
 *
 * \date    2012-05-13
 *
 * \brief Contains mExtendedPhysicsSimulationNewton
 *
 * \b mExtendedPhysicsSimulationNewton
 *
 * This module instantiates the physics simulation newton including a simvis3d simulation environment from the description file
 *
 */
//----------------------------------------------------------------------

#ifndef _icarus__gIcarusSimulationPhysics_h_
#define _icarus__gIcarusSimulationPhysics_h_


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <vector>
#include "plugins/structure/tSenseControlGroup.h"
//#include "plugins/runtime_construction/tFinstructableGroup.h"
#include "plugins/scheduling/tThreadContainerElement.h"
#include "libraries/laser_scanner/simulation/mLaserScannerCoin.h"
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tVector.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------


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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module instantiates the physics simulation newton including a simvis3d simulation environment from the descr file
 */
class gIcarusSimulationPhysics : public structure::tSenseControlGroup
{
  static finroc::runtime_construction::tStandardCreateModuleAction<gIcarusSimulationPhysics> cCREATE_ACTION;

  //----------------------------------------------------------------------
  // Ports (These are the only variables that may be declared public)
  //----------------------------------------------------------------------
public:

  /*
   * SUGV
   */
  tControllerInput<double> desired_linear_velocity_sugv;
  tControllerInput<double> desired_angular_velocity_sugv;
//  tControllerInput<double> ci_angular_velocity_sugv, ci_velocity_sugv, ci_enable_motors_sugv; //double added by me


//  /*! Outputs for physics simulation */
//  tControllerOutput<double> desired_left_wheel_omega_sugv, desired_right_wheel_omega_sugv; //, desired_fork_pos;

  /*
   * LUGV
   */
  tControllerInput<double> desired_linear_velocity_lugv;
  tControllerInput<double> desired_angular_velocity_lugv;
//  tControllerInput<double> ci_angular_velocity_lugv, ci_velocity_lugv, ci_enable_motors_lugv; //double added by me



//  /*! Outputs for physics simulation */
//  tControllerOutput<double> desired_left_wheel_omega_lugv, desired_right_wheel_omega_lugv;


  /*! JOYSTICK VALUE FOR AXIS 4 AND 5*/
//  tControllerOutput<double> desired_linear_velocity_lugv_js_axis;
//  tControllerOutput<double> desired_angular_velocity_lugv_js_axis;

  //----------------------------------------------------------------------
  // Public methods and typedefs (no fields/variables)
  //----------------------------------------------------------------------
public:

  gIcarusSimulationPhysics(core::tFrameworkElement *parent, const std::string &name = "IcarusSimulationPhysics",
                           const std::string &structure_config_file = __FILE__".xml");


  //----------------------------------------------------------------------
  // Protected methods (no fields/variables)
  //----------------------------------------------------------------------
protected:

  //----------------------------------------------------------------------
  // Private fields and methods
  //----------------------------------------------------------------------
private:
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
