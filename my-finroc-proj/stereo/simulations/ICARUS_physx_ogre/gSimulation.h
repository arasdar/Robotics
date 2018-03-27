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
/*!\file    libraries/simulation_physx_ogre/gSimulation.h
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-08-05
 *
 * \brief Contains gSimulation
 *
 * \b gSimulation
 *
 * This group encapsulates all modules required for simulating robots.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__simulation_physx_ogre__gSimulation_h__
#define __projects__stereo_traversability_experiments__simulation_physx_ogre__gSimulation_h__

#include "plugins/structure/tGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//#include "projects/icarus/control/hardware_abstraction/manipulator_control/manipulator_declarations.h"
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tPose3D.h"
#include "rrlib/coviroa/tImage.h"
#include "rrlib/math/tVector.h"
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This group encapsulates all modules required for simulating robots.
 */
class gSimulation : public structure::tGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tInput<float> in_velocity;
  tInput<float> in_angular_velocity;

  tOutput<float> out_velocity;
  tOutput<float> out_angular_velocity;
  tOutput<rrlib::math::tPose3D> out_current_pose;
  tOutput<rrlib::math::tPose3D> out_tcp_pose;



//----------------------------------------------------------------------
// Manipulator Ports
//----------------------------------------------------------------------

//  tInput<finroc::icarus::control::hardware_abstraction::manipulator_control::tManipulatorControlSetType> in_desired_manipulator_joint_angles;
//  tInput<finroc::icarus::control::hardware_abstraction::manipulator_control::tManipulatorVelocityType> in_desired_manipulator_joint_velocities;
//
//  tOutput<finroc::icarus::control::hardware_abstraction::manipulator_control::tManipulatorControlSetType> out_actual_manipulator_joint_angles;
//  // tOutput<finroc::icarus::control::hardware_abstraction::manipulator_control::tManipulatorVelocityType>   out_actual_manipulator_joint_velocities;


//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gSimulation(core::tFrameworkElement *parent, const std::string &name = "Simulation",
              const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~gSimulation();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
