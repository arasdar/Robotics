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
/*!\file    projects/simple_robot_simulation/gSimulation.h
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 * \brief Contains gSimulation
 *
 * \b gSimulation
 *
 * explaination
 * ..........
 * this is the group which realizes a simplae simulation for a differential-driven robot.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__simple_robot_simulation__gSimulation_h__
#define __projects__simple_robot_simulation__gSimulation_h__

#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/math/tPose2D.h"

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
namespace simulation_simple_robot
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * explaination
 * ..........
 * this is the group which realizes a simple simulation for a differential-driven robot.
 */
class gSimulation : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  tControllerInput<double> ci_signal_1;

  /*! Desired velocity (in m/s) */
  tControllerInput<double> velocity;

  /*! Desired angular velocity */
  tControllerInput<double> angular_velocity;

  /*! Position of our robot in the world coordinate system */
  tSensorOutput<rrlib::math::tPose2D> pose;

  /*! Simulated distance sensor values to the front and to the rear */
  tSensorOutput<double> ir_distance_front, ir_distance_rear;

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
