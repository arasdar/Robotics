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
/*!\file    projects/simple_robot_simulation/mMainSimulation.h
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 * \brief Contains mMainSimulation
 *
 * \b mMainSimulation
 *
 * this module simulates a differential drive robot and two distance sensors.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__simple_robot_simulation__mMainSimulation_h__
#define __projects__simple_robot_simulation__mMainSimulation_h__

#include "plugins/structure/tSenseControlModule.h"

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
 * this module simulates a differential drive robot and two distance sensors.
 */
class mMainSimulation : public structure::tSenseControlModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  /*
//   * example ports
//   */
//  tStaticParameter<double> static_parameter_1;   Example for a static parameter. Replace or delete it!
//
//  tParameter<double> parameter_1;   Example for a runtime parameter. Replace or delete it!
//
//  tSensorInput<double> si_signal_1;   Example for sensor input ports. Replace or delete them!
//  tSensorInput<double> si_signal_2;
//
//  tSensorOutput<double> so_signal_1;   Examples for sensor output ports. Replace or delete them!
//
//  tControllerInput<double> ci_signal_1;   Example for controller input ports. Replace or delete them!
//  tControllerInput<double> ci_signal_2;
//
//  tControllerOutput<double> co_signal_1;   Examples for controller output ports. Replace or delete them!

  /*! Desired velocity (in m/s) */
  tControllerInput<double> velocity;

  /*! Desired angular velocity */
  tControllerInput<double> angular_velocity;

  /*! Position of our robot in the world coordinate system */
  tSensorOutput<rrlib::math::tPose2D> pose;

  /*! Simulated distance sensor values to the front and to the rear */
  tSensorOutput<double> ir_distance_front, ir_distance_rear;

  /*! Pose of last collision */
  tSensorOutput<rrlib::math::tPose2D> last_collision_pose;

  /*! Counts the number of spawned robots */
  tSensorOutput<int> robot_counter;

  /*! Maximum acceleration of robot - in m/s² */
  tParameter<double> max_acceleration;

  /*! If the robot hits the wall with more than this speed, it is destroyed */
  tParameter<double> destructive_collision_speed;

  /*! Maximum range of IR sensors */
  tParameter<double> max_ir_sensor_distance;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mMainSimulation(core::tFrameworkElement *parent, const std::string &name = "MainSimulation");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*
   * Here is the right place for your variables. Replace this line by your declarations!
   */

  /*! Robot's current speed */
  double current_speed;

  /*! Robot's current position and orientation */
  rrlib::math::tPose2D current_pose;

  /*! Counts the number of spawned robots (internal) */
  uint robot_counter_internal;


  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mMainSimulation();

  virtual void OnStaticParameterChange();   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Sense();

  virtual void Control();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
