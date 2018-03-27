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
/*!\file    projects/simple_robot_simulation/mMainSimulation.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_simple_robot/mMainSimulation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/scheduling/tThreadContainerThread.h"

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
using namespace finroc::data_ports;
using namespace rrlib::math;

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
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<mMainSimulation> cCREATE_ACTION_FOR_M_MAINSIMULATION("MainSimulation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mMainSimulation constructor
//----------------------------------------------------------------------
mMainSimulation::mMainSimulation(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name, false),
/*! change to 'true' to make module's ports shared (so that ports in other processes can connect to its sensor outputs and controller inputs)*/
/*! If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
  velocity(tUnit::m_s),
  ir_distance_front(tUnit::mm),
  ir_distance_rear(tUnit::mm),
  max_acceleration(0.3),
  destructive_collision_speed(0.9, tUnit::m_s),
  max_ir_sensor_distance(2000, tUnit::mm),
  current_speed(0),
  current_pose(),
  robot_counter_internal(0)
{}

//----------------------------------------------------------------------
// mMainSimulation destructor
//----------------------------------------------------------------------
mMainSimulation::~mMainSimulation()
{}

//----------------------------------------------------------------------
// mMainSimulation OnStaticParameterChange
//----------------------------------------------------------------------
void mMainSimulation::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    //As this static parameter has changed, do something with its value!
//  }
}

//----------------------------------------------------------------------
// mMainSimulation OnParameterChange
//----------------------------------------------------------------------
void mMainSimulation::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mMainSimulation Sense
//----------------------------------------------------------------------
void mMainSimulation::Sense()
{
//  if (this->SensorInputChanged())
//  {
//    /*
//     *     At least one of your sensor input ports has changed. Do something useful with its data.
//     *     However, using the .HasChanged() method on each port you can check in more detail.
//     */
//  }

  /*
   * Do something each cycle independent from changing ports.
   */
  rrlib::time::tDuration cycle_time = scheduling::tThreadContainerThread::CurrentThread()->GetCycleTime(); // cycle time of thread container
  double delta_t = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_time).count() / 1000.0; // cycle time in seconds
  rrlib::time::tTimestamp now = rrlib::time::Now();

  // Calculate new speed
  double desired_speed = velocity.Get();
  double new_speed = 0;
  if (desired_speed >= 0)
  {
    new_speed = current_speed < 0 ? 0 : ((desired_speed < current_speed) ? desired_speed :
                                         std::min<double>(velocity.Get(), current_speed + max_acceleration.Get() * delta_t));
  }
  else
  {
    new_speed = current_speed > 0 ? 0 : ((desired_speed > current_speed) ? desired_speed :
                                         std::max<double>(velocity.Get(), current_speed - max_acceleration.Get() * delta_t));
  }
  double avg_speed = (current_speed + new_speed) / 2;
  current_speed = new_speed;

  // Calculate new orientation
  double new_direction = current_pose.Yaw() + angular_velocity.Get() * delta_t;
  double avg_direction = (current_pose.Yaw() + new_direction) / 2;

  // Calculate new coordinates
  double s = avg_speed * delta_t;
  current_pose.Set(current_pose.X() + s * cos(avg_direction), current_pose.Y() + s * sin(avg_direction), new_direction);

  // Did we collide with the wall?
  tPose2D back_left = current_pose;
  back_left.ApplyRelativePoseTransformation(tPose2D(-0.2, 0.2));
  tPose2D back_right = current_pose;
  back_right.ApplyRelativePoseTransformation(tPose2D(-0.2, -0.2));
  tPose2D front_center = current_pose;
  front_center.ApplyRelativePoseTransformation(tPose2D(0.1, 0));
  double max_x = std::max(back_left.X(), std::max(back_right.X(), front_center.X() + 0.2));
  if (max_x > 2)
  {
    last_collision_pose.Publish(current_pose, now);
    if (fabs(avg_speed) < fabs(destructive_collision_speed.Get()))
    {
      current_pose.SetPosition(current_pose.X() - (max_x - 2), current_pose.Y());
      current_speed = 0;
      FINROC_LOG_PRINTF(WARNING, "Robot collided with wall at a speed of %f m/s.", avg_speed);
    }
    else
    {
      robot_counter_internal++;
      robot_counter.Publish(robot_counter_internal);
      current_pose.Set(0, 0, 0);
      current_speed = 0;
      FINROC_LOG_PRINTF(ERROR, "Robot crashed at a speed of %f m/s and was destroyed. Respawning. Destroyed Robots: %d.", avg_speed, robot_counter_internal);
    }
  }
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "New robot position: ", current_pose);

  // Calculate sensor values
  tPose2D robot_in_2m = current_pose.Translated(tVec2d(2, 0).Rotated(current_pose.Yaw()));
  tPose2D robot_2m_back = current_pose.Translated(tVec2d(-2, 0).Rotated(current_pose.Yaw()));
  double front_distance = max_ir_sensor_distance.Get();
  double rear_distance = max_ir_sensor_distance.Get();
  double dx = fabs(robot_in_2m.X() - current_pose.X());
  if (robot_in_2m.X() > 2)
  {
    front_distance = (2 * (2 - current_pose.X()) / dx) * 1000.0;
  }
  if (robot_2m_back.X() > 2)
  {
    rear_distance = (2 * (2 - current_pose.X()) / dx) * 1000.0;
  }

  /*! this->so_signal_1.Publish(some meaningful value); can be used to publish data via your sensor output ports.*/
  // publish updated values
  pose.Publish(current_pose, now);
  ir_distance_front.Publish(front_distance, now);
  ir_distance_rear.Publish(rear_distance, now);
}

//----------------------------------------------------------------------
// mMainSimulation Control
//----------------------------------------------------------------------
void mMainSimulation::Control()
{
//  if (this->ControllerInputChanged())
//  {
//    /*
//     *     At least one of your controller input ports has changed. Do something useful with its data.
//     *     However, using the .HasChanged() method on each port you can check in more detail.
//     */
//  }

  //Do something each cycle independent from changing ports.

  //this->co_signal_1.Publish(some meaningful value); can be used to publish data via your sensor output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
