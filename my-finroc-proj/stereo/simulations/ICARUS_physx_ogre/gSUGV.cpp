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
/*!\file    projects/icarus/simulation_physx_ogre/gSUGV.cpp
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-09-30
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/gSUGV.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "plugins/scheduling/tThreadContainerElement.h"
#include "libraries/hid/mJoystick.h"

#include "libraries/simulation_physx_ogre/elements/sensors/mStereoCamera.h"
#include "rrlib/math/tPose3D.h"

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
using namespace rrlib::math;

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
static runtime_construction::tStandardCreateModuleAction<gSUGV> cCREATE_ACTION_FOR_G_SUGV("SUGV");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gLUGV constructor
//----------------------------------------------------------------------
gSUGV::gSUGV(core::tFrameworkElement *parent, const std::string &name,
             const std::string &structure_config_file) :
  tGroup(parent, name, structure_config_file, true, true)
{
  sugv = new mIcarusSUGV(this, "SUGV");
  sugv->par_pose.Set(rrlib::math::tPose3D(0.0, 0.0, 1.0, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0)));
  // position near entry of destroyed house
  //sugv->par_pose.Set(rrlib::math::tPose3D(36.0, -1.0, 1.0, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0)));
  //sugv->par_pose.Set(rrlib::math::tPose3D(22.0, 3.0, 1.0, 0, 0, 0));
  sugv->out_chassis_pose.ConnectTo(this->out_current_pose);


  /*scheduling::tThreadContainerElement<finroc::simulation_physx_ogre::elements::sensors::mDepthCamera> *depth =
    new scheduling::tThreadContainerElement<finroc::simulation_physx_ogre::elements::sensors::mDepthCamera>(this, "WorldDepthCamSim");
  depth->SetCycleTime(std::chrono::microseconds(33000));
  depth->par_width.Set(32);
  depth->par_height.Set(32);
  depth->par_near_clip_distance.Set(0.1);
  depth->par_far_clip_distance.Set(20);
  depth->par_enabled.Set(true);
  depth->par_debug.Set(true);
  depth->par_local_pose.Set(rrlib::math::tPose3D(2, 0, 0, 0, 0, 0));
  depth->GetCamera().SetCameraNode(lugv->GetChassisNode());*/

  /*gui_joystick = new mGUIJoystick(this, "GUI-Joystick");
  gui_joystick->out_accel.ConnectTo(sugv->in_accel);
  gui_joystick->out_left.ConnectTo(sugv->in_thrust_left);
  gui_joystick->out_right.ConnectTo(sugv->in_thrust_right);
  gui_joystick->out_left_brake.ConnectTo(sugv->in_brake_left);
  gui_joystick->out_right_brake.ConnectTo(sugv->in_brake_right);*/

//  front_scanner =
//    new finroc::simulation_physx_ogre::elements::sensors::mSickLMS511(this, "FrontLaserScannerSUGV");
//  front_scanner->SetSceneNode(sugv->GetChassisNode());
//  front_scanner->par_local_pose.Set(rrlib::math::tPose3D(2, 0, 0, 0, 0, 0));
//  front_scanner->par_debug.Set(true);
//  front_scanner->distances.ConnectTo(this->out_front_lrf);


  /*hid::mJoystick* hdw_joystick = new hid::mJoystick(this);
  hdw_joystick->device_name.Set("js0");
  hdw_joystick->device_file.Set("/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick");


  mRumblePad *rumble_pad = new mRumblePad(this);
  rumble_pad->rumble_pad_ready.ConnectTo("/Main Thread/Simulation/SUGV/Joystick/Controller Output/Joystick Initialised");
  rumble_pad->par_left_zero.Set(127);
  rumble_pad->par_right_zero.Set(128);
  rumble_pad->par_threshold.Set(5);
  rumble_pad->ci_left_axis.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Axis 1");
  rumble_pad->ci_right_axis.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Axis 3");
  rumble_pad->ci_left_brake.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 6");
  rumble_pad->ci_right_brake.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 7");
  rumble_pad->ci_arm0_positive.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 4");
  rumble_pad->ci_arm0_negative.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 5");
  rumble_pad->ci_arm1_positive.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 0");
  rumble_pad->ci_arm1_negative.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 1");
  rumble_pad->ci_arm2_positive.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 2");
  rumble_pad->ci_arm2_negative.ConnectTo("/Main Thread/Simulation/LUGV/Joystick/Controller Output/Button 3");
  rumble_pad->co_accel.ConnectTo(sugv->in_accel);
  rumble_pad->co_left.ConnectTo(sugv->in_thrust_left);
  rumble_pad->co_right.ConnectTo(sugv->in_thrust_right);
  rumble_pad->co_left_brake.ConnectTo(sugv->in_brake_left);
  rumble_pad->co_right_brake.ConnectTo(sugv->in_brake_right);
  rumble_pad->co_arm0.ConnectTo(sugv->in_arm_0_angular_vel);
  rumble_pad->co_arm1.ConnectTo(sugv->in_arm_1_angular_vel);
  rumble_pad->co_arm2.ConnectTo(sugv->in_arm_2_angular_vel);
  rumble_pad->co_arm3.ConnectTo(sugv->in_arm_3_angular_vel);*/

  scheduling::tThreadContainerElement<mTrackVelocityController> *controller =
    new scheduling::tThreadContainerElement<mTrackVelocityController>(this, "velocity controller");
  controller->SetCycleTime(std::chrono::milliseconds(16));
  controller->in_actual_velocity_left.ConnectTo(sugv->out_left_track_speed);
  controller->in_actual_velocity_right.ConnectTo(sugv->out_right_track_speed);
// controller->out_accel.ConnectTo(sugv->in_accel);
  controller->out_acc.ConnectTo(sugv->in_accel);
  controller->out_thrust_left.ConnectTo(sugv->in_thrust_left);
  controller->out_thrust_right.ConnectTo(sugv->in_thrust_right);
  controller->out_brake_left.ConnectTo(sugv->in_brake_left);
  controller->out_brake_right.ConnectTo(sugv->in_brake_right);
//  controller->in_enable.Set(true);

  kinematic = new finroc::vehicle_kinematics::mDifferentialDriveKinematics(this, "DifferentialDriveKinematics");
  kinematic->par_wheel_distance.Set(0.37);
  kinematic->co_velocity_left.ConnectTo(controller->in_velocity_left);
  kinematic->co_velocity_right.ConnectTo(controller->in_velocity_right);
  kinematic->ci_velocity.ConnectTo(this->in_velocity);
  kinematic->ci_angular_velocity.ConnectTo(this->in_angular_velocity);
  kinematic->so_velocity.ConnectTo(this->out_velocity);
  kinematic->so_angular_velocity.ConnectTo(this->out_angular_velocity);

  /*

  */

  // StereoCamera
//  scheduling::tThreadContainerElement<finroc::simulation_physx_ogre::elements::sensors::mStereoCamera> *stereo_cam =
//    //new scheduling::tThreadContainerElement<finroc::simulation_physx_ogre::elements::sensors::mStereoCamera>(this, "StereoCamera", 640, 480);
//    new scheduling::tThreadContainerElement<finroc::simulation_physx_ogre::elements::sensors::mStereoCamera>(this, "StereoCamera", 800, 600, 0.3); //parent, name, width, height, base
//  // Update-Rate 10Hz
//  stereo_cam->SetCycleTime(std::chrono::milliseconds(100));

  finroc::simulation_physx_ogre::elements::sensors::mStereoCamera *stereo_cam =
    new finroc::simulation_physx_ogre::elements::sensors::mStereoCamera(this, "StereoCamera", 640, 480, 0.3);
  stereo_cam->par_near_clip_distance.Set(0.1);
  stereo_cam->par_far_clip_distance.Set(100);
  stereo_cam->SetSceneNode(sugv->GetChassisNode());
  stereo_cam->par_local_pose.Set(rrlib::math::tPose3D(0, 0, 0.5, rrlib::math::tAngleDeg(0), rrlib::math::tAngleDeg(0), rrlib::math::tAngleDeg(0)));
  stereo_cam->par_base.Set(0.10);
  out_stereo_images.ConnectTo(stereo_cam->out_images);
}

//----------------------------------------------------------------------
// gLUGV destructor
//----------------------------------------------------------------------
gSUGV::~gSUGV()
{
  delete(front_scanner);
//  delete(gui_joystick);
  delete(sugv);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
