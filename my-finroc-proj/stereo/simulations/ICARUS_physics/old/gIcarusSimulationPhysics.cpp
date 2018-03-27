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
 * \date    2012-04-13
 *
 * \brief Contains mExtendedPhysicsSimulationNewton
 *
 * \b mExtendedPhysicsSimulationNewton
 *
 * This module instantiates the physics simulation newton including a simvis3d simulation environment from the description file
 *
 */
//----------------------------------------------------------------------



//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/scheduling/tThreadContainerElement.h"
#include "libraries/vehicle_kinematics/mAckermannDriveKinematics.h"
#include "libraries/vehicle_kinematics/mDifferentialDriveKinematics.h"
#include "libraries/structure_elements/mMultiply.h"
#include "libraries/structure_elements/mVectorDemultiplexer.h"


//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/gIcarusSimulationPhysics.h"
#include "projects/stereo_traversability_experiments/simulation_physics/mSimulation.h"
#include "libraries/camera/test/mTestFrameGrabberCoin.h"
#include "rrlib/camera/tFrameGrabber.h"
//#include "projects/simulations/simulation_physics/mStViPr.h"
//#include "rrlib/mapping/definitions.h"
//#include "rrlib/canvas/tCanvas2D.h"
//#include "libraries/mapping/handlers/display/mMapToCanvas.h"
//#include "libraries/hid/mJoystick.h"

#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::laser_scanner;
using namespace finroc::physics_simulation;
using namespace finroc::simvis3d;
using namespace finroc::vehicle_kinematics;
//using namespace rrlib::mapping::handlers;
//using namespace finroc::mapping::handlers::display;


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
finroc::runtime_construction::tStandardCreateModuleAction<gIcarusSimulationPhysics> cCREATE_ACTION_FOR_G_ICARUS_SIMULATION_PHYSICS("IcarusSimulationPhysics");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gIcarusSimulation constructors
//----------------------------------------------------------------------
gIcarusSimulationPhysics::gIcarusSimulationPhysics(core::tFrameworkElement *parent, const std::string &name,
    const std::string &structure_config_file)
  : tSenseControlGroup(parent, name, structure_config_file, true /*make ports global*/)
{

  fprintf(stderr, "gIcarusSimulationPhysics constructor started.....................................................................................\n");  //::ctor >>


  /*
   * BEGIN  -------------- 3D simulation ------------------
   */

  /* physics and newton for the environment */
  //mExtendedPhysicsSimulationNewton *simulation = new mExtendedPhysicsSimulationNewton(this); //the same as mSimulation but with physics :)
  mSimulation *simulation = new mSimulation(this); //the same as mSimulation but with physics :)


  /* SUGV kinematics module */

  // Create and connect kinematics module
  vehicle_kinematics::mDifferentialDriveKinematics* dd_kinematics = new vehicle_kinematics::mDifferentialDriveKinematics(this);
  //  dd_kinematics->par_wheel_distance.Set(0.205); //attached_to="wheels" ==>  pose_offset="0.50  0.1025 0.050 -90 0 0"/> && pose_offset="0.50  -0.1025 0.050 -90 0 0"/>
  //attached_to="LUGV_wheels" ==>   pose_offset="0.50  0.2 0.050 -90 0 0"/> && pose_offset="0.50  -0.2 0.050 -90 0 0"/>
  dd_kinematics->par_wheel_distance.Set(0.4); // 2* 0.2
  desired_linear_velocity_sugv.ConnectTo(dd_kinematics->ci_velocity);
  desired_angular_velocity_sugv.ConnectTo(dd_kinematics->ci_angular_velocity);

  // Create and connect multiplication modules
  structure_elements::mMultiply* multiply = new structure_elements::mMultiply(this, "WheelVelocityToOmega");
  const double wheel_velocity_to_omega_wheel_diameter = 0.08; //radius="0.08"
  multiply->factor.Set(2.0 / wheel_velocity_to_omega_wheel_diameter);
  multiply->number_of_ports.Set(2);
  multiply->Init();
  dd_kinematics->co_velocity_left.ConnectTo(multiply->input_values[0]);
  dd_kinematics->co_velocity_right.ConnectTo(multiply->input_values[1]);

  /* LUGV kinematics module */

  // Create and connect kinematics module
  vehicle_kinematics::mDifferentialDriveKinematics* dd_kinematics_lugv = new vehicle_kinematics::mDifferentialDriveKinematics(this, "DifferentialDriveKinematics_lugv");
  dd_kinematics_lugv->par_wheel_distance.Set(2 * 0.6); //2* 0.55
  desired_linear_velocity_lugv.ConnectTo(dd_kinematics_lugv->ci_velocity);
  desired_angular_velocity_lugv.ConnectTo(dd_kinematics_lugv->ci_angular_velocity);

  // Create and connect multiplication modules
  structure_elements::mMultiply* multiply_lugv = new structure_elements::mMultiply(this, "WheelVelocityToOmega_lugv");
  const double wheel_velocity_to_omega_wheel_diameter_lugv = 0.1; //wheel raduis as it is in lugv_wheel.col and .iv
  multiply_lugv->factor.Set(2.0 / wheel_velocity_to_omega_wheel_diameter_lugv);
  multiply_lugv->number_of_ports.Set(2);
  multiply_lugv->Init();
  dd_kinematics_lugv->co_velocity_left.ConnectTo(multiply_lugv->input_values[0]);
  dd_kinematics_lugv->co_velocity_right.ConnectTo(multiply_lugv->input_values[1]);

  /*! camera capture simvis3d */
  camera::mTestFrameGrabberCoin* frame_grabber = new finroc::camera::mTestFrameGrabberCoin(this, "FrameGrabber", &simulation->shared_scene);


  /*! stereo vision processing*/
//  new mStViPr(this);
//  new scheduling::tThreadContainerElement<mStViPr>(this); //new thread added :)
  stereo_processing::mStereoProcessing* stereo_processing = new stereo_processing::mStereoProcessing(this);
//  scheduling::tThreadContainerElement<stereo_processing::mStereoProcessing>* stereo_processing = new scheduling::tThreadContainerElement<stereo_traversability_experiments::stereo_processing::mStereoProcessing>(this);
  stereo_processing->si_images.ConnectTo(frame_grabber->so_image);
//  stereo_processing->si_ugv_chassi_pose.ConnectTo(simulation->lugv_1_chassi_obj_forklift);
//ExtendedPhysicsSimulationNewton/Sensor Output/lugv_1_chassi_obj_forklift - Pose


//  /* grid map display for visualization grid map*/
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for stereo temporary");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for stereo environment");
//
//
//  /*! joystick */
//  hid::mJoystick* hdw_joystick = new hid::mJoystick(this);
//  hdw_joystick->device_name.Set("js0");
//  hdw_joystick->device_file.Set("/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick");
//
//  /*
//  *   usb-Logitech_Logitech_Cordless_RumblePad_2-joystick
//  usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick
//  */
//  //desired_linear_velocity_lugv_js_axis.Get(hdw_joystick->co_axis_values.at(4).Get * 0.1);
//  //FINROC_LOG_PRINT(DEBUG, "hdw_joystick->co_axis_values.at(4).Get() * 0.1: ", hdw_joystick->co_axis_values.at(4).Get() * 0.1);



  fprintf(stderr, "gIcarusSimulationPhysics constructor finished :).....................................................................................\n");  //::ctor >>


}//end

}
}
}

