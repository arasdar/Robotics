//
// You received this file as part of Finroc
// A framework for integrated robot control
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
/*!\file    gIcarusSimulation.cpp
 *
 * \author  Aras Dargazany
 * \date    2012-05-16
 *
 */
//----------------------------------------------------------------------



//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/parameters/tConfigFile.h"
#include "plugins/scheduling/tThreadContainerElement.h"
#include "libraries/vehicle_kinematics/mAckermannDriveKinematics.h"
#include "libraries/vehicle_kinematics/mDifferentialDriveKinematics.h"
#include "libraries/vehicle_kinematics/mDifferentialDriveSimulation.h"
#include "libraries/vehicle_kinematics/mTicksToVelocity.h"
#include "libraries/structure_elements/mMultiply.h"
#include "libraries/structure_elements/mVectorDemultiplexer.h"
#include "libraries/structure_elements/mLambdaTypeConverter.h"
#include "libraries/localization/mOdometry.h"
#include "libraries/simvis3d/mSimulation.h"   //sensors simulation
#include "libraries/simvis3d/mVisualization.h"   //sensors simulation
#include "libraries/simvis3d/utils/mPoseWrapper.h"
#include "libraries/simvis3d/utils/mLineSetWrapper.h"
#include "libraries/simvis3d/utils/mRotationWrapper.h"
#include "libraries/laser_scanner/mActuatedLaserScanner.h"
#include "libraries/laser_scanner/simulation/mTimeStepCounter.h"
#include "libraries/laser_scanner/simulation/mLaserTilterSimulator.h"
#include "libraries/laser_scanner/simulation/mLaserTilterControl.h"
#include "libraries/laser_scanner/mSimulateDistanceSensor.h"
#include "libraries/distance_data/mAppendRobotPoseToDistanceDataSimple.h"
#include "libraries/camera/test/mTestFrameGrabberCoin.h"
#include "rrlib/camera/tFrameGrabber.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/time/time.h"
//#include "libraries/sensor_data_representation/tests/test_project_2d/definitions/tTestGridMapDefinitions.h"  //Sensor Data Representation
//#include "libraries/sensor_data_representation/handlers/display/mDisplayGridMap2D.h"
//#include "libraries/sensor_data_representation/tests/test_project_2d/mTestGridMap2D.h" //was working before!!


//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation/gIcarusSimulation.h"
#include "projects/stereo_traversability_experiments/simulation/mCalculateDistanceSensorLines.h"
#include "projects/stereo_traversability_experiments/simulation/mProcessDistanceData.h"
//#include "projects/icarus/simulation/mStViPr.h" //libs pcl opencv vtk
//#include "projects/icarus/simulation/mProcessedSensorDataToMapping.h"
#include "libraries/mapping/handlers/display/mMapToCanvas.h"



//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>
#include <vector>
#include <string>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::laser_scanner;
using namespace finroc::simvis3d;
using namespace finroc::vehicle_kinematics;
using namespace finroc::camera;
using namespace finroc::localization;
using namespace finroc::distance_data;
using namespace rrlib::simvis3d;
using namespace rrlib::camera;
using namespace rrlib::coviroa;
using namespace std;
using namespace rrlib::mapping::handlers;
using namespace finroc::mapping::handlers::display;

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
finroc::runtime_construction::tStandardCreateModuleAction<gIcarusSimulation> cCREATE_ACTION_FOR_G_ICARUS_SIMULATION("IcarusSimulation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gIcarusSimulation constructors
//----------------------------------------------------------------------
gIcarusSimulation::gIcarusSimulation(core::tFrameworkElement *parent, const std::string &name,
                                     const std::string &structure_config_file):
  tSenseControlGroup(parent, name, structure_config_file, true /*make ports global*/)
{


  fprintf(stderr, "gIcarusSimulation started.....................................................................................\n");  //::ctor >>


  /*
   * BEGIN  -------------- 3D simulation -----------------------------------------------------------------------------
   */
  mSimulation *simulation = new mSimulation(this, "Simulation");

  mPoseWrapper *sim_robot_pose_wrapper = new mPoseWrapper(this, "Robot Pose");
  sim_robot_pose_wrapper->pose_input_mode.Set(mPoseWrapper::eSI); //.SetValue(mPoseWrapper::eSI); ==> old finroc
  sim_robot_pose_wrapper->insertion_point_name.Set("ROBOT"); //.Set("ROBOT"); //.SetValue("ROBOT"); ==> old finroc
  sim_robot_pose_wrapper->element_name.Set("pose"); //.SetValue("pose"); ==> old finroc
  sim_robot_pose_wrapper->ConnectTo(simulation->parameter_value_bb, simulation->element_descriptor_bb);
  sim_robot_pose_wrapper->co_scene_parameter_update_request.ConnectTo(simulation->ci_scene_parameter_update_request);
  sim_robot_pose_wrapper->co_time_stamp.ConnectTo(simulation->ci_simulation_time);


  /*-------------- differential drive simulation ------------------*/
  mDifferentialDriveSimulation *differential_drive_simulation = new mDifferentialDriveSimulation(this);
  differential_drive_simulation->so_encoder_time_stamp.ConnectTo(sim_robot_pose_wrapper->si_time_stamp);
  differential_drive_simulation->par_max_velocity.SetConfigEntry("max. velocity");
  differential_drive_simulation->par_max_acceleration.SetConfigEntry("max. acceleration");
  differential_drive_simulation->par_max_deceleration.SetConfigEntry("max. deceleration");
  differential_drive_simulation->par_max_encoder_ticks.SetConfigEntry("max. encoder ticks");
  differential_drive_simulation->par_encoder_ticks_left.SetConfigEntry("encoder ticks per meter");
  differential_drive_simulation->par_encoder_ticks_right.SetConfigEntry("encoder ticks per meter");
  differential_drive_simulation->ci_enable_left.ConnectTo(this->ci_enable_motors);
  differential_drive_simulation->ci_enable_right.ConnectTo(this->ci_enable_motors);

  mDifferentialDriveKinematics * wheel_velocity_to_robot_velocity = new mDifferentialDriveKinematics(this);
  wheel_velocity_to_robot_velocity->co_velocity_left.ConnectTo(differential_drive_simulation->ci_velocity_left);
  wheel_velocity_to_robot_velocity->co_velocity_right.ConnectTo(differential_drive_simulation->ci_velocity_right);
  wheel_velocity_to_robot_velocity->par_wheel_distance.SetConfigEntry("wheel distance (m)");
  wheel_velocity_to_robot_velocity->ci_angular_velocity.ConnectTo(this->ci_angular_velocity);
  wheel_velocity_to_robot_velocity->ci_velocity.ConnectTo(this->ci_velocity);

  mOdometry * robot_velocity_to_pose = new mOdometry(this);
  robot_velocity_to_pose->so_pose.ConnectTo(sim_robot_pose_wrapper->si_current_pose);
  wheel_velocity_to_robot_velocity->so_velocity.ConnectTo(robot_velocity_to_pose->si_velocity);
  wheel_velocity_to_robot_velocity->so_angular_velocity.ConnectTo(robot_velocity_to_pose->si_angular_velocity);
  robot_velocity_to_pose->so_pose.ConnectTo(this->so_pose);

  /*! right and left */
  mTicksToVelocity * encoder_ticks_to_left_wheel_velocity = new mTicksToVelocity(this, "Encoder Ticks To Left Wheel Velocity"); //65535, 7500.0, 0.0,
  differential_drive_simulation->so_encoder_ticks_left.ConnectTo(encoder_ticks_to_left_wheel_velocity->si_encoder_ticks);
  differential_drive_simulation->so_encoder_time_stamp.ConnectTo(encoder_ticks_to_left_wheel_velocity->si_encoder_time_stamp);
  encoder_ticks_to_left_wheel_velocity->so_wheel_velocity.ConnectTo(wheel_velocity_to_robot_velocity->si_velocity_left);
  encoder_ticks_to_left_wheel_velocity->par_max_encoder_ticks.SetConfigEntry("max. encoder ticks");
  encoder_ticks_to_left_wheel_velocity->par_encoder_ticks_per_m.SetConfigEntry("encoder ticks per meter");
  encoder_ticks_to_left_wheel_velocity->par_encoder_ticks_per_rad.SetConfigEntry("encoder ticks per radian");

  mTicksToVelocity * encoder_ticks_to_right_wheel_velocity = new mTicksToVelocity(this, "Encoder Ticks To Right Wheel Velocity"); //65535, 7500.0, 0.0,
  differential_drive_simulation->so_encoder_ticks_right.ConnectTo(encoder_ticks_to_right_wheel_velocity->si_encoder_ticks);
  differential_drive_simulation->so_encoder_time_stamp.ConnectTo(encoder_ticks_to_right_wheel_velocity->si_encoder_time_stamp);
  encoder_ticks_to_right_wheel_velocity->so_wheel_velocity.ConnectTo(wheel_velocity_to_robot_velocity->si_velocity_right);
  encoder_ticks_to_right_wheel_velocity->par_max_encoder_ticks.SetConfigEntry("max. encoder ticks");
  encoder_ticks_to_right_wheel_velocity->par_encoder_ticks_per_m.SetConfigEntry("encoder ticks per meter");
  encoder_ticks_to_right_wheel_velocity->par_encoder_ticks_per_rad.SetConfigEntry("encoder ticks per radian");

  /*! camera capture simvis3d */
  new mTestFrameGrabberCoin(this, "FrameGrabber", &simulation->shared_scene);

//  /*! stereo vision processing*/
//  //new mStViPr (this);
//  new scheduling::tThreadContainerElement<mStViPr>(this); //new thread added :)
//
//  /*----------------  processsedSensorData to mapping -----------------*/
//  new finroc::icarus::simulation::mProcessedSensorDataToMapping(this);  //, "ProcessedSensorDataToMapping"  //, "ProcessedSensorDataToMapping_new"

//  /* sector map display for visualization */
//  new mMapToCanvas<rrlib::mapping::tMapGridPolar2D<double>, rrlib::canvas::tCanvas2D> (this, "sector map front scanner");
//  new mMapToCanvas<rrlib::mapping::tMapGridPolar2D<double>, rrlib::canvas::tCanvas2D> (this, "sector map rear scanner");
//
//  /* grid map display for visualization grid map*/
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for planar scanner temporary");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for planar scanner environment");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for actuated scanner temporary");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for actuated scanner environment");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for stereo temporary");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "grid map for stereo environment");
//
//  /*sector mapping */
//  new mMapToCanvas<rrlib::mapping::tMapGridPolar2D<double>, rrlib::canvas::tCanvas2D> (this, "sector map front left polar");
//  new mMapToCanvas<rrlib::mapping::tMapGridPolar2D<double>, rrlib::canvas::tCanvas2D> (this, "sector map front right polar");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "sector map fronnt cartesian");
//  new mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "sector map front cartesian left");




////  /*------------- test sensor_data_representation--------------------*/
////  new finroc::sensor_data_representation::test_grid_map_2d::tTestGridMapDefinitions::mDisplayGridMap(this, "DisplayGridMap");
//
//
  /*
   * BEGIN  -------------- 3D visualization -------------------------------------------------------------------------------
   */
  mVisualization * visualization = new mVisualization(this, "Visualization");

  mPoseWrapper * vis_robot_pose_wrapper = new mPoseWrapper(this, "Robot Pose Visualization");
  vis_robot_pose_wrapper->pose_input_mode.Set(mPoseWrapper::eSI); //.SetValue(mPoseWrapper::eSI);
  vis_robot_pose_wrapper->insertion_point_name.Set("ROBOT"); //.SetValue("ROBOT");
  vis_robot_pose_wrapper->element_name.Set("pose"); //.SetValue("pose");
  vis_robot_pose_wrapper->ConnectTo(visualization->parameter_value_bb, visualization->element_descriptor_bb);
  robot_velocity_to_pose->so_pose.ConnectTo(vis_robot_pose_wrapper->si_current_pose);
  vis_robot_pose_wrapper->co_scene_parameter_update_request.ConnectTo(visualization->ci_scene_parameter_update_request);

  mLineSetWrapper * vis_sensor_lines_wrapper = new mLineSetWrapper(this, "Sensor Lines");
  vis_sensor_lines_wrapper->insertion_point_name.Set("ROBOT"); //.SetValue("ROBOT");
  vis_sensor_lines_wrapper->element_name.Set("minimum distance line"); //.SetValue("minimum distance line");
  vis_sensor_lines_wrapper->ConnectTo(visualization->parameter_value_bb, visualization->element_descriptor_bb);
  vis_sensor_lines_wrapper->co_scene_parameter_update_request.ConnectTo(visualization->ci_scene_parameter_update_request);

  /*------------------------ ir sensors------------------*/
  vector<string> sensor_names;
  sensor_names.push_back("ir_sensor_0");
  sensor_names.push_back("ir_sensor_1");

  mSimulateDistanceSensor *distance_sensors = new mSimulateDistanceSensor(this, "Distance Sensors", &simulation->shared_scene, sensor_names);
  simulation->so_scene_changed.ConnectTo(distance_sensors->si_scene_changed);

  mCalculateDistanceSensorLines * calc_lines = new mCalculateDistanceSensorLines(this, "CalculateDistanceSensorLines", "sensors/ir_sensor_");
  distance_sensors->so_distance_value.ConnectTo(calc_lines->si_distance_values);
  calc_lines->so_start_points.ConnectTo(vis_sensor_lines_wrapper->si_line_start_points);
  calc_lines->so_end_points.ConnectTo(vis_sensor_lines_wrapper->si_line_end_points);

//  /*
//   * rear laser scanner 2d
//   */
//  mLaserScannerCoin *scanner = new mLaserScannerCoin(this, "Rear Scanner (COIN)");
//  scanner->shared_scene.AttachTo(simulation->shared_scene);
//  simulation->so_scene_changed.ConnectTo(scanner->si_scene_changed);
//  scanner->Init();
//  assert(!scanner->so_scans.empty());
//
//  mAppendRobotPoseToDistanceDataSimple *append_robot_pose_to_scan = new mAppendRobotPoseToDistanceDataSimple(this, "Rear Scanner Data in WCS");
//  scanner->so_scans [0].ConnectTo(append_robot_pose_to_scan->si_distance_data);
//  differential_drive_simulation->so_encoder_time_stamp.ConnectTo(append_robot_pose_to_scan->si_time_stamp);
//  robot_velocity_to_pose->so_pose.ConnectTo(append_robot_pose_to_scan->si_robot_pose);
//
//  mProcessDistanceData *process_distance_data = new mProcessDistanceData(this, "process_distance_data_rear_scanner");
//  append_robot_pose_to_scan->so_distance_data.ConnectTo(process_distance_data->si_distance_data);
//
//  /*! line from scanner to distance points */
//  mLineSetWrapper * vis_sensor_lines_wrapper2 = new mLineSetWrapper(this, "Sensor Lines Visualization");
//  vis_sensor_lines_wrapper2->insertion_point_name.Set("ROOT"); //.SetValue("ROOT");
//  vis_sensor_lines_wrapper2->element_name.Set("minimum distance line"); //.SetValue("minimum distance line");
//  vis_sensor_lines_wrapper2->ConnectTo(visualization->parameter_value_bb, visualization->element_descriptor_bb);
//  vis_sensor_lines_wrapper2->co_scene_parameter_update_request.ConnectTo(visualization->ci_scene_parameter_update_request);
//  process_distance_data->so_start_points.ConnectTo(vis_sensor_lines_wrapper2->si_line_start_points);
//  process_distance_data->so_end_points.ConnectTo(vis_sensor_lines_wrapper2->si_line_end_points);
//
//
//  /*
//   *  laser scanner 2d front
//   *
//   */
//  mLaserScannerCoin *scanner_front = new mLaserScannerCoin(this, "Front Scanner (COIN)");
//  scanner_front->shared_scene.AttachTo(simulation->shared_scene);
//  simulation->so_scene_changed.ConnectTo(scanner_front->si_scene_changed);
//  scanner_front->Init();
//  assert(!scanner_front->so_scans.empty());
//
//  /*! displaying the laser scanner and pose transform for updating the pose */
//  mAppendRobotPoseToDistanceDataSimple *append_robot_pose_to_scan_front = new mAppendRobotPoseToDistanceDataSimple(this, "Front Scanner Data in WCS");
//  scanner_front->so_scans [0].ConnectTo(append_robot_pose_to_scan_front->si_distance_data);
//  differential_drive_simulation->so_encoder_time_stamp.ConnectTo(append_robot_pose_to_scan_front->si_time_stamp);
//  robot_velocity_to_pose->so_pose.ConnectTo(append_robot_pose_to_scan_front->si_robot_pose);
//
//  /*! drawing line for min distance point for the front scanner*/
//  mProcessDistanceData *process_distance_data_front = new mProcessDistanceData(this, "process_distance_data_fron_scanner");
//  append_robot_pose_to_scan_front->so_distance_data.ConnectTo(process_distance_data_front->si_distance_data);
//
//  /*-------line from scanner to distance points -------*/
//  mLineSetWrapper * vis_sensor_lines_wrapper2_front = new mLineSetWrapper(this, "Sensor Lines Visualization_front");
//  vis_sensor_lines_wrapper2_front->insertion_point_name.Set("ROOT"); //.SetValue("ROOT");
//  vis_sensor_lines_wrapper2_front->element_name.Set("minimum distance line front"); //.SetValue("minimum distance line");
//  vis_sensor_lines_wrapper2_front->ConnectTo(visualization->parameter_value_bb, visualization->element_descriptor_bb);
//  vis_sensor_lines_wrapper2_front->co_scene_parameter_update_request.ConnectTo(visualization->ci_scene_parameter_update_request);
//  process_distance_data_front->so_start_points.ConnectTo(vis_sensor_lines_wrapper2_front->si_line_start_points);
//  process_distance_data_front->so_end_points.ConnectTo(vis_sensor_lines_wrapper2_front->si_line_end_points);
//
//  /*
//   *   3d front actauted scanner
//   *
//   */
//  mActuatedLaserScanner<mLaserScannerCoin> *actuated_scanner = new mActuatedLaserScanner<mLaserScannerCoin>(this, "Actuated Front Scanner (COIN)");
//  actuated_scanner->shared_scene.AttachTo(simulation->shared_scene);
//  simulation->so_scene_changed.ConnectTo(actuated_scanner->si_scene_changed);
//  actuated_scanner->Init();
//  assert(!actuated_scanner->so_scans.empty());
//
//  mTimeStepCounter *time_step_counter = new mTimeStepCounter(this);
//
//  mLaserTilterSimulator *laser_tilter_simulator = new mLaserTilterSimulator(this);
//  time_step_counter->so_time_step.ConnectTo(laser_tilter_simulator->si_scan_telegram_index);
//  laser_tilter_simulator->so_scan_telegram_index.ConnectTo(actuated_scanner->si_scan_telegram_index);
//  laser_tilter_simulator->so_angle.ConnectTo(actuated_scanner->si_scan_angle_pitch);
//
//  mLaserTilterControl *laser_tilter_control = new mLaserTilterControl(this);
//  laser_tilter_control->co_tilter_control.ConnectTo(laser_tilter_simulator->ci_control);
//  laser_tilter_control->co_tilter_velocity.ConnectTo(laser_tilter_simulator->ci_velocity);
//  laser_tilter_control->co_tilter_max_angle.ConnectTo(laser_tilter_simulator->ci_max_angle);
//  laser_tilter_control->co_tilter_min_angle.ConnectTo(laser_tilter_simulator->ci_min_angle);
//  laser_tilter_simulator->so_state.ConnectTo(laser_tilter_control->si_tilter_state);
//
//  mAppendRobotPoseToDistanceDataSimple *append_robot_pose_to_scan2 = new mAppendRobotPoseToDistanceDataSimple(this, "Actuated Scanner Data in WCS");
//  actuated_scanner->so_scans [0].ConnectTo(append_robot_pose_to_scan2->si_distance_data);
//  differential_drive_simulation->so_encoder_time_stamp.ConnectTo(append_robot_pose_to_scan2->si_time_stamp);
//  robot_velocity_to_pose->so_pose.ConnectTo(append_robot_pose_to_scan2->si_robot_pose);
//
//  mRotationWrapper *rotation_wrapper = new mRotationWrapper(this, "Actuated Scanner Rotation");
//  rotation_wrapper->insertion_point_name.Set("ACTUATED_SCANNER"); //.SetValue("ACTUATED_SCANNER");
//  rotation_wrapper->element_name.Set("actuated_scanner_mount"); //.SetValue("actuated_scanner_mount");
//  rotation_wrapper->ConnectTo(simulation->parameter_value_bb, simulation->element_descriptor_bb);
//  laser_tilter_simulator->so_angle.ConnectTo(rotation_wrapper->si_angle);
//  rotation_wrapper->co_scene_parameter_update_request.ConnectTo(simulation->ci_scene_parameter_update_request);
//
//  mRotationWrapper *rotation_wrapper_visualization = new mRotationWrapper(this, "Actuated Scanner Rotation Visualization");
//  rotation_wrapper_visualization->insertion_point_name.Set("ACTUATED_SCANNER"); //.SetValue("ACTUATED_SCANNER");
//  rotation_wrapper_visualization->element_name.Set("actuated_scanner_mount"); //.SetValue("actuated_scanner_mount");
//  rotation_wrapper_visualization->ConnectTo(visualization->parameter_value_bb, visualization->element_descriptor_bb);
//  laser_tilter_simulator->so_angle.ConnectTo(rotation_wrapper_visualization->si_angle);


  fprintf(stderr, "gIcarusSimulation finished.................................................................................\n");  //::ctor >>

}//end //gIcarusSimulation

}//simulation
}//icarus
}//finroc
