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
/*!\file    projects/stereo_traversability_experiments/write/mWrite.h
 *
 * \author  Aras Dargazany
 *
 * \date    2016-03-04
 *
 * \brief Contains mWrite
 *
 * \b mWrite
 *
 * This module writes the recorded IO data sequence into the memory index by index or somehow unload the recorded data so that it is possible to use it for learning and recognition.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__write__mWrite_h__
#define __projects__stereo_traversability_experiments__write__mWrite_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
#include "rrlib/distance_data/tDistanceData.h"
#include <opencv2/core/core.hpp> //Mat & data structures
#include <opencv2/highgui/highgui.hpp> //imshow & gui
#include "rrlib/time/time.h"

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
namespace write
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module writes the recorded IO data sequence into the memory index by index or somehow unload the recorded data so that it is possible to use it for learning and recognition.
 */
class mWrite : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<std::string> recorded_IO_data_file; // this is static because it is only loaded when you run the program and it is not changing later during run-time

  tParameter<bool> start_xml_writer; // this is dynamic because it might change during run-time

  /*! Current robot pose - according to Odometry */
  //tInput<rrlib::localization::tUncertainPose3D<>> pose; //converted to rrlib::math::tPose3D and then to double
  tInput<double> pose_x, pose_y, /*pose_z, pose_roll, pose_pitch,*/ pose_yaw;

  /*! Current camera image from forklift's Logitech Pro 9000 camera with 360° mirror */
  tInput<rrlib::coviroa::tImage> camera_image; //not used yet

  /*! Distance values from infrared sensors. 'ir_distance_l' is the inner-most left sensor - 'ir_distance_lll' the outer-most */
  tInput<rrlib::si_units::tLength<>> ir_distance_front, ir_distance_l, ir_distance_ll, ir_distance_lll;
  tInput<rrlib::si_units::tLength<>> ir_distance_r, ir_distance_rr, ir_distance_rrr, ir_distance_rear;

  /*! recorded output*/
  /*! Desired velocity (in m/s) */
  tInput<rrlib::si_units::tVelocity<>> desired_velocity;

  /*! Desired angular velocity */
  tInput<rrlib::si_units::tAngularVelocity<>> desired_angular_velocity;

  /*! Desired fork position */
  tInput<double> desired_fork_position;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mWrite(core::tFrameworkElement *parent, const std::string &name = "Write");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mWrite();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //    Here is the right place for your variables. Replace this line by your declarations!
  /*unsigned*/
  int data_index;
  const rrlib::time::tTimestamp time_stamp; // label used for IO data
  const std::string time_stamp_string_label; // should be constant AND must not change since it is used as label
  std::string dir_name;

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;
  void write_xml();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
