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
/*!\file    projects/stereo_traversability_experiments/control/mRecognition.h
 *
 * \author  Aras Dargazany
 *
 * \date    2016-01-22
 *
 * \brief Contains mRecognition
 *
 * \b mRecognition
 *
 * This module receive the learnt data space parameters and also receives the current input from the low level control group or sensor output and try to recognize the similar pattern in the current input inside the dataspace and therefore generate the similar output corresponding to the similar input.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__control__mRecognition_h__
#define __projects__stereo_traversability_experiments__control__mRecognition_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
#include "rrlib/distance_data/tDistanceData.h"
#include <opencv2/core/core.hpp> //Mat & data structures
#include <opencv2/highgui/highgui.hpp> //imshow & gui
#include "rrlib/time/time.h"
#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

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
namespace control
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module receive the learnt data space parameters and also receives the current input from the low level control group or sensor output and try to recognize the similar pattern in the current input inside the dataspace and therefore generate the similar output corresponding to the similar input.
 */
class mRecognition : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  //tStaticParameter<double> static_parameter_1;   //Example for a static parameter. Replace or delete it!
  tStaticParameter<std::string> dir_path;   //dir_path("/home/aras/database/robot_control/"),

  //  /tParameter<double> par_parameter_1; //Example for a runtime parameter named "Parameter 1". Replace or delete it!
  /*! Recognition part to receive data flow from the robot interface and do the control  --- data flow in real-time needed */
  tParameter<bool> enable_camera, load_recognizer, start_recognizer;

  /*! Input - IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII */
  //tInput<rrlib::localization::tUncertainPose3D<>> pose; //converted to rrlib::math::tPose3D and then to double
  tInput<double> pose_x, pose_y, /*pose_z, pose_roll, pose_pitch,*/ pose_yaw;

  /*! Current camera image from forklift's Logitech Pro 9000 camera with 360° mirror */
  tInput<rrlib::coviroa::tImage> camera_image; //not used yet

  /*! Distance values from infrared sensors. 'ir_distance_l' is the inner-most left sensor - 'ir_distance_lll' the outer-most */
  tInput<rrlib::si_units::tLength<>> ir_distance_front, ir_distance_l, ir_distance_ll, ir_distance_lll;
  tInput<rrlib::si_units::tLength<>> ir_distance_r, ir_distance_rr, ir_distance_rrr, ir_distance_rear;

  /*! Output - OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO */
  /*! Desired velocity (in m/s) */
  tOutput<rrlib::si_units::tVelocity<>> desired_velocity;

  /*! Desired angular velocity */
  tOutput<rrlib::si_units::tAngularVelocity<>> desired_angular_velocity;

  /*! Desired fork position */
  tOutput<double> desired_fork_position;



//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mRecognition(core::tFrameworkElement *parent, const std::string &name = "Recognition");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mRecognition();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  stereo_traversability_experiments::mlr::tMLR* mlr;
  mlr::learn_PCA::tPCA* pca; //cv::PCA* pca; replace due to FLOAT precision
  //also calculating the S_diag_inv
  cv::Mat S_diag_inv; // this is used for compare function
  std::vector<std::string> filenames;
  std::string file_parent_path;
  mlr::RecordedData* recorded_data;

  // to wait for a new output and create the meaningful autonomy
  unsigned int cycle_counter; // to count the number of cycle we are waiting till check on the input change
  bool is_recognizer_loaded, are_filenames_loaded;

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;

  // recognizer
  void load();
  void process();
  void search(/*INPUT*/const cv::Mat& I_new);
  void output(/*INPUT*/const std::vector<mlr::RecordedData>& vec_recorded_data);



};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
