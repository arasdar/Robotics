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
/*!\file    projects/stereo_traversability_experiments/write/mWrite.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2016-03-04
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/MLR/write/mWrite.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/filesystem.hpp>
#include "libraries/structure_elements/mLambdaTypeConverter.h" // convert type to scalar
#include "rrlib/localization/tUncertainPose.h" //for tPose needed

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
using namespace std;

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
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mWrite> cCREATE_ACTION_FOR_M_WRITE("Write");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mWrite constructor
//----------------------------------------------------------------------
mWrite::mWrite(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  camera_image("Camera Image"),
  ir_distance_front("IR Distance Front"),
  ir_distance_l("IR Distance L"),
  ir_distance_ll("IR Distance LL"),
  ir_distance_lll("IR Distance LLL"),
  ir_distance_r("IR Distance R"),
  ir_distance_rr("IR Distance RR"),
  ir_distance_rrr("IR Distance RRR"),
  ir_distance_rear("IR Distance Rear"),
  desired_velocity("Desired Velocity", data_ports::tBounds<rrlib::si_units::tVelocity<>>(-1, 1)),
  desired_angular_velocity("Desired Angular Velocity", data_ports::tBounds<rrlib::si_units::tAngularVelocity<>>(-1, 1)),
  desired_fork_position("Desired Fork Position"),
  data_index(0),
  time_stamp(rrlib::time::Now()), // only once for dir and label - constant
  time_stamp_string_label(rrlib::time::ToIsoString(time_stamp)), // only once -- constant
  dir_name()
{
  /*
  //  pose_to_->ConnectTo(this->pose);
    // this is finroc group and not rrlib but lib
    // rrlib only portable classes - class type at input and output ports -- rrlib libs - indep lib for ports
    // libs modules and groups and parts - finroc components visualizable
    //plugins to plug it into to the core and extend the core functonality
    // core finroc kernel and core which is the whole foundation
    // projects is a complete application of all these components
    // java for gui and structure
  */
  new structure_elements::mLambdaTypeConverter<rrlib::localization::tUncertainPose3D<>, double>(this,
      [](const rrlib::math::tPose3D & pose)
  {
    return pose.X();
  }, "Pose_to_X", 1);
  new structure_elements::mLambdaTypeConverter<rrlib::localization::tUncertainPose3D<>, double>(this,
      [](const rrlib::math::tPose3D & pose)
  {
    return pose.Y();
  }, "Pose_to_Y", 1);
  new structure_elements::mLambdaTypeConverter<rrlib::localization::tUncertainPose3D<>, double>(this,
      [](const rrlib::math::tPose3D & pose)
  {
    return pose.Yaw().Value();
  }, "Pose_to_Yaw", 1);

}// mWrite::mWrite()

//----------------------------------------------------------------------
// mWrite destructor
//----------------------------------------------------------------------
mWrite::~mWrite()
{}

//----------------------------------------------------------------------
// mWrite OnStaticParameterChange
//----------------------------------------------------------------------
void mWrite::OnStaticParameterChange()
{
  //make sure the the recorded file is given
  if (this->recorded_IO_data_file.HasChanged()) // meaning the parameter was given
  {
    FINROC_LOG_PRINT(DEBUG, "this->recorded_IO_data_file.HasChanged(): ", this->recorded_IO_data_file.HasChanged());
    FINROC_LOG_PRINT(DEBUG, "this->recorded_IO_data_file.Get(): ", this->recorded_IO_data_file.Get());

    this->dir_name = this->recorded_IO_data_file.Get() + "_";

    // Creating the director to write the recorded bin file
    //boost::filesystem::path p(argv[1]);    // p reads clearer than argv[1] in the following code
    boost::filesystem::path p(this->dir_name);

    // if it already exists, remove it first completely with its content
    if (boost::filesystem::is_directory(p)) // it already exist or already have been written to the memory -- unloade recorded_
    {
      boost::filesystem::remove_all(p); // remove dir and all its content
      std::cout  << this->dir_name << " already exists and now it is deleted.............." << "\n";
    }// else if

    // This is the actual creation of directory
    if (boost::filesystem::create_directories(p))
    {
      std::cout  << dir_name << " created.............." << "\n";
    }
  }// if parameter

}// mWrite::OnStaticParameterChange()

//----------------------------------------------------------------------
// mWrite OnParameterChange
//----------------------------------------------------------------------
void mWrite::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mWrite Update
//----------------------------------------------------------------------
void mWrite::Update()
{
  if (this->InputChanged()) // input has changed or has JUST changed --> input data
  {
    //At least one of your input ports has changed. Do something useful with its data.
    //However, using the .HasChanged() method on each port you can check in more detail.
    if (this->start_xml_writer.Get()) // if true, starts writing
    {
      FINROC_LOG_PRINT(DEBUG, "this->start_xml_writer.Get(): ", this->start_xml_writer.Get());
      /*! NOT SURE REALLY IF THIS IS NECCESSARY!!!!!!!!!!!!!!!!
       * To add a path for the directory argv is needed to be passed hier
        void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
          playback->file_to_playback.Set(remaining_arguments[0]);
          */

      std::cout << "writing data files into " << this->dir_name << std::endl;
      this->write_xml();

    }// if this->inputchanged

    //Do something each cycle independent from changing ports.

    //this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
  }
}// mWrite::Update()

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
