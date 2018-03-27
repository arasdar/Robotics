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
/*!\file    plugins/data_playback/examples/pPlayer.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-10-17
 *
 *
 * \b pPlayer
 *
 * Example part which can be used to play back files specified on the command line.
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <chrono>
#include "plugins/data_playback/mDataPlayback.h"
#include "plugins/runtime_construction/dynamic_loading.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/MLR/write/mWrite.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>


// register some data types
#ifdef _LIB_RRLIB_MATH_PRESENT_
#include "rrlib/math/tPose3D.h"
#include "rrlib/math/tPose2D.h"
static rrlib::rtti::tDataType<rrlib::math::tPose2D> __pose2d_t;
static rrlib::rtti::tDataType<rrlib::math::tPose3D> __pose3d_t;
#endif

#ifdef _LIB_RRLIB_TIME_PRESENT_
#include "rrlib/time/time.h"
static rrlib::rtti::tDataType<rrlib::time::tTimestamp> __timestamp_t;
#endif

#ifdef _LIB_RRLIB_LOCALIZATION_POSE_PRESENT_
#include "rrlib/localization/tPose.h"
static rrlib::rtti::tDataType<rrlib::localization::tPose2D<>> init_type_pose_2d;
static rrlib::rtti::tDataType<rrlib::localization::tPose3D<>> init_type_pose_3d;

static rrlib::rtti::tDataType<rrlib::localization::tTwist2D<>> init_type_twist_2d;
static rrlib::rtti::tDataType<rrlib::localization::tTwist3D<>> init_type_twist_3d;

#include "rrlib/localization/tUncertainPose.h"
static rrlib::rtti::tDataType<rrlib::localization::tUncertainPose2D<>> init_type_uncertain_pose_2d;
static rrlib::rtti::tDataType<rrlib::localization::tUncertainPose3D<>> init_type_uncertain_pose_3d;

static rrlib::rtti::tDataType<rrlib::localization::tUncertainTwist2D<>> init_type_uncertain_twist_2d;
static rrlib::rtti::tDataType<rrlib::localization::tUncertainTwist3D<>> init_type_uncertain_twist_3d;

#endif

#ifdef _LIB_RRLIB_NAVIGATION_PRESENT_
#include "rrlib/navigation/path/tPath.h"
static rrlib::rtti::tDataType<rrlib::navigation::path::tPath> init_type_path;

#endif

#ifdef _LIB_RRLIB_COVIROA_BASE_PRESENT_
#include "rrlib/coviroa/tImage.h"
static rrlib::rtti::tDataType<rrlib::coviroa::tImage> init_type_image;
#endif

//patrick locally added
#ifdef _LIB_RRLIB_COVIROA_OPENCV_PRESENT_
#include "rrlib/coviroa/tImageCompressionOpenCV.h"
#endif


#ifdef _LIB_FINROC_LIBRARIES_DISTANCE_DATA_PRESENT_
#include "rrlib/distance_data/tDistanceData.h"
static rrlib::rtti::tDataType<rrlib::distance_data::tDistanceData> init_type_distance_data;
#endif

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the DataPlayback module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "<file>";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// CreateMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  finroc::structure::tTopLevelThreadContainer<> *main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);

  if (remaining_arguments.size() != 1)
  {
    FINROC_LOG_PRINT(ERROR, "Exactly one file to play back has to be specified");
    exit(1);
  }

  auto *playback = new finroc::data_playback::mDataPlayback(main_thread, "DataPlayback", true /* share the ports when running the standalone part */);
  playback->file_to_playback.Set(remaining_arguments[0]);

  main_thread->SetCycleTime(std::chrono::milliseconds(50));

  // how to create a module in part or process
  ///*std::*//*auto* writer =*/ finroc::runtime_construction::LoadComponentType("libfinroc_projects_stereo_traversability_experiments_write.so", "Learning").CreateModule(main_thread, "Learning");
  //new finroc::stereo_traversability_experiments::write::mLearning(main_thread, "Write", true);
  //writer.set_dir(remaining_arguments[0]);

  finroc::runtime_construction::tCreateFrameworkElementAction& framework_element_action =
    finroc::runtime_construction::LoadComponentType("libfinroc_projects_stereo_traversability_experiments_write.so", "Write");
  finroc::core::tFrameworkElement* framework_element = framework_element_action.CreateModule(main_thread, "Write");
  finroc::stereo_traversability_experiments::write::mWrite* writer = dynamic_cast<finroc::stereo_traversability_experiments::write::mWrite*>(framework_element);
  writer->recorded_IO_data_file.Set(remaining_arguments[0]);

}// CreateMainGroup()
