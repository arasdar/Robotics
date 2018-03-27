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
/*!\file    projects/icarus/sensor_processing/kinect/capture/finroc/pSensorProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-15
 *
 * \brief Contains mSensorProcessing
 *
 * \b pSensorProcessing
 *
 * This is the part fr sensor processing for testing the kinect procesing online.
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <chrono>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//#include "projects/icarus/sensor_processing/kinect/capture/finroc/mSensorProcessing.h"
#include "projects/icarus/sensor_processing/kinect/capture/finroc/gSensorProcessing.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the SensorProcessing module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
const std::string cMAIN_THREAD_CONTAINER_NAME = "Main Thread";
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
// InitMainGroup
//----------------------------------------------------------------------
void InitMainGroup(finroc::structure::tThreadContainer *main_thread, const std::vector<std::string> &remaining_arguments)
{

  //new finroc::icarus::sensor_processing::kinect::capture::finroc::mSensorProcessing(main_thread);
  //new finroc::icarus::sensor_processing::kinect::capture::finroc::gSensorProcessing(main_thread);

  new finroc::icarus::sensor_processing::kinect::capture::gSensorProcessing(main_thread);
  main_thread->SetCycleTime(std::chrono::milliseconds(50));
}
