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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/gPerception.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2015-04-27
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physics/gPerception.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"
//#include "libraries/localization/mOdometry.h"
//#include "libraries/localization/mOdometry.h"
#include "rrlib/mapping/definitions.h"
#include "rrlib/canvas/tCanvas2D.h"
#include "libraries/mapping/handlers/display/mMapToCanvas.h"


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

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
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
static runtime_construction::tStandardCreateModuleAction<gPerception> cCREATE_ACTION_FOR_G_PERCEPTION("Perception");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gPerception constructor
//----------------------------------------------------------------------
gPerception::gPerception(core::tFrameworkElement *parent, const std::string &name,
                         const std::string &structure_config_file) :
  tGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{

  // mapping group using sensors
  new stereo_processing::mStereoProcessing(this, "mapping-stereo");
  /* mapping -- visualization of gridmap -- robot environment representation */
  new mapping::handlers::display::mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "gridmap");

  //localization sensor output is used hier for accurate lozaliation used in mapping the environment
  // robot and environment representation
//  new localization::mOdometry(this, "localization-odometry");

}

//----------------------------------------------------------------------
// gPerception destructor
//----------------------------------------------------------------------
gPerception::~gPerception()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
