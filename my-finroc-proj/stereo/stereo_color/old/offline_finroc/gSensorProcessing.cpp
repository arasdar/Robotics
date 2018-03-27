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
/*!\file    projects/icarus/sensor_processing/stereo_color/offline/finroc/gSensorProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-15
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_color/offline/finroc/gSensorProcessing.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_color/offline/finroc/mStereoTestColorOffline.h"
#include "rrlib/canvas/tCanvas2D.h"
#include "libraries/mapping/handlers/display/mMapToCanvas.h"

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
namespace icarus
{
namespace sensor_processing
{
namespace stereo_color
{
namespace offline
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gSensorProcessing> cCREATE_ACTION_FOR_G_SENSORPROCESSING("SensorProcessing");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gSensorProcessing constructor
//----------------------------------------------------------------------
gSensorProcessing::gSensorProcessing(core::tFrameworkElement *parent, const std::string &name,
                                     const std::string &structure_config_file) :
  tGroup(parent, name, structure_config_file, false, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{

//  mStereoColorOffline* stereo_color_offline = new mStereoColorOffline(this);
  mStereoTestColorOffline* stereo_color_offline = new mStereoTestColorOffline(this);

  /* grid map display for visualization grid map*/
  finroc::mapping::handlers::display::mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>* map2canvas = new finroc::mapping::handlers::display::mMapToCanvas<rrlib::mapping::tMapGridCartesian2D<double>, rrlib::canvas::tCanvas2D>(this, "gridmap");

  stereo_color_offline->output_gridmap.ConnectTo(map2canvas->in_map);

}

//----------------------------------------------------------------------
// gSensorProcessing destructor
//----------------------------------------------------------------------
gSensorProcessing::~gSensorProcessing()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}
