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
/*!\file    projects/icarus/sensor_processing/stereo_sugv/capture/finroc/gSensorProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-02-03
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/gSensorProcessing.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/canvas/tCanvas2D.h"
#include "libraries/mapping/handlers/display/mMapToCanvas.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.h"
//#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereoToSectorMap.h"

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
namespace stereo_sugv
{
namespace capture
{
using namespace finroc::mapping::handlers::display;
using namespace rrlib::mapping;
using namespace rrlib::canvas;

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
//  new mStereo(this);
  mStereo* stereo_processing = new mStereo(this);

  /* grid map display for visualization grid map*/
  mMapToCanvas<tMapGridCartesian2D<double>, tCanvas2D>* gridmap2canvas = new mMapToCanvas<tMapGridCartesian2D<double>, tCanvas2D>(this, "gridmap");

  stereo_processing->output_gridmap.ConnectTo(gridmap2canvas->in_map);

//  mStereoToSectorMap* stereo_to_sectormap = new mStereoToSectorMap(this);
//  stereo_to_sectormap->input_canvas_gridmap.ConnectTo(gridmap2canvas->out_canvas);
//
//  mMapToCanvas<tMapGridPolar2D<double>, tCanvas2D>* secletmap2canvas = new mMapToCanvas<tMapGridPolar2D<double>, tCanvas2D>(this, "secletmap");
//  stereo_processing->output_secletmap.ConnectTo(secletmap2canvas->in_map);
//
//  stereo_to_sectormap->input_canvas_secletmap.ConnectTo(secletmap2canvas->out_canvas);
//  stereo_processing->output_secletmap.ConnectTo(stereo_to_sectormap->input_secletmap);
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
