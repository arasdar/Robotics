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
/*!\file    projects/icarus/sensor_processing/libstereo_test/finroc/mStereoToSectorMap.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-07
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc/mStereoToSectorMap.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
namespace icarus
{
namespace sensor_processing
{
namespace stereo_gray
{
namespace offline
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStereoToSectorMap> cCREATE_ACTION_FOR_M_STEREOTOSECTORMAP("StereoToSectorMap");

//const int gridmap_bound_limit = 80; //10 //20 //40; //80;
//const double gridmap_cell_resolution = 0.1; //0.05; //meter as unit
//const double robot_width = 2, robot_height = 3.3; //this is based on vehicle size
//const double sector_cell_height = 1;
//const double sector_polar_cell_angle = 9;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mStereoToSectorMap constructor
//----------------------------------------------------------------------
mStereoToSectorMap::mStereoToSectorMap(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
  , sectormap_polar(new tSectorMapPolar2D)
{
  /*! sector polar big located at the robot center and facing front*/
//    tSectorMapPolar2D* sectormap_polar = new tSectorMapPolar2D;
  sectormap_polar->origin.Set(0, 0, 0);
}

//----------------------------------------------------------------------
// mStereoToSectorMap destructor
//----------------------------------------------------------------------
mStereoToSectorMap::~mStereoToSectorMap()
{}

////----------------------------------------------------------------------
//// mStereoToSectorMap OnStaticParameterChange
////----------------------------------------------------------------------
//void mStereoToSectorMap::OnStaticParameterChange()
//{
//  if (this->static_parameter_1.HasChanged())
//  {
//    /*As this static parameter has changed, do something with its value!*/
//  }
//}
//
////----------------------------------------------------------------------
//// mStereoToSectorMap OnParameterChange
////----------------------------------------------------------------------
//void mStereoToSectorMap::OnParameterChange()
//{
//  /*If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.*/
//}

//----------------------------------------------------------------------
// mStereoToSectorMap Update
//----------------------------------------------------------------------
void mStereoToSectorMap::Update()
{
  if (this->InputChanged())
  {
    /*At least one of your input ports has changed. Do something useful with its data.
    However, using the .HasChanged() method on each port you can check in more detail.*/
  }

  /*Do something each cycle independent from changing ports.*/
  gridmapStereo();
  secletmapStereo();

  /*this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.*/
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}
