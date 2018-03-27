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
/*!\file    mProcessDistanceData.h
 *
 * \author  Jens Wettach
 *
 * \date    2011-05-26
 *
 * \brief Contains mProcessDistanceData
 *
 * \b mProcessDistanceData
 *
 */
//----------------------------------------------------------------------
#ifndef _simvis3d_demo__mProcessDistanceData_h_
#define _simvis3d_demo__mProcessDistanceData_h_


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/structure/tSenseControlModule.h"
#include "rrlib/distance_data/tDistanceData.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
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
// Class declaration
//----------------------------------------------------------------------
//! Short description of mProcessDistanceData
/*! A more detailed description of mProcessDistanceData which
 *  Jens Wettach has not done yet!
 *
 */
class mProcessDistanceData : public structure::tSenseControlModule
{
//  static core::tStandardCreateModuleAction<mProcessDistanceData> cCREATE_ACTION;
  static finroc::runtime_construction::tStandardCreateModuleAction<mProcessDistanceData> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data;

  tSensorOutput<std::vector<rrlib::math::tVec3f>> so_start_points;
  tSensorOutput<std::vector<rrlib::math::tVec3f>> so_end_points;

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  mProcessDistanceData(core::tFrameworkElement *parent, const std::string &name = "ProcessDistanceData");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual void Sense();

  virtual void Control();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
