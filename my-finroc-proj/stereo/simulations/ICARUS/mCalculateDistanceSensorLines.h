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
/*!\file    mCalculateDistanceSensorLines.h
 *
 * \author  Jens Wettach
 *
 * \date    2011-05-10
 *
 * \brief Contains mCalculateDistanceSensorLines
 *
 * \b mCalculateDistanceSensorLines
 *
 */
//----------------------------------------------------------------------
#ifndef _simvis3d_demo__mCalculateDistanceSensorLines_h_
#define _simvis3d_demo__mCalculateDistanceSensorLines_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/structure/tSenseControlModule.h"
#include "rrlib/math/tVector.h"
#include "rrlib/math/tPose3D.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <vector>

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
//! Short description of mCalculateDistanceSensorLines
/*! A more detailed description of mCalculateDistanceSensorLines which
 *  Jens Wettach has not done yet!
 *
 */
class mCalculateDistanceSensorLines : public structure::tSenseControlModule
{
//  static core::tStandardCreateModuleAction<mCalculateDistanceSensorLines> cCREATE_ACTION; //OLD FINROC VERSION
  static finroc::runtime_construction::tStandardCreateModuleAction<mCalculateDistanceSensorLines> cCREATE_ACTION;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tSensorInput<std::vector<float>> si_distance_values;

  tSensorOutput<std::vector<rrlib::math::tVec3f>> so_start_points;
  tSensorOutput<std::vector<rrlib::math::tVec3f>> so_end_points;

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  mCalculateDistanceSensorLines(
    core::tFrameworkElement *parent,
    const std::string &name = "CalculateDistanceSensorLines",
    const std::string & distance_sensor_config_path = "");

//----------------------------------------------------------------------
// Protected methods (no fields/variables)
//----------------------------------------------------------------------
protected:

  virtual void Control();

  virtual void Sense();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
  std::vector<rrlib::math::tPose3D> sensor_poses;
  std::vector<rrlib::math::tVec3f> start_points, end_points;
  std::string distance_sensor_config_path;

  void Resize(size_t new_dimension);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
