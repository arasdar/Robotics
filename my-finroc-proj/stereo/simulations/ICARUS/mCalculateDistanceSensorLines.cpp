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
/*!\file    mCalculateDistanceSensorLines.cpp
 *
 * \author  Jens Wettach
 *
 * \date    2011-05-10
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/lexical_cast.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/parameters/tConfigFile.h"
#include "rrlib/math/tPose3D.h"
#include "projects/stereo_traversability_experiments/simulation/mCalculateDistanceSensorLines.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation
{

using namespace rrlib::logging;
using rrlib::math::tVec3f;
using rrlib::math::tPose3D;
using std::vector;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
finroc::runtime_construction::tStandardCreateModuleAction<mCalculateDistanceSensorLines> cCREATE_ACTION_FOR_M_CALCULATE_DISTANCE_SENSOR_LINES("CalculateDistanceSensorLines");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mCalculateDistanceSensorLines constructors
//----------------------------------------------------------------------
mCalculateDistanceSensorLines::mCalculateDistanceSensorLines(
  core::tFrameworkElement *_parent,
  const std::string &_name,
  const std::string &_distance_sensor_config_path)
  : tSenseControlModule(_parent, _name),
    sensor_poses(),
    distance_sensor_config_path(_distance_sensor_config_path)
{}

//----------------------------------------------------------------------
// mCalculateDistanceSensorLines Control
//----------------------------------------------------------------------
void mCalculateDistanceSensorLines::Control()
{} // Control()

//----------------------------------------------------------------------
// mCalculateDistanceSensorLines Sense
//----------------------------------------------------------------------
void mCalculateDistanceSensorLines::Sense()
{
  if (si_distance_values.HasChanged())
  {
    std::vector<float> distance_values;
    si_distance_values.Get(distance_values);
    if (this->sensor_poses.size() != distance_values.size())
    {
      Resize(distance_values.size());
    }
    for (size_t i = 0; i < distance_values.size(); ++i)
    {
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "got distance value ", distance_values[i]);
      this->end_points.at(i) = tPose3D(distance_values[i], 0, 0).GetPoseInParentFrame(this->sensor_poses[i]).Position();
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "writing start ", this->start_points.at(i));
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "writing end ", this->end_points.at(i));
    }
    so_start_points.Publish(start_points);
    so_end_points.Publish(end_points);

    si_distance_values.ResetChanged();
  }
} // Sense()


//----------------------------------------------------------------------
// mCalculateDistanceSensorLines Resize
//----------------------------------------------------------------------
void mCalculateDistanceSensorLines::Resize(size_t new_dimension)
{
  parameters::tConfigFile* config_file = parameters::tConfigFile::Find(*this->GetParent());
  tPose3D sensor_pose;
  this->sensor_poses.resize(new_dimension);
  this->start_points.resize(new_dimension);
  this->end_points.resize(new_dimension);
  for (size_t i = 0; i < new_dimension; ++i)
  {
    std::string config_entry(this->distance_sensor_config_path + boost::lexical_cast<std::string>(i) + "/pose");
    try
    {
      config_file->GetEntry(config_entry) >> sensor_pose;
    }
    //catch (util::tRuntimeException& e)  //old finroc //error!!!!
    catch (const std::exception& e)
    {
      FINROC_LOG_PRINT(ERROR, "could not find ", config_entry);
      sensor_pose = tPose3D::Zero();
    }
    this->sensor_poses.at(i) = sensor_pose;
    this->start_points.at(i) = this->sensor_poses.at(i).Position();
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "got sensor pose from config file entry <", config_entry, ">: ", this->sensor_poses.at(i));
  }
} // Resize()

}
}
}

