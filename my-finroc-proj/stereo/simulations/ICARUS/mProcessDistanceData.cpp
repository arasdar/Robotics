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
/*!\file    mProcessDistanceData.cpp
 *
 * \author  Jens Wettach
 *
 * \date    2011-05-26
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation/mProcessDistanceData.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>
#include <iterator>     // std::iterator, std::input_iterator_tag
#include <iostream>     // std::cout
#include <vector>       // std::vector, std::begin, std::end


//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::logging;
using namespace rrlib::math;

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
// Const values
//----------------------------------------------------------------------
finroc::runtime_construction::tStandardCreateModuleAction<mProcessDistanceData> cCREATE_ACTION_FOR_M_PROCESS_DISTANCE_DATA("ProcessDistanceData");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mProcessDistanceData constructors
//----------------------------------------------------------------------
mProcessDistanceData::mProcessDistanceData(core::tFrameworkElement *parent, const std::string &name)
  : tSenseControlModule(parent, name)
{}

//----------------------------------------------------------------------
// mProcessDistanceData Control
//----------------------------------------------------------------------
void mProcessDistanceData::Control()
{}

//----------------------------------------------------------------------
// local helper function tComparePolarRadius()
//----------------------------------------------------------------------
bool tComparePolarRadius(const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> &v1, const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> &v2)
{
  return v1.Length() < v2.Length();
}

//----------------------------------------------------------------------
// mProcessDistanceData Sense
//----------------------------------------------------------------------
void mProcessDistanceData::Sense()
{
  if (SensorInputChanged() &&
      si_distance_data.HasChanged())
  {
    data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data.GetPointer();
    if (distance_data->DistanceDataFormat() != rrlib::distance_data::eDF_POLAR_2D_DOUBLE)
    {
      FINROC_LOG_PRINT(WARNING, "Illegal input distance data format, eDF_POLAR_2D_DOUBLE required!\n");
      distance_data->Print();
      return;
    }
    FINROC_LOG_PRINT(DEBUG_VERBOSE_3, *distance_data);
    assert(distance_data->DistanceDataFormat() == rrlib::distance_data::eDF_POLAR_2D_DOUBLE);
    // NOTE: now we are sure that each sample is represented as tVec2 in polar form, i.e. (angle, radius)

    // access robot pose from source ... init absolute_sensor_pose
    rrlib::math::tPose3D absolute_sensor_pose(distance_data->RobotPose());

    // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
    absolute_sensor_pose.ApplyRelativePoseTransformation(distance_data->SensorPose());
    absolute_sensor_pose.Scale(0.001); // for displaying the minimum distance sample we need unit meter

    // access distance data samples from source
    const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *data = reinterpret_cast<const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *>(distance_data->ConstDataPtr());

    // process data ... calculate sample with minimum distance to sensor
    // uses local helper function for comparing two samples with respect to their radius, i. e. y value
    // NOTE: as we are directly working on distance_data_bb->Element(this->distance_data_bb_element_index)
    // the blackboard has to be kept locked as long as we access the data
    const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *min_distance_sample = std::min_element(data, data + distance_data->Dimension(), tComparePolarRadius);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1,  "min_distance_sample with length ", min_distance_sample->Length(), " at angle ", static_cast<double>(min_distance_sample->Alpha()));
    //DEBUG_VERBOSE_1

//    //test //working
//    std::vector<rrlib::math::tVector<2, double, rrlib::math::vector::Polar>> myvector (data, data + distance_data->Dimension());
//    std::for_each(myvector.begin(), myvector.end(), test);
//
//    //test 2 //error
////    rrlib::math::tVector<2, double, rrlib::math::vector::Polar> from(data);
////    rrlib::math::tVector<2, double, rrlib::math::vector::Polar> from(myvector.begin());
////    rrlib::math::tVector<2, double, rrlib::math::vector::Polar> until(distance_data->Dimension());
//    for (std::vector<rrlib::math::tVector<2, double, rrlib::math::vector::Polar>>& it= *data; it!=distance_data->Dimension(); it++)
////      std::cout << *it << ' ';
//    std::cout << '\n';
//
//    //test 3 //working
//    // iterate bar: print contents:
//    for (auto it = myvector.begin(); it!=myvector.end(); ++it)
//      std::cout << ' ' << *it;
//    std::cout << '\n';
//
//    //test3-2 //working
//    for (auto it = data; it != data+distance_data->Dimension(); ++it)
//      std::cout << ' ' << *it;

    // save sensor relative position of min_distance_sample to be used independent of distance_data_bb
    tVec2f min_distance_sample_position = min_distance_sample->GetCartesianVector() * 0.001; // for displaying the minimum distance sample we need unit meter
    std::vector<tVec3f> start, end;
    start.push_back(absolute_sensor_pose.Position());
//    absolute_sensor_pose.ApplyRelativePoseTransformation(rrlib::math::tPose3D(min_distance_sample_position.X(), min_distance_sample_position.Y(), 0, 0, 0, 0));
    absolute_sensor_pose.ApplyRelativePoseTransformation(rrlib::math::tPose3D(min_distance_sample_position.X(), min_distance_sample_position.Y(), 0, rrlib::math::tAngleDeg(0), rrlib::math::tAngleDeg(0), rrlib::math::tAngleDeg(0)));
    end.push_back(absolute_sensor_pose.Position());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, start.front(), end.front());
    so_start_points.Publish(start);
    so_end_points.Publish(end);
  }
}

}
}
}
