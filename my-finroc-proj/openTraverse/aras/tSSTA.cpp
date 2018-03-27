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
/*!\file    projects/stereo_traversability_experiments/openTraverse/aras/tSSTA.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-10-21
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/openTraverse/aras/tSSTA.h"

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
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace aras
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tSSTA constructors
//----------------------------------------------------------------------
tSSTA::tSSTA():
  comp(new libsegmentation::GroundPlaneComparator<PointT, Normal>),
  segm(comp),
  prev_normal_cloud(new PointCloud<Normal>)
  ,
  prev_cloud(new Cloud),
  prev_cloud_segm(new Cloud),
  prev_cloud_trav_slopeAnalysis(new Cloud),
  prev_cloud_trav_dominGroundPlane(new Cloud),
  prev_cloud_trav_stepAnalysis(new Cloud)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{}

//----------------------------------------------------------------------
// tSSTA destructor
//----------------------------------------------------------------------
tSSTA::~tSSTA()
{}

/*
//----------------------------------------------------------------------
// tSSTA SomeExampleMethod
//----------------------------------------------------------------------
void tSSTA::SomeExampleMethod()
{
  This is an example for a method. Replace it by your own methods!
}
*/

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
