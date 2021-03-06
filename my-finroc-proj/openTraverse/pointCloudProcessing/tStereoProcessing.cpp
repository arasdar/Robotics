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
/*!\file    projects/icarus/sensor_processing/stereo_gray/offline/tStereoProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-08-03
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/openTraverse/pointCloudProcessing/tStereoProcessing.h"

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
namespace pointCloudProcessing
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
// tStereoProcessing constructors
//----------------------------------------------------------------------
tStereoProcessing::tStereoProcessing(const std::vector<std::string> left_images, const int img_pairs_num):
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer - input cloud")),
  prev_cloud(new Cloud),
  prev_cloud_filtered(new Cloud),
  prev_cloud_trav(new Cloud)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{
  trigger = true;
  continuous = false;
  display_normals = false;

  this->left_images = left_images;
  images_idx = -1;
  this->img_pairs_num = img_pairs_num;

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->addCoordinateSystem(1.0, "global");
  viewer->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

}

//----------------------------------------------------------------------
// tStereoProcessing destructor
//----------------------------------------------------------------------
tStereoProcessing::~tStereoProcessing()
{}

/*
//----------------------------------------------------------------------
// tStereoProcessing SomeExampleMethod
//----------------------------------------------------------------------
void tStereoProcessing::SomeExampleMethod()
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
