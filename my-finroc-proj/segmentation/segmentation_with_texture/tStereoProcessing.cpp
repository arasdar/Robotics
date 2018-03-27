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
#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/tStereoProcessing.h"

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
namespace daniel
{
namespace segmentation_with_texture
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
tStereoProcessing::tStereoProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                                     const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename):
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer - PCD and Normals")),
  viewer_disparity(new pcl::visualization::PCLVisualizer("3D Viewer Disparity")),
  viewer_proc_segm_normal(new pcl::visualization::PCLVisualizer("3D Viewer Segmentation Normal")),
  viewer_proc_segm_texture(new pcl::visualization::PCLVisualizer("3D Viewer Segmentation Texture")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer - PCD and Normals")),
  image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
  image_viewer_proc_segm_normal(new visualization::ImageViewer("Image Viewer Segmentation Normal")),
  image_viewer_proc_segm_texture(new visualization::ImageViewer("Image Viewer Segmentation Texture")),
  comp_normal(new libsegmentation::GroundPlaneComparator<PointT, Normal>),
  segm_normal(comp_normal),
  comp_texture(new libsegmentation::PlaneTextureComparator<PointT, Texture>),
  segm_texture(comp_texture)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{
  trigger = true;
  continuous = false;
  display_normals = false;

  this->left_images = left_images;
  this->right_images = right_images;
  images_idx = 0;
  this->img_pairs_num = img_pairs_num;
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_disparity->setBackgroundColor(0, 0, 0);
  viewer_disparity->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disparity->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_disparity->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_proc_segm_normal->setBackgroundColor(0, 0, 0);
  viewer_proc_segm_normal->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_proc_segm_normal->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_proc_segm_normal->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_proc_segm_texture->setBackgroundColor(0, 0, 0);
  viewer_proc_segm_texture->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_proc_segm_texture->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_proc_segm_texture->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);
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
