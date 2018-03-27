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
#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

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
namespace stereo_color_hybrid
{
namespace offline
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
  viewer_disparity_processed(new pcl::visualization::PCLVisualizer("3D Viewer Disparity Processed")),
  viewer_proc_invalid_points(new pcl::visualization::PCLVisualizer("3D Viewer Invalid Points")),
  viewer_proc_segm(new pcl::visualization::PCLVisualizer("3D Viewer Segmentation")),
  viewer_proc_segm_appear(new pcl::visualization::PCLVisualizer("3D Viewer Appearance-Based Segmentation")),
  viewer_proc_trav(new pcl::visualization::PCLVisualizer("3D Viewer Traversability Dominant Segment and Frame Quality")),
  viewer_proc_trav_slopeAnalysis(new visualization::PCLVisualizer("3D Viewer Traversability SlopeAnalysis")),
  viewer_proc_trav_slopeAnalysis_appear(new visualization::PCLVisualizer("3D Viewer Traversability Appearance-Based SlopeAnalysis")),
  //viewer_proc_trav_slopeMapping_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Mapped Slope Discrete")),
  //viewer_proc_trav_stepMapping_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Mapped Step Discrete")),
  viewer_proc_trav_stepAnalysis(new visualization::PCLVisualizer("3D Viewer Traversability StepAnalysis")),
  viewer_proc_trav_stepAnalysis_appear(new visualization::PCLVisualizer("3D Viewer Traversability Appearance-Based StepAnalysis")),
  //viewer_proc_trav_slopeFusion_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Fused Slope Discrete")),
  //viewer_proc_trav_stepFusion_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Fused Step Discrete")),
  viewer_proc_trav_geomFusion_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Fused Geom Discrete")),
  viewer_proc_trav_geomFusionMapping_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Fused & Mapped Geom Discrete")),
  viewer_proc_trav_geomFusionMapping_discrete_uh(new visualization::PCLVisualizer("3D Viewer Traversability Fused & Mapped Unknown Handled Geom Discrete")),
  //viewer_proc_trav_finalFusion_discrete(new visualization::PCLVisualizer("3D Viewer Traversability Fused Final Discrete")),
  //viewer_proc_trav_final(new visualization::PCLVisualizer("3D Viewer Traversability final")),
  //viewer_proc_trav_final_appear(new visualization::PCLVisualizer("3D Viewer Traversability Appearance-Based final"))
  //,
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer - PCD and Normals")),
  image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
  image_viewer_disparity_processed(new visualization::ImageViewer("Image Viewer Disparity Processed")),
  image_viewer_proc_invalid_points(new visualization::ImageViewer("Image Viewer Invalid Points")),
  image_viewer_proc_segm(new visualization::ImageViewer("Image Viewer Segmentation")),
  image_viewer_proc_segm_appear(new visualization::ImageViewer("Image Viewer Appearance-Based Segmentation")),
  image_viewer_proc_trav(new visualization::ImageViewer("Image Viewer Traversability Dominant Segment and Frame Quality")),
  image_viewer_proc_trav_slopeAnalysis(new visualization::ImageViewer("Image Viewer Traversability SlopeAnalysis")),
  image_viewer_proc_trav_slopeAnalysis_appear(new visualization::ImageViewer("Image Viewer Traversability Appearance-Based SlopeAnalysis")),
  //image_viewer_proc_trav_slopeMapping_discrete(new visualization::ImageViewer("Image Viewer Traversability Mapped Slope Discrete")),
  //image_viewer_proc_trav_stepMapping_discrete(new visualization::ImageViewer("Image Viewer Traversability Mapped Step Discrete")),
  image_viewer_proc_trav_stepAnalysis(new visualization::ImageViewer("Image Viewer Traversability StepAnalysis")),
  image_viewer_proc_trav_stepAnalysis_appear(new visualization::ImageViewer("Image Viewer Traversability Appearance-Based StepAnalysis")),
  //image_viewer_proc_trav_slopeFusion_discrete(new visualization::ImageViewer("Image Viewer Traversability Fused Slope Discrete")),
  //image_viewer_proc_trav_stepFusion_discrete(new visualization::ImageViewer("Image Viewer Traversability Fused Step Discrete")),
  image_viewer_proc_trav_geomFusion_discrete(new visualization::ImageViewer("Image Viewer Traversability Fused Geom Discrete")),
  image_viewer_proc_trav_geomFusionMapping_discrete(new visualization::ImageViewer("Image Viewer Traversability Fused & Mapped Geom Discrete")),
  image_viewer_proc_trav_geomFusionMapping_discrete_uh(new visualization::ImageViewer("Image Viewer Traversability Fused & Mapped Unknown Handled Geom Discrete")),
  //image_viewer_proc_trav_finalFusion_discrete(new visualization::ImageViewer("Image Viewer Traversability Fused Final Discrete")),
  //image_viewer_proc_trav_final(new visualization::ImageViewer("Image Viewer Traversability final")),
  //image_viewer_proc_trav_final_appear(new visualization::ImageViewer("Image Viewer Traversability Appearance-based final"))
  //,
  comp(new libsegmentation::GroundPlaneComparator<PointT, Normal>),
  segm(comp)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{
  trigger = true;
  continuous = false;
  display_normals = false;
  display_plane_normals = false;
//  display_all = false;

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

  viewer_disparity_processed->setBackgroundColor(0, 0, 0);
  viewer_disparity_processed->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disparity_processed->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_disparity_processed->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_proc_invalid_points->setBackgroundColor(0, 0, 0);
  viewer_proc_invalid_points->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_proc_invalid_points->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);


  viewer_proc_segm->setBackgroundColor(0, 0, 0);
  viewer_proc_segm->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_proc_segm->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_proc_segm->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_proc_segm_appear->setBackgroundColor(0, 0, 0);
  viewer_proc_segm_appear->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_proc_segm_appear->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_proc_segm_appear->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_proc_trav->setBackgroundColor(0, 0, 0);
  viewer_proc_trav->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_proc_trav->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  viewer_proc_trav->registerPointPickingCallback(&tStereoProcessing::pointPickingcallback, *this, (void*)&cb_args);

  viewer_proc_trav_slopeAnalysis->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_slopeAnalysis->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_slopeAnalysis->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_slopeAnalysis_appear->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_slopeAnalysis_appear->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_slopeAnalysis_appear->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  /*
  viewer_proc_trav_slopeMapping_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_slopeMapping_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_slopeMapping_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_stepMapping_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_stepMapping_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_stepMapping_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  */

  viewer_proc_trav_stepAnalysis->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_stepAnalysis->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_stepAnalysis->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_stepAnalysis_appear->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_stepAnalysis_appear->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_stepAnalysis_appear->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  /*
  viewer_proc_trav_slopeFusion_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_slopeFusion_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_slopeFusion_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_stepFusion_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_stepFusion_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_stepFusion_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  */


  viewer_proc_trav_geomFusion_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_geomFusion_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_geomFusion_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_geomFusionMapping_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_geomFusionMapping_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_geomFusionMapping_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_geomFusionMapping_discrete_uh->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_geomFusionMapping_discrete_uh->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_geomFusionMapping_discrete_uh->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  /*
  viewer_proc_trav_stepFusion_discrete->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_stepFusion_discrete->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_stepFusion_discrete->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);



  viewer_proc_trav_final->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_final->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_final->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);

  viewer_proc_trav_final_appear->setBackgroundColor(0, 0, 0);
  viewer_proc_trav_final_appear->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer_proc_trav_final_appear->registerKeyboardCallback(&tStereoProcessing::keyboardCallback, *this, 0);
  */
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
}
