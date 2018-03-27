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
/*!\file    projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-05
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/coviroa/opencv_utils.h"

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
namespace stereo_processing
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStereoProcessing> cCREATE_ACTION_FOR_M_STEREOPROCESSING("StereoProcessing");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mStereoProcessing constructor
//----------------------------------------------------------------------
mStereoProcessing::mStereoProcessing(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false),
  viewer(new visualization::PCLVisualizer("3D viewer")),
  image_viewer(new visualization::ImageViewer("image viewer")),
  image_viewer_disparity(new visualization::ImageViewer("image viewer disparity")),
  image_viewer_debugging(new visualization::ImageViewer("debugging visualization")),
  comp(new libsegmentation::GroundPlaneComparator<PointT, Normal>),
  segm(comp),
  prev_cloud_segments_labels(new PointCloud<Label>)
//  gridmap(new rrlib::mapping::tMapGridCartesian2D<double>)
// change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
//  If you have some member variables, please initialize them here. Especially built - in types(like pointers!). Delete this line otherwise!
{
  viewer->registerKeyboardCallback(&mStereoProcessing::keyboardCallback, *this, 0);
  viewer->registerPointPickingCallback(&mStereoProcessing::pointPickingCallback, *this, (void*)&cb_args);
}

//----------------------------------------------------------------------
// mStereoProcessing destructor
//----------------------------------------------------------------------
mStereoProcessing::~mStereoProcessing()
{}

//----------------------------------------------------------------------
// mStereoProcessing OnStaticParameterChange
//----------------------------------------------------------------------
void mStereoProcessing::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    As this static parameter has changed, do something with its value!
//    }
}

//----------------------------------------------------------------------
// mStereoProcessing OnParameterChange
//----------------------------------------------------------------------
void mStereoProcessing::OnParameterChange()
{
//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mStereoProcessing Update
//----------------------------------------------------------------------
void mStereoProcessing::Update()
{
//  if (this->InputChanged())
//  {
//    At least one of your input ports has changed. Do something useful with its data.
//    However, using the .HasChanged() method on each port you can check in more detail.
//  }
//
//  Do something each cycle independent from changing ports.
//
//  this->out_signal_1.Publish(some meaningful value);
//  can be used to publish data via your output ports.

  // Grabing images from tFrameGrabber
  data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> input_images = in_images.GetPointer();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "input_images->size(): ", input_images->size());

  //data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap = out_gridmap.GetUnusedBuffer();
  output_gridmap = out_gridmap.GetUnusedBuffer();
  rrlib::mapping::tMapGridCartesian2D<double>& map = *output_gridmap;

  //  std::cout << "Bounds: " << map.GetBounds()[0] << ", " << map.GetBounds()[1] << ", " << map.GetBounds()[2] << ", " << map.GetBounds()[3] << std::endl;
  std::cout << "Number of cells: " << map.GetNumberOfCells() << std::endl;
  map.SetResolution(rrlib::math::tVec2d(0.05, 0.05)); // resolution: 1 meter, 1 meter
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds bounds;
  bounds.upper_bounds[0] = rrlib::math::tVec2d(10, 0); //10 meters range
  bounds.lower_bounds[0] = rrlib::math::tVec2d(-10, 0);
  bounds.upper_bounds[1] = rrlib::math::tVec2d(0, 10);
  bounds.lower_bounds[1] = rrlib::math::tVec2d(0, -10);
  map.SetBounds(bounds);
  map.RecalculateSize();


  // when enabled
  if (input_images->size() > 0)
  {
    input_images_left = rrlib::coviroa::AccessImageAsMat(input_images->at(0));
    input_images_right = rrlib::coviroa::AccessImageAsMat(input_images->at(1));

    run_initialize();

    stereo_reconst();
    processCloud_normals();
    processCloud_segm();
    processCloud_trav();

    run_visualize();
  }

  out_gridmap.Publish(map);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
