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
/*!\file    projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-15
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.h"

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
namespace stereo_sugv
{
namespace capture
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStereo> cCREATE_ACTION_FOR_M_STEREO("Stereo");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mStereo constructor
//----------------------------------------------------------------------
mStereo::mStereo(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
  ,
  image_viewer(new visualization::ImageViewer("Image Viewer")),
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
  ,
  flycap_img(new Image),
  flycap_img_2(new Image)
  ,
  road_comparator(new GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator),
  nominal_road_normal(0.0, -1.0, 0.0)
{
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  //changing the resolution
  imageSettings.offsetX = 0;
  imageSettings.offsetY = 0;
  imageSettings.height = 900; //480;  //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
  imageSettings.width = 1200; //640; //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
  imageSettings.pixelFormat = PIXEL_FORMAT_MONO8;

  //stereo_capture -- RunSingleCamera(pointgrey_guid);
  bus_manager.GetCameraFromIndex(0, &pointgrey_guid); //left eth1
  camera.Connect(&pointgrey_guid);
  camera.SetGigEImageSettings(&imageSettings);
  camera.StartCapture();

  bus_manager.GetCameraFromIndex(1, &pointgrey_guid); //right eth2
  camera_2.Connect(&pointgrey_guid);
  camera_2.SetGigEImageSettings(&imageSettings);
  camera_2.StartCapture();

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

  /*! AdaptiveCostSOStereoMatching*/
  stereo.setXOffset(50); //dmin=50, zmax=5 scale=250
  stereo.setMaxDisparity(200); //dmax=250 //dnum=200 zmin=1 scale=250

  /*
  stereo.setMaxDisparity(90); //original
  stereo.setXOffset(10); //original  //dmin = scale/zmax and dmax = scale/zmin ===> scale = 250, Zmax = 25 meters, Zmin= 250/100 ~ 2.5

   stereo.setXOffset(25); //zmax=10 meters
   stereo.setMaxDisparity(225); //zmin=1, dmax=250 dmin=25, dnum=225

    stereo.setXOffset(5); //zmax = 50 meters ==> 250/50 = 5
    stereo.setXOffset(2); //zmax = 250/2=125 meters and dmax = 62 ==> zmin=250/62=4 meters
  */



  stereo.setRadius(5); //original

  smooth_weak = 20;
  smooth_strong = 100;
  stereo.setSmoothWeak(smooth_weak); //20 original
  stereo.setSmoothStrong(smooth_strong); //original
  stereo.setGammaC(25); //original
  stereo.setGammaS(10); //10 original was original

  stereo.setPreProcessing(true);

  /*Set up normal extraction*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

  bool use_depth_dependent_smoothing = false;
  ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

  /*// Set up segmentation using the ground plane comparator -- If the camera was pointing straight out, the normal would be:*/
  tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(25.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
  road_comparator->setExpectedGroundNormal(tilt_road_normal);

  float ground_angular_threshold = pcl::deg2rad(30.0f);
  road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original

  float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
  road_comparator->setAngularThreshold(angular_threshold); //3.0f original

  /*
    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);
  */
}

//----------------------------------------------------------------------
// mStereo destructor
//----------------------------------------------------------------------
mStereo::~mStereo()
{
  // Stop capturing images
  camera.StopCapture();
  camera_2.StopCapture();

  // Disconnect the camera
  camera.Disconnect();
  camera_2.Disconnect();
}

//----------------------------------------------------------------------
// mStereo OnStaticParameterChange
//----------------------------------------------------------------------
void mStereo::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    /*As this static parameter has changed, do something with its value!*/
//  }
}

//----------------------------------------------------------------------
// mStereo OnParameterChange
//----------------------------------------------------------------------
void mStereo::OnParameterChange()
{
  /*If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.*/
}

//----------------------------------------------------------------------
// mStereo Update
//----------------------------------------------------------------------
void mStereo::Update()
{
  if (this->InputChanged())
  {
    /*At least one of your input ports has changed. Do something useful with its data.
    However, using the .HasChanged() method on each port you can check in more detail.*/
  }

  /*Do something each cycle independent from changing ports.*/
  run();
  gridmap();
  secletmap();

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
