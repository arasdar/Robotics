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
/*!\file    projects/icarus/sensor_processing/kinect/capture/finroc/mKinectCapture.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-15
 *
 * \brief Contains mKinectCapture
 *
 * \b mKinectCapture
 *
 * This is the module for capturing kinect point clouds using openni device and proecssing the point cloud real-time.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__kinect__capture__finroc__mKinectCapture_h__
#define __projects__icarus__sensor_processing__kinect__capture__finroc__mKinectCapture_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
/** \brief kinectTraversability is an application for processing point clouds to classify terrain based on traversability estimation using kinect.
  *
  * \author Aras Dargazany
  */

#include <pcl/io/io.h>

// FIXME: openni relies on the non-standard "i386" macro GCC used to define
#if __i386__
#define i386 1
#endif
//#include <XnCppWrapper.h>  //this one is from rrlib/openni/tDevice
#include <pcl/io/openni_grabber.h>
#undef i386

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h> //computeMeanAndCovarianceMatrix
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h> //planeRefinementComparator
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_plane.h> //pointToPlaneDistance

//#include <pcl/visualization/image_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h> //PCDwriter

#include <pcl/io/point_cloud_image_extractors.h>

//template
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;


using namespace std;
using namespace pcl;

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/mapping/definitions.h"
#include "rrlib/coviroa/tImage.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace kinect
{
namespace capture
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the module for capturing kinect point clouds using openni device and proecssing the point cloud real-time.
 */
class mKinectCapture : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tOutput<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap;
  tOutput<rrlib::coviroa::tImage> output_img;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mKinectCapture(core::tFrameworkElement *parent, const std::string &name = "KinectCapture");

  void
  cloud_cb_(const CloudConstPtr& cloud);

  CloudConstPtr
  getLatestCloud();

  void processCloud(const CloudConstPtr& cloud);

//  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void saveCloud(const CloudConstPtr& cloud);

  void gridmap();

  void pcd2img();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  CloudConstPtr cloud_;
  mutable boost::mutex cloud_mutex_;

  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber grabber_; //("#1", depth_mode, image_mode)

  //processCloud
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;

  boost::mutex cloud_mutex; //this is prolly included in visualization library

//  pcl::visualization::ImageViewer::Ptr image_viewer; //(new pcl::visualization::ImageViewer ("test"))
//  pcl::visualization::PCLVisualizer::Ptr viewer;
//  pcl::visualization::ImageViewer::Ptr image_viewer_debugging;

  bool display_normals = false;

  pcl::PCDWriter writer_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;
  int images_idx = 0;

  const int bound_limit = 10; //100; //500 //80 //40 //20
  const double cell_resolution = 0.05; //1; //0.13; //0.05; //meter //0.1 //1
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds bounds_stereo_env;

  // displaying the dominant normal and deviation of the dominant plane from nominal ground normal
  Eigen::VectorXf prev_ground_normal;  //Vector4f this was buggy
  Eigen::VectorXf prev_ground_centroid; //Vector4f


  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mKinectCapture();

  virtual void OnStaticParameterChange();   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}



#endif
