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
/*!\file    projects/icarus/sensor_processing/pointCloudProcessing/StereoVision/mPointCloudProc.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-14
 *
 * \brief Contains mPointCloudProc
 *
 * \b mPointCloudProc
 *
 * this is offline point cloud processing module.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__pointCloudProcessing__StereoVision__mPointCloudProc_h__
#define __projects__icarus__sensor_processing__pointCloudProcessing__StereoVision__mPointCloudProc_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //read and write

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/io/point_cloud_image_extractors.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


// templates
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
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
namespace pointCloudProcessing
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this is offline point cloud processing module.
 */
class mPointCloudProc : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<double> static_parameter_1;   //Example for a static parameter. Replace or delete it!
  tParameter<double> par_parameter_1;   //Example for a runtime parameter named "Parameter 1". Replace or delete it!
  tInput<double> in_signal_1;   //Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
  tOutput<double> out_signal_1;   //Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!

  tOutput<rrlib::coviroa::tImage> output_img;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mPointCloudProc(core::tFrameworkElement *parent, const std::string &name = "PointCloudProc"); //const std::vector<std::string> left_images, const int frames_number_total
  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);
  void processCloud(const CloudConstPtr& cloud);
  void run();
  void pcd2tImage();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_test;
  boost::mutex cloud_mutex;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  CloudConstPtr prev_ground_image_test; //ransac
  Eigen::VectorXf prev_ground_normal;
  Eigen::VectorXf prev_ground_centroid;


  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;

  //  std::vector<std::string> right_images;
  std::vector<std::string> left_images;
  int images_idx;
  int frames_number_total;

  bool trigger;
  bool continuous;
  bool display_normals;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mPointCloudProc();

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



#endif
