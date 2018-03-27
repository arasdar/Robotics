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
/*!\file    projects/icarus/sensor_processing/libsegmentation/tPointCloudProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-30
 *
 * \brief   Contains tPointCloudProcessing
 *
 * \b tPointCloudProcessing
 *
 * This is an application for processing point clouds to classify terrain based on traversability estimation.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libsegmentation__tPointCloudProcessing_h__
#define __projects__icarus__sensor_processing__libsegmentation__tPointCloudProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
/*// output*/
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/pcd_io.h> //input
#include <pcl/common/centroid.h> ////computeMeanAndCovarianceMatrix

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

/*normal features*/
#include "projects/icarus/sensor_processing/libfeature/tIntegralImageNormalEstimation.h"

/*segmentation*/
#include "projects/icarus/sensor_processing/libsegmentation/tGroundPlaneComparator.h"
#include "projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace libsegmentation
{

using namespace std;
using namespace pcl;
using namespace finroc::icarus::sensor_processing::libsegmentation;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is an application for processing point clouds to classify terrain based on traversability estimation.
 */
class PointCloudProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  PointCloudProcessing(const std::vector<std::string> left_images, const int frames_number_total);

  ~PointCloudProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  inline void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  inline void
  processCloud(const CloudConstPtr& cloud);

  void run();

  void processCloud_orig(const CloudConstPtr& cloud);
  void run_orig();



//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
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
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;
  CloudConstPtr prev_ground_image_test; //ransac

  libfeature::tIntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;

  std::vector<std::string> left_images;
  int files_idx;
  int images_idx;
  int frames_number_total;

  bool trigger;
  bool continuous;
  bool display_normals;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
