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
/*!\file    projects/stereo_traversability_experiments/openTraverse/aras/tSSTA.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-10-21
 *
 * \brief   Contains tSSTA
 *
 * \b tSSTA
 *
 * This is the Superpixel Surface Traversability Analysis (SSTA) module.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__openTraverse__aras__tSSTA_h__
#define __projects__stereo_traversability_experiments__openTraverse__aras__tSSTA_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/io/io.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libfeature/tIntegralImageNormalEstimation.h"

#include "projects/stereo_traversability_experiments/aras/libsegmentation/tGroundPlaneComparator.h"
#include "projects/stereo_traversability_experiments/aras/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

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

using namespace pcl;
using namespace std;
using namespace stereo_traversability_experiments::aras;

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
 * This is the Superpixel Surface Traversability Analysis (SSTA) module.
 */
class tSSTA
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tSSTA();

  ~tSSTA();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  //Here is the right place for your public methods. Replace this line by your declarations!
  void processCloud_normals(const CloudConstPtr& cloud);
  void processCloud_segm();

  void processCloud_trav();
  void processCloud_trav_segm2plane();
  void processCloud_trav_slopeAnalysis();
  void processCloud_trav_dominGroundPlane();
  void processCloud_trav_stepAnalysis();

  CloudPtr prev_cloud;
  CloudPtr prev_cloud_segm;
  CloudPtr prev_cloud_trav_slopeAnalysis;
  CloudPtr prev_cloud_trav_dominGroundPlane;
  CloudPtr prev_cloud_trav_stepAnalysis;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  libfeature::tIntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  PointCloud<Normal>::ConstPtr prev_normal_cloud;

  libsegmentation::GroundPlaneComparator<PointT, Normal>::Ptr  comp;
  libsegmentation::OrganizedConnectedComponentSegmentation<PointT, Label> segm;
  PointCloud<pcl::Label> prev_cloud_segment_labels;
  vector<pcl::PointIndices> prev_cloud_segments;
  unsigned int thresh_segments = 1000;

  unsigned prev_idx = 0;
//  vector<Eigen::Vector4f> prev_cloud_planes;
  vector<Eigen::VectorXf> prev_cloud_planes;
  vector<PointXYZ> prev_cloud_planes_cent_pt;

  vector<unsigned> prev_segm_sizes_trav;
  vector<unsigned> prev_segm_sizes_semi;
  vector<unsigned> prev_segm_sizes_non;
  vector<unsigned> prev_segm_sizes;

  Eigen::Vector4f prev_dominant_ground_plane, prev_dominant_obstacle_plane;
  bool prev_frame_isBad = false;
  PointXYZ prev_dominant_ground_plane_cent;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
