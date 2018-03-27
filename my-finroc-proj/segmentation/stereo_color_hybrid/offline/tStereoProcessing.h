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
/*!\file    projects/icarus/sensor_processing/stereo_gray/offline/tStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-08-03
 *
 * \brief   Contains tStereoProcessing
 *
 * \b tStereoProcessing
 *
 * This is the stereo processing module to generate point cloud using stereo images.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__daniel__stereo_color_hybrid__offline__tStereoProcessing_h__
#define __projects__stereo_traversability_experiments__daniel__stereo_color_hybrid__offline__tStereoProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "projects/stereo_traversability_experiments/aras/libstereo/tTestACSO.h"
#include "projects/stereo_traversability_experiments/aras/libstereo/tBlockBasedStereoMatching.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //pcdwriter

#include "projects/stereo_traversability_experiments/aras/libfeature/tIntegralImageNormalEstimation.h"

#include "projects/stereo_traversability_experiments/aras/libsegmentation/tGroundPlaneComparator.h"
#include "projects/stereo_traversability_experiments/aras/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

//----------------------------------------------------------------------
// Internal includes with ""
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

using namespace pcl;
using namespace std;
using namespace cv;
using namespace finroc::stereo_traversability_experiments::aras; // necessary so calls to libstereo functions work

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

struct callback_args
{
  /*structure used to pass arguments to the callback function*/
  Cloud::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_2;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_3;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_4;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_5;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_6;
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the stereo processing module to generate point cloud using stereo image.
 */
class tStereoProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tStereoProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                    const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename);

  ~tStereoProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*! Here is the right place for your public methods. Replace this line by your declarations!*/
  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  pointPickingcallback(const pcl::visualization::PointPickingEvent& event, void* args);

  void stereo_rectify();
  void stereo_rectify_test();
  void stereo_cropping(Mat* img_rect, const Rect* validRoi);
  void stereo_reconst();
  void stereo_getPointCloudParams();
  void stereo_getPointCloudParams_test();
  void stereo_computePointCloud();

  void processCloud_normals(const CloudConstPtr& cloud);
  void processCloud_segm();
  void processCloud_segm_appear();
  void processCloud_segm_slic(std::string image_path);

  void processCloud_trav();
  void processCloud_trav_segm2plane();
  void processCloud_trav_segm2plane_appear();
  void processCloud_trav_slopeAnalysis();
  void processCloud_trav_slopeAnalysis_appear();
  void processCloud_trav_slope_and_step_mapping_discrete();
  void processCloud_trav_geom_fuse_and_map();
  void processCloud_trav_fuse_pessimistically();
  void processCloud_trav_dominGroundPlane();
  void processCloud_trav_stepAnalysis();
  void processCloud_trav_stepAnalysis_appear();

  void run_initialize();
  void run();
  void run_proceed_callbacks();
  void run_proceed_callbacks_test();
  void run_visualize();
  void run_saveCloud(const CloudConstPtr& cloud);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  void processCloud_invalid_points();
  void drawContoursAroundSegments(CloudPtr cloud, PointCloud<pcl::Label>& labels, std::vector<pcl::PointIndices>& segment_indices); // adapted from SLICSuperpixels
  void drawSurfaceNormals(visualization::PCLVisualizer::Ptr viewer, vector<pcl::PointIndices> segment_indices, vector<Eigen::VectorXf> planes, vector<PointXYZ> centroid, unsigned& max_id);
  void removeSurfaceNormals(visualization::PCLVisualizer::Ptr viewer, unsigned& max_id);

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity_processed;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_proc_segm;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_proc_segm_appear;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_proc_trav;
  visualization::PCLVisualizer::Ptr viewer_proc_invalid_points;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_slopeAnalysis;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_slopeAnalysis_appear;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_slopeMapping_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_stepMapping_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_stepAnalysis;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_stepAnalysis_appear;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_slopeFusion_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_stepFusion_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_geomFusion_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_geomFusionMapping_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_geomFusionMapping_discrete_uh;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_finalFusion_discrete;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_final;
  visualization::PCLVisualizer::Ptr viewer_proc_trav_final_appear;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_processed;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_segm;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_segm_appear;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_proc_trav;
  visualization::ImageViewer::Ptr image_viewer_proc_invalid_points;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_slopeAnalysis;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_slopeAnalysis_appear;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_slopeMapping_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_stepMapping_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_stepAnalysis;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_stepAnalysis_appear;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_slopeFusion_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_stepFusion_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_geomFusion_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_geomFusionMapping_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_geomFusionMapping_discrete_uh;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_finalFusion_discrete;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_final;
  visualization::ImageViewer::Ptr image_viewer_proc_trav_final_appear;

  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool bwd; //trigger backward
  bool continuous;
  bool display_normals;
  bool display_plane_normals;
//  bool display_all;

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;

  /*! Stereo capture, rectification initialization*/
  string input_intrinsic_filename;
  string input_extrinsic_filename;
  string input_images_left;
  string input_images_right;
  Mat left_img_rect;
  Mat right_img_rect;
  Mat P2_calib;
  Point pt_top_left_crop;

  libstereo::tTestACSO stereo;
  int smooth_weak;
  int smooth_strong;

//  libstereo::BlockBasedStereoMatching stereo;

  PointCloud<RGB>::Ptr prev_img_disp;
  CloudPtr prev_cloud_disp;
  float u_c, v_c, focal, baseline;

  libfeature::tIntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  PointCloud<Normal>::ConstPtr prev_normal_cloud;
  CloudPtr prev_cloud;
  CloudPtr prev_cloud_invalid_points;

  libsegmentation::GroundPlaneComparator<PointT, Normal>::Ptr  comp;
  libsegmentation::OrganizedConnectedComponentSegmentation<PointT, Label> segm;
  CloudPtr prev_cloud_segm;
  CloudPtr prev_cloud_segm_appear;
  PointCloud<pcl::Label> prev_cloud_segment_labels;
  vector<pcl::PointIndices> prev_cloud_segments;
  PointCloud<pcl::Label> prev_cloud_segment_labels_appear;
  vector<pcl::PointIndices> prev_cloud_segments_appear;
  unsigned int thresh_segments = 1000;

  unsigned prev_idx = 0;
  unsigned prev_idx_appear = 0;
  unsigned prev_idx_normal = 0;

//  vector<Eigen::Vector4f> prev_cloud_planes;
  vector<Eigen::VectorXf> prev_cloud_planes;
  vector<PointXYZ> prev_cloud_planes_cent_pt;
  vector<Eigen::VectorXf> prev_cloud_planes_appear;
  vector<PointXYZ> prev_cloud_planes_cent_pt_appear;

  CloudPtr prev_cloud_trav_slopeAnalysis;
  CloudPtr prev_cloud_trav_slopeAnalysis_appear;
  CloudPtr prev_cloud_trav_slopeMapping_discrete;
  CloudPtr prev_cloud_trav_stepMapping_discrete;
  CloudPtr prev_cloud_trav_slopeFusion_discrete;
  CloudPtr prev_cloud_trav_stepFusion_discrete;
  CloudPtr prev_cloud_trav_finalFusion_discrete;
  CloudPtr prev_cloud_trav_geomFusion_discrete;
  CloudPtr prev_cloud_trav_fusedMapping_discrete;
  CloudPtr prev_cloud_trav_fusedMapping_discrete_uh;
  vector<unsigned> prev_segm_sizes_trav;
  vector<unsigned> prev_segm_sizes_semi;
  vector<unsigned> prev_segm_sizes_non;
  vector<unsigned> prev_segm_sizes;


  CloudPtr prev_cloud_trav_dominGroundPlane;
  Eigen::Vector4f prev_dominant_ground_plane, prev_dominant_obstacle_plane;
  bool prev_frame_isBad = false;
  PointXYZ prev_dominant_ground_plane_cent;

  CloudPtr prev_cloud_trav_stepAnalysis;
  CloudPtr prev_cloud_trav_stepAnalysis_appear;
  CloudPtr prev_cloud_trav_final;
  CloudPtr prev_cloud_trav_final_appear;

  /*save cloud parameters*/
  PCDWriter writer_;
  std::string file_name_ = "left";
  std::string dir_name_ = "/home/aras/Desktop/frames_pcd_acso";
//  unsigned format_ = 1; //binary
  unsigned format_ = 2; //compressedBinary


  // parameter values
  float thresh_slope_upper = 0.7;
  float thresh_slope_lower = 0.3;
  float thresh_step_above_ground_upper = 0.3;
  float thresh_step_above_ground_lower = 0.15;
  float thresh_step_below_ground = -0.2;
  float thresh_uncertainty = 0.5;
  float thresh_mapping = 0.2;
  float weight_non = 4.0;
  float weight_semi = 2.0;
  float weight_trav = 1.0;
  float invalid_slope = std::numeric_limits<float>::min();
  float invalid_step = std::numeric_limits<float>::min();
  unsigned invalid_label = std::numeric_limits<unsigned>::max();
  unsigned code_unknown = 4;
  unsigned code_trav = 3;
  unsigned code_semi = 2;
  unsigned code_non = 1;
  unsigned code_invalid = 0;

  // vectors to hold intermediate results
  vector<unsigned> segments_slope_appear_discrete; // holds trav class acc. to slope for each app.based segment
  vector<unsigned> segments_step_appear_discrete; // holds trav class acc. to step for each app.based segment
  vector<unsigned> segments_slope_geom_discrete; // holds trav class acc. to slope for each geom.based segment
  vector<unsigned> segments_step_geom_discrete; // holds trav class acc. to step for each geom.based segment
  vector<unsigned> segments_fused_geom_discrete; // holds trav class acc. to fusion of slope and step for each geom.based segment
  vector<unsigned> segments_mapped_geom_discrete; // holds trav class after fusion and mapping for each geom.based segment
  vector<unsigned> segments_mapped_geom_discrete_unknown_handled; // code_unknown replaced by code from app.based analysis
  vector<unsigned> segments_slope_mapped_discrete; // holds trav class acc. to slope for each mapped (from geom. to app.) segment
  vector<unsigned> segments_step_mapped_discrete;  // holds trav class acc. to step for each mapped (from geom. to app.) segment
  vector<unsigned> segments_step_fused_discrete; // holds trav class acc. to step fusion (app. and mapped app.) for eavh app. segment
  vector<unsigned> segments_slope_fused_discrete; // holds trav class acc. to slope fusion (app. and mapped app.) for eavh app. segment
  vector<unsigned> segments_final_fused_discrete; // holds trav class acc. to slope and step fusion (each fused from app. and mapped app.) for each app. segment
  vector<float> segments_geom_slope; // holds slope for each geom. based segment
  vector<float> segments_geom_step; // holds step for each geom. based segment
  vector<float> segments_appear_slope; // holds slope for each app. based segment
  vector<float> segments_appear_step; // holds step for each app. based segment
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
