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
/*!\file    projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-05
 *
 * \brief Contains mStereoProcessing
 *
 * \b mStereoProcessing
 *
 * This module is processing the stereo pair images for point cloud generation and 3D reconstruction of left image.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__stereo_processing__mStereoProcessing_h__
#define __projects__stereo_traversability_experiments__stereo_processing__mStereoProcessing_h__

#include "plugins/structure/tModule.h"

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

#include "rrlib/coviroa/tImage.h"
#include "rrlib/math/tPose3D.h"

#include "rrlib/mapping/definitions.h"

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
namespace stereo_processing
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
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module is processing the stereo pair images for point cloud generation and 3D reconstruction of left image.
 */
class mStereoProcessing : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  tStaticParameter<double> static_parameter_1;
//  Example for a static parameter. Replace or delete it!
//
//  tParameter<double> par_parameter_1;
//Example for a runtime parameter named "Parameter 1". Replace or delete it!

//  tInput<double> in_signal_1;
//Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tInput<double> in_signal_2;
//
//tOutput<double> out_signal_1;
//Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tOutput<double> out_signal_2;

  tInput<vector<rrlib::coviroa::tImage>> in_images;
  tInput<rrlib::math::tPose3D> si_ugv_chassi_pose;

  tOutput<rrlib::mapping::tMapGridCartesian2D<double>> out_gridmap;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStereoProcessing(core::tFrameworkElement *parent, const std::string &name = "StereoProcessing");

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mStereoProcessing();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!
  Mat input_images_left, input_images_right;
  visualization::PCLVisualizer::Ptr viewer;
  visualization::ImageViewer::Ptr image_viewer, image_viewer_disparity, image_viewer_debugging;

  Mat left_img_rect, right_img_rect, P2_calib;
  Point pt_top_left_crop;

  libstereo::tTestACSO stereo;
  int smooth_weak, smooth_strong;

  PointCloud<RGB>::Ptr prev_img_disp;
  CloudPtr prev_cloud_disp, prev_cloud;
  float u_c, v_c, focal, baseline;

  libfeature::tIntegralImageNormalEstimation<PointT, Normal> ne;
  PointCloud<Normal>::ConstPtr prev_normal_cloud;
  bool display_normals = false;

  libsegmentation::GroundPlaneComparator<PointT, Normal>::Ptr comp;
  libsegmentation::OrganizedConnectedComponentSegmentation<PointT, Label> segm;
  CloudPtr prev_cloud_segm;
  PointCloud<Label>::Ptr prev_cloud_segments_labels;
  vector<PointIndices> prev_cloud_segments;
  unsigned thresh_segments = 100; //1000 for 1300 * 1000 resolution

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;

  vector<Eigen::VectorXf> prev_cloud_planes;
  vector<PointXYZ> prev_cloud_planes_cent_pt;

  CloudPtr prev_cloud_trav_slopeAnalysis;
  vector<unsigned int> prev_segm_sizes_trav,
         prev_segm_sizes_semi,
         prev_segm_sizes_non,
         prev_segm_sizes;

  CloudPtr prev_cloud_trav_dominGroundPlane;
  bool prev_frame_isBad;
  Eigen::VectorXf prev_dominant_ground_plane, prev_dominant_obstacle_plane;
  PointXYZ prev_dominant_ground_plane_cent;

  CloudPtr prev_cloud_trav_stepAnalysis,
           prev_cloud_trav_final;



  //LUGV
  //Dimensions in meters
  float length = 3.3,
        width = 2,
        height = 1.8; //meters
//  float ground_clearance = 5; //not given???
  float weight = 2100; //kg approximately
  float max_speed = 25, //km/h //1000/3600 //10/36 = 5/18
        max_speed_meterPerSec = 7; // meter/ second
  //maneuverability
  float max_gap_crossing = 0.6, //meter
        max_height_crossing = 0.3, //meter
        max_slope = 45; //meter
  //environment conditions
  float operational_temperature_min = -10, //°C
        operational_temperature_max = 50; //°C

  // recently added parameters
  //overhanging obstacles
  float min_passage_height = 3; //this is by me


  /*
    //SUGV
    //Dimensions in meters
    float length = 1.05, //meter
        width = 0.45; //meter
  //float     height = 1.8; //not given????
    float ground_clearance = 0.05; //cm
    float weight = 48; //kg approximately
    float max_speed = 2, //km/h //1000/3600 //10/36 = 5/18
        max_speed_meterPerSec = 0.56; // meter/ second
    //maneuverability
    float max_gap_crossing = 0.4; //meter
  //  flaot   max_height_crossing = 0.3; //meter //NOT GIVEN???
  float     max_slope = 45; //meter
  float operating_time = 2; //hours //120 min
  //environment conditions
  float operational_temperature_min = -20, //°C
      operational_temperature_max = 45; //°C
  */


  /*
    //Traversability parameters
    float cos_slope_traversable = cos(deg2rad(max_slope)),
          cos_slope_non_traversable = cos(deg2rad(90 - max_slope)); //max slope climbing
    float max_traversable_step = max_height_crossing; //max step or height crossing
  */



  virtual void OnStaticParameterChange() override;
//  Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;
//  Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;


  void keyboardCallback(const visualization::KeyboardEvent& event, void*);
  void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* args);

  void stereo_reconst();
  void stereo_rectify();
  void stereo_computePointCloud();
  void stereo_getPointCloudParams();
  void run_visualize();
  void processCloud_normals();
  void processCloud_segm();
  void run_initialize();

  void processCloud_trav();
  void processCloud_trav_segm2plane();
  void processCloud_trav_slopeAnalysis();
  void processCloud_trav_dominGroundPlane();
  void processCloud_trav_stepAnalysis();


//  rrlib::mapping::tMapGridCartesian2D<double>* gridmap;
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
