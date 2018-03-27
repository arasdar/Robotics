//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/simulation/mStViPr.h
 *
 * \author  Aras Dargazany
 *
 * \date    2013-04-26
 *
 * \brief Contains mStViPr
 *
 * \b mStViPr
 *
 * this is sense and control module for stereo vision processing (StViPr)
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__physics__mStViPr_h__
#define __projects__icarus__physics__mStViPr_h__

#include "plugins/structure/tSenseControlModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
//#include <opencv/cv.h>  //flann error!!!!!!!!!!!
#include "rrlib/distance_data/tDistanceData.h"
//
////----------------------------------------------------------------------
//// Internal includes with ""
////----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
#include "projects/icarus/simulation_physics/mStViPr.hpp"
#include "rrlib/mapping/definitions.h"
#include "rrlib/canvas/tCanvas2D.h"


//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace simulation_physics
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this is sensor and control module for stereo vision processing
 */
class mStViPr : public structure::tSenseControlModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /*-----------> stereo vision processing <--------------*/
  tSensorInput<std::vector<rrlib::coviroa::tImage>> si_image;

  /* -----------> sector map <---------------*/
  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data_front_scanner;
  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data_rear_scanner;

  /*----------> grid map <-----------------*/
  tSensorOutput<rrlib::mapping::tMapGridCartesian2D<double>> so_map_grid_stereo;
  tSensorOutput<rrlib::mapping::tMapGridCartesian2D<double>> so_map_grid_stereo_env;
  tSensorInput<rrlib::canvas::tCanvas2D> si_canvas_map_env_stereo;
  tSensorOutput<rrlib::canvas::tCanvas2D> so_canvas_map_env_stereo;

  /*! reseting the grid map*/
  tControllerInput<double> ci_reset_grdimap_stereo_env;




//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStViPr(core::tFrameworkElement *parent, const std::string &name = "StViPr");

  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);
  PointCloud<RGB>::Ptr png2pcd(const Mat rect_png_image);
  void stereo_rectify(const Mat input_png_images_left, const Mat input_png_images_right);
  void processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, CloudPtr& out_cloud);
  void processCloud(const CloudConstPtr& cloud);
  void processCloud_new(const CloudConstPtr& cloud);
  void processCloud_new_test(const CloudConstPtr& cloud);
  void saveCloud(const CloudConstPtr& cloud);
  void run();

  void chessboard_detection();
  void saving_frames_right_left_folders(const Mat imgl, const Mat imgr);
  void display_images();

  void gridmap_stereo();

  virtual ~mStViPr();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private: //should be initialized in constructor!!!

  /*
   *   //Here is the right place for your variables. Replace this line by your declarations!
   */
  // stereo input from environment
  IplImage* input_iplimage_left;
  IplImage* input_iplimage_right;

  // stereo vision processing
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_original;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_2;
  boost::mutex cloud_mutex;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int images_idx_saveImage;
  int n_r ;
  int n_l ;
  int img_pairs_num;
  bool trigger;
  bool continuous;
  bool display_normals;
  bool detect_obstacles;

  // stereo capture, initialization
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename = "etc/calib_intrinsics.yml";
  string input_extrinsic_filename = "etc/calib_extrinsics.yml";
  Mat left_rect_png_img;
  Mat right_rect_png_img;

  //stereo matching
  pcl::AdaptiveCostSOStereoMatching stereo;

  // save cloud parameters
  pcl::PCDWriter writer_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;

  /*
   * gridmap for stereo
   */
  rrlib::mapping::tMapGridCartesian2D<double> map_stereo_env;
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds bounds_stereo_env;


  virtual void OnStaticParameterChange();   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Sense();

  virtual void Control();

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
