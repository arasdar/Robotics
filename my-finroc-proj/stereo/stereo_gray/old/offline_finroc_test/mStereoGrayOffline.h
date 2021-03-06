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
/*!\file    projects/icarus/sensor_processing/stereo_gray/offline/finroc/mStereoGrayOffline.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-15
 *
 * \brief Contains mStereoGrayOffline
 *
 * \b mStereoGrayOffline
 *
 * This is the module for stereo gray offline processing using ACSO stereo matching.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__stereo_gray__offline__finroc__mStereoGrayOffline_h__
#define __projects__icarus__sensor_processing__stereo_gray__offline__finroc__mStereoGrayOffline_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>

/*! terrain features*/
//#include <pcl/features/integral_image_normal.h>
//#include "projects/icarus/sensor_processing/libfeature/integral_image_normal.h"
#include "projects/icarus/sensor_processing/libfeature/tIntegralImageNormalEstimation.h"

#include <pcl/common/centroid.h> ////computeMeanAndCovarianceMatrix
#include <pcl/sample_consensus/sac_model_plane.h> //pointToPlaneDistance

#include "projects/icarus/sensor_processing/libstereo_test/tTestACSO.h"

/*! traversability using segmentation*/
#include "projects/icarus/sensor_processing/libsegmentation/tGroundPlaneComparator.h"
#include "projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <pcl/io/point_cloud_image_extractors.h> //fingui ??

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
#include "rrlib/mapping/definitions.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace stereo_gray
{
namespace offline_test
{

using namespace std;
using namespace pcl;
using namespace cv;
using namespace libsegmentation;

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
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the modeul for stereo gray offline processing using ACSO stereo matching.
 */
class mStereoGrayOffline : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tOutput<rrlib::coviroa::tImage> left_img, right_img;
  tOutput<rrlib::coviroa::tImage> output_rect_stereo;
  tOutput<rrlib::coviroa::tImage> output_disparity_img;
  tOutput<rrlib::coviroa::tImage> output_img;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStereoGrayOffline(core::tFrameworkElement *parent, const std::string &name = "StereoGrayOffline");

  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  pointPickingcallback(const pcl::visualization::PointPickingEvent& event, void* args);

  void
  stereo_rectify(const string input_images_left, const string input_images_right);

  void cropping(Mat* img_rect, const Rect* validRoi);

  void
  processCloud(const CloudConstPtr& cloud);

  void processCloud(const CloudConstPtr& cloud, const CloudConstPtr& cloud_disp);

  void processCloud_test(const CloudConstPtr& cloud, const CloudConstPtr& cloud_disp);

  void processSmallSegments();

  //void filterCloudOutlierRemoval();

  //void filterCloudRadius();

  void drawVisualization();

  void run();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity_processed;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disp_proc;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_original;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_processed;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_right;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_original;

  boost::mutex cloud_mutex;
  CloudConstPtr prev_cloud;
  CloudPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  CloudConstPtr prev_disp_image;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;

  libfeature::tIntegralImageNormalEstimation<PointT, pcl::Normal> ne;

  GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;

  /*! Show the ground plane normal*/
  Eigen::Vector3f nominal_road_normal; //-1 default  x,y,z --> (z is depth here)

  /*! Adjust for camera tilt*/
  Eigen::Vector3f tilt_road_normal;


  std::vector<pcl::PointIndices> prev_region_indices;
  bool prev_enoughPoints = false;
  float prev_ground_ymin = 0, prev_ground_ymax = 0;
  std::vector<pcl::ModelCoefficients> prev_model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> prev_centroids;
  unsigned prev_ground_size = 0;
  float step_max = 0.10;



  Eigen::VectorXf prev_ground_normal; //vecotr4f originally which was buggy
  Eigen::VectorXf prev_ground_centroid;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool bwd; //trigger backward
  bool continuous;
  bool display_normals;

  /*! Stereo capture, rectification initialization*/
  string input_intrinsic_filename;
  string input_extrinsic_filename;
  Mat left_img_rect;
  Mat right_img_rect;
  Mat P2_calib;
  Point pt_top_left_crop;

  libstereo_test::tTestACSO stereo;
  int smooth_weak;
  int smooth_strong;

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mStereoGrayOffline();

//  virtual void OnStaticParameterChange();   //Might be needed to process static parameters. Delete otherwise!
//
//  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

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
