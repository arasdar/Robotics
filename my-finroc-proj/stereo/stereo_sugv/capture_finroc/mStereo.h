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
/*!\file    projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-15
 *
 * \brief Contains mStereo
 *
 * \b mStereo
 *
 * This is stereo processing module for online capturing.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__stereo_sugv__capture_finroc__mStereo_h__
#define __projects__icarus__sensor_processing__stereo_sugv__capture_finroc__mStereo_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/io/io.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <FlyCapture2.h>

#include "projects/icarus/sensor_processing/libstereo_test/tTestACSO.h"

/*! terrain features*/
//#include <pcl/features/integral_image_normal.h>
#include "projects/icarus/sensor_processing/libfeature/tIntegralImageNormalEstimation.h"
#include <pcl/common/centroid.h> ////computeMeanAndCovarianceMatrix

/*! traversability using segmentation*/
#include "projects/icarus/sensor_processing/libsegmentation/tGroundPlaneComparator.h"
#include "projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentation.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/mapping/definitions.h"
//#include "rrlib/math/"

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

using namespace std;
using namespace pcl;
using namespace cv;
using namespace FlyCapture2;
using namespace libsegmentation;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is stereo processing module for online capturing.
 */
class mStereo : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  tStaticParameter<double> static_parameter_1;   //Example for a static parameter. Replace or delete it!
//
//  tParameter<double> par_parameter_1;   //Example for a runtime parameter named "Parameter 1". Replace or delete it!
//
//  tInput<double> in_signal_1;   //Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tInput<double> in_signal_2;
//
//  tOutput<double> out_signal_1;   //Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tOutput<double> out_signal_2;

  tOutput<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap;
  tOutput<rrlib::mapping::tMapGridPolar2D<double>> output_secletmap;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStereo(core::tFrameworkElement *parent, const std::string &name = "Stereo");

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  IplImage* ConvertImageToOpenCV(Image* pImage);

  void stereoCapture();

  void stereoRect();

  void processCloud(const CloudConstPtr& cloud);
  void run();

  void gridmap();

  void secletmap();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  int images_idx = 0;

  //stereo capture
  BusManager bus_manager;
  PGRGuid pointgrey_guid;

  //Camera camera;
  GigECamera camera;
  Image* flycap_img;

  //Camera camera_2;
  GigECamera camera_2;
  Image* flycap_img_2;

  // changing the resolution of images
  GigEImageSettings imageSettings;

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;

  //stereo_images/stereo_sugv/2_on-sugv/calib_8_good
  string input_intrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/intrinsics.yml",
         input_extrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/extrinsics.yml";
  Mat input_images_left, input_images_right;
  Mat left_img_rect;
  Mat right_img_rect;

  finroc::icarus::sensor_processing::libstereo_test::tTestACSO stereo;
  int smooth_weak;
  int smooth_strong;

  boost::mutex cloud_mutex;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;
  Eigen::Vector3f tilt_road_normal;
  Eigen::Vector3f nominal_road_normal;

  //pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  libfeature::tIntegralImageNormalEstimation<PointT, Normal> ne;
  GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mStereo();

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update(); // override;

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
