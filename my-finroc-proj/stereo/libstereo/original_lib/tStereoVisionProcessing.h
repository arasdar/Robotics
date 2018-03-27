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
/*!\file    projects/icarus/sensor_processing/libstereo/tStereoVisionProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 * \brief   Contains tStereoVisionProcessing
 *
 * \b tStereoVisionProcessing
 *
 * This is StereoVisionProcessing class for processing two stereo images using stereo matching algorithms.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo__tStereoVisionProcessing_h__
#define __projects__icarus__sensor_processing__libstereo__tStereoVisionProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/io/io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
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
namespace libstereo
{

using namespace std;
using namespace pcl;
using namespace cv;

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
 * This is StereoVisionProcessing class for processing two stereo images using stereo matching algorithms.
 *
 * In this, we will rectify images using calibration parameters, apply stereo matching on stere images and generate the point cloud.
 */
class StereoVisionProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  StereoVisionProcessing();

  virtual ~StereoVisionProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  virtual void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  stereo_rectify(const string input_images_left, const string input_images_right);

  PointCloud<RGB>::Ptr
  img2pcd(const Mat image_rect);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //----------------------------------------------------------------------
  // Protected fields and methods
  //----------------------------------------------------------------------
protected:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx = 0;
  int img_pairs_num;
  bool trigger;
  bool continuous;

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename;
  string input_extrinsic_filename ;
  Mat left_img_rect;
  Mat right_img_rect;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
