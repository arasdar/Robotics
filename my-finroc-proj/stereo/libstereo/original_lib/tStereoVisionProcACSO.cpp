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
/*!\file    projects/icarus/sensor_processing/libstereo/tStereoVisionProcACSO.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tStereoVisionProcACSO.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tStereoVisionProcACSO constructors
//----------------------------------------------------------------------
StereoVisionProcACSO::StereoVisionProcACSO(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
    const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{

  this->left_images = left_images;
  this->right_images = right_images;
  this->img_pairs_num = img_pairs_num;
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  /*! initialization for AdaptiveCostSOStereoMatching*/
  smooth_weak = 20;
  smooth_strong = 100;
  stereo.setSmoothWeak(smooth_weak); //20 original
  stereo.setSmoothStrong(smooth_strong); //original
  stereo.setGammaC(25); //original
  stereo.setGammaS(10); //10 original was original

  stereo.setMaxDisparity(60); //original
  stereo.setXOffset(0); //setter for horizontal offset, i.e. number of pixels to shift the disparity range over the target image
  stereo.setRadius(5); //original

//  /*
//   * setter for the value of the ratio filter
//   * Parameters:
//   * [in] ratio_filter  value of the ratio filter; it is a number in the range [0, 100] (0: no filtering action; 100: all disparities are filtered)
//   *
//   */
//  /**
//    *
//    * * postprocessing: Ratio Filter (eliminating generic matching ambiguities, similar to that present in OpenCV Block Matching Stereo)
//    */
//  stereo.setRatioFilter(20);
//
//  /*
//   * setter for the value of the peak filter
//   * Parameters:
//   * [in] peak_filter value of the peak filter; it is a number in the range [0, inf] (0: no filtering action)
//   *
//   */
//  /**
//    * * postprocessing: filtering of wrong disparities via Peak Filter (eliminating ambiguities due to low-textured regions)
//    *
//    */
//  stereo.setPeakFilter(0);
//
//  /** \brief Stereo Matching abstract class
//    * * postprocessing: Left-Right consistency check (eliminates wrong disparities at the cost of twice the stereo matching
//    *   computation)
//    * * postprocessing: subpixel refinement of computed disparities, to reduce the depth quantization effect
//    */
//
//  stereo.setLeftRightCheck(true);
//  stereo.setLeftRightCheckThreshold(1);

  /**
    * * preprocessing of the image pair, to improve robustness against photometric distortions
    *   (wrt. to a spatially constant additive photometric factor)
    */
  stereo.setPreProcessing(true);

}

//----------------------------------------------------------------------
// tStereoVisionProcACSO destructor
//----------------------------------------------------------------------
StereoVisionProcACSO::~StereoVisionProcACSO()
{}

void
StereoVisionProcACSO::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
    case ' ':
      trigger = true;
      break;
    case 'c':
      continuous = !continuous;
      break;

      /*smoothness used for ACSO*/
    case '1':
      smooth_strong -= 10;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '2':
      smooth_strong += 10;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '3':
      smooth_weak -= 10;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    case '4':
      smooth_weak += 10;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    }
  }
}

void
StereoVisionProcACSO::processStereoPair(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud, pcl::PointCloud<pcl::RGB>::Ptr& texture)
{

  stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
  stereo.medianFilter(10);  // better but slower //optimal

  /*stereo images camera calibration parameters for ptgrey grayscale camera*/
  float u_c = 403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  float v_c = 358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
  float focal = 840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  float baseline = 0.46; //meter for unit //0.359294689f; //real one using calculator
  stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud, texture);

}// processPair

void
StereoVisionProcACSO::
processStereoPair(const pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud)
{

  stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
  stereo.medianFilter(10);  // better but slower //optimal

  /*stereo images camera calibration parameters for ptgrey grayscale camera*/
  float u_c = 403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  float v_c = 358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
  float focal = 840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  float baseline = 0.46; //meter for unit //0.359294689f; //real one using calculator
  stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud);

}// processPair

void
StereoVisionProcACSO::run()
{
  while (!viewer->wasStopped())
  {

    stereo_rectify(left_images[images_idx], right_images[images_idx]);
    waitKey(1);

    if (continuous)
    {
      images_idx++;
    }

    if (trigger)
    {
      images_idx++;
      trigger = false;
    }



    /*! displaying disparity map*/
//      CloudPtr out_cloud_disp(new Cloud);
//      pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
//      left_cloud = img2pcd(left_img_rect);
//      processStereoPair(out_cloud_disp, left_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_disp(new pcl::PointCloud<pcl::PointXYZ>);
    processStereoPair(out_cloud_disp);

    if (!viewer->updatePointCloud(out_cloud_disp, "cloud disparity"))
    {
      viewer->addPointCloud(out_cloud_disp, "cloud disparity");
    }//if

    pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
    stereo.getVisualMap(vmap);
    image_viewer->addRGBImage<RGB> (vmap);


    cout << "left_images[img_index]: " << left_images[images_idx] << std::endl;
    cout << "right_images[img_index]: " << right_images[images_idx] << std::endl;
    cout << "images_idx: " << images_idx << endl;
    cout << "img_pairs_num: " << img_pairs_num << endl;
    cout << "press q or Q on the main viewer to exit or space to continue................. " << endl;

    viewer->spinOnce(1);
    image_viewer->spinOnce(1);
  } // while

}// run


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
