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
/*!\file    projects/icarus/sensor_processing/libstereo_test/tTestStereoMatching.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 * \brief   Contains tTestStereoMatching
 *
 * \b tTestStereoMatching
 *
 *
 *
 *
 * This is the test class for stereo matching using OpenMP and GPU.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__libstereo__tTestStereoMatching_h__
#define __projects__stereo_traversability_experiments__aras__libstereo__tTestStereoMatching_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>

#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <iostream>

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
namespace aras
{
namespace libstereo
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the test class for stereo matching using OpenMP and GPU.
 */
class tTestStereoMatching
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tTestStereoMatching();

  virtual ~tTestStereoMatching();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  /** \brief setter for number of disparity candidates (disparity range)
    *
    * \param[in] max_disp number of disparity candidates (disparity range); has to be > 0
    */
  void
  setMaxDisparity(int max_disp)
  {
    max_disp_ = max_disp;
  };

  /** \brief setter for horizontal offset, i.e. number of pixels to shift the disparity range over the target image
    *
    * \param[in] x_off horizontal offset value; has to be >= 0
    */
  void
  setXOffset(int x_off)
  {
    x_off_ = x_off;
  };

  /** \brief setter for the pre processing step
    *
    * \param[in] is_pre_proc setting the boolean to true activates the pre-processing step for both stereo images
    */
  void
  setPreProcessing(bool is_pre_proc)
  {
    is_pre_proc_ = is_pre_proc;
  };

  /** \brief median filter applied on the previously computed disparity map
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[in] radius radius of the squared window used to compute the median filter; the window side is
    * equal to 2*radius + 1
    */
  void
  medianFilter(int radius);

  /** \brief computation of a pcl::RGB cloud with scaled disparity values
    * it can be used to display a rescaled version of the disparity map by means of the pcl::ImageViewer
    * invalid disparity values are shown in green
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[out] vMap output cloud
    */
  void
  getVisualMap(pcl::PointCloud<pcl::RGB>::Ptr vMap);

  /** \brief computation of a cv::Mat cloud with scaled disparity values
    * it can be used to display a rescaled version of the disparity map by means of the cv::imshow
    * invalid disparity values are shown in green
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[out] vMap output img
    */
  void
  getVisualMap(cv::Mat vMap);

  /** \brief computation of the 3D point cloud from the previously computed disparity map without color information
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[in] u_c horizontal coordinate of the principal point (calibration parameter)
    * \param[in] v_c vertical coordinate of the principal point (calibration parameter)
    * \param[in] focal focal length in pixels (calibration parameter)
    * \param[in] baseline distance between the two cameras (calibration parameter); the measure unit used to
    * specify this parameter will be the same as the 3D points in the output point cloud
    * \param[out] cloud output 3D point cloud; it is organized and non-dense, with NaNs where 3D points are invalid
    */
  bool
  getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /** extra \param[in] input 3D cloud (same size of the output cloud) used to associate to each 3D point of the
    * output cloud a color triplet
    */
  bool
  getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  bool
  getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                pcl::PointCloud<pcl::RGB>::Ptr texture);

  bool getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     cv::Mat texture);

  /** \brief stereo processing, it computes a disparity map stored internally by the class
    *
    * \param[in] ref_img reference array of image pixels (left image), has to be grayscale single channel
    * \param[in] trg_img target array of image pixels (right image), has to be grayscale single channel
    * \param[in] width number of elements per row for both input arrays
    * \param[in] height number of elements per column for both input arrays
    */
  virtual void
  compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height);

//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:

  /*Here is the right place for your variables. Replace this line by your declarations!*/

  /** \brief The internal disparity map. */
  short int *disp_map_;

  /** \brief Local aligned copies of the cloud data. */
  unsigned char* ref_img_;
  unsigned char* trg_img_;

  /** \brief Local aligned copies used for pre processing. */
  unsigned char* pp_ref_img_;
  unsigned char* pp_trg_img_;

  /** \brief number of pixels per row of the input stereo pair . */
  unsigned int width_;

  /** \brief number of pixels per column of the input stereo pair . */
  unsigned int height_;

  /** \brief Disparity range used for stereo processing. */
  unsigned int max_disp_;

  /** \brief Horizontal displacement (x offset) used for stereo processing */
  unsigned int x_off_;

  /** \brief toggle for the activation of the pre-processing stage */
  bool is_pre_proc_;

  virtual void
  compute_impl(unsigned char* ref_img, unsigned char* trg_img) = 0;

  virtual void
  preProcessing(unsigned char *img, unsigned char *pp_img);

  virtual void preProcessing_openmp(unsigned char* img, unsigned char* pp_img);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
