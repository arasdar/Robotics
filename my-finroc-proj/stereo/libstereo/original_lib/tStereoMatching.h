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
/*!\file    projects/icarus/sensor_processing/libstereo/tStereoMatching.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 * \brief   Contains tStereoMatching
 *
 * \b tStereoMatching
 *
 * This is stereo matching base class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo__tStereoMatching_h__
#define __projects__icarus__sensor_processing__libstereo__tStereoMatching_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/conversions.h>
#include <pcl/point_types.h>

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is stereo matching abstract class.
  *
  * The class performs stereo matching on a rectified stereo pair
  * Includes the following functionalities:
  * * preprocessing of the image pair, to improve robustness against photometric distortions
  *   (wrt. to a spatially constant additive photometric factor)
  * * postprocessing: filtering of wrong disparities via Peak Filter (eliminating ambiguities due to low-textured regions)
  *   and Ratio Filter (eliminating generic matching ambiguities, similar to that present in OpenCV Block Matching Stereo)
  * * postprocessing: Left-Right consistency check (eliminates wrong disparities at the cost of twice the stereo matching
  *   computation)
  * * postprocessing: subpixel refinement of computed disparities, to reduce the depth quantization effect
  * * postprocessing: smoothing of the disparity map via median filter
  * * after stereo matching a PCL point cloud can be computed, given the stereo intrinsic (focal, principal point
  *   coordinates) and extrinsic (baseline) calibration parameters
 */
class StereoMatching
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  StereoMatching();

  virtual ~StereoMatching();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

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

  /** \brief setter for the value of the ratio filter
    *
    * \param[in] ratio_filter value of the ratio filter; it is a number in the range [0, 100]
    * (0: no filtering action; 100: all disparities are filtered)
    */
  void
  setRatioFilter(int ratio_filter)
  {
    ratio_filter_ = ratio_filter;
  };

  /** \brief setter for the value of the peak filter
    *
    * \param[in] peak_filter value of the peak filter; it is a number in the range [0, inf]
    * (0: no filtering action)
    */
  void
  setPeakFilter(int peak_filter)
  {
    peak_filter_ = peak_filter;
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

  /** \brief setter for the left-right consistency check stage, that eliminates inconsistent/wrong disparity
    * values from the disparity map at approx. twice the processing cost of the selected stereo algorithm
    *
    * \param[in] is_lr_check setting the boolean to true activates the left-right consistency check
    */
  void
  setLeftRightCheck(bool is_lr_check)
  {
    is_lr_check_ = is_lr_check;
  };

  /** \brief setter for the left-right consistency check threshold
    *
    * \param[in] lr_check_th sets the value of the left-right consistency check threshold
    * only has some influence if the left-right check is active
    * typically has either the value 0 ("strong" consistency check, more points being filtered) or 1 ("weak"
    * consistency check, less points being filtered)
    */
  void
  setLeftRightCheckThreshold(int lr_check_th)
  {
    lr_check_th_ = lr_check_th;
  };

  /** \brief stereo processing, it computes a disparity map stored internally by the class
    *
    * \param[in] ref_img reference array of image pixels (left image)
    * \param[in] trg_img target array of image pixels (right image)
    * \param[in] width number of elements per row for both input arrays
    * \param[in] height number of elements per column for both input arrays
    */
  virtual void
  compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height) = 0;

  /** \brief stereo processing, it computes a disparity map stored internally by the class
    *
    * \param[in] ref point cloud of pcl::RGB type containing the pixels of the reference image (left image)
    * \param[in] trg point cloud of pcl::RGB type containing the pixels of the target image (right image)
    */
  virtual void
  compute(pcl::PointCloud<pcl::RGB> &ref, pcl::PointCloud<pcl::RGB> &trg) = 0;

  /** \brief median filter applied on the previously computed disparity map
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[in] radius radius of the squared window used to compute the median filter; the window side is
    * equal to 2*radius + 1
    */
  void
  medianFilter(int radius);

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
  virtual bool
  getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /** \brief computation of the 3D point cloud from the previously computed disparity map including color information
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[in] u_c horizontal coordinate of the principal point (calibration parameter)
    * \param[in] v_c vertical coordinate of the principal point (calibration parameter)
    * \param[in] focal focal length in pixels (calibration parameter)
    * \param[in] baseline distance between the two cameras (calibration parameter); the measure unit used to
    * specify this parameter will be the same as the 3D points in the output point cloud
    * \param[out] cloud output 3D point cloud; it is organized and non-dense, with NaNs where 3D points are invalid
    * \param[in] input 3D cloud (same size of the output cloud) used to associate to each 3D point of the
    * output cloud a color triplet
    */
  virtual bool
  getPointCloud(float u_c, float v_c, float focal, float baseline, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,  pcl::PointCloud<pcl::RGB>::Ptr texture);

  /** \brief computation of a pcl::RGB cloud with scaled disparity values
    * it can be used to display a rescaled version of the disparity map by means of the pcl::ImageViewer
    * invalid disparity values are shown in green
    * Note: the "compute" method must have been previously called at least once in order for this function
    * to have any effect
    * \param[out] vMap output cloud
    */
  void
  getVisualMap(pcl::PointCloud<pcl::RGB>::Ptr vMap);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*EMPTY*/

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

  /** \brief Disparity map used for left-right check. */
  short int *disp_map_trg_;

  /** \brief Local aligned copies used for pre processing. */
  unsigned char* pp_ref_img_;
  unsigned char* pp_trg_img_;

  /** \brief number of pixels per row of the input stereo pair . */
  int width_;

  /** \brief number of pixels per column of the input stereo pair . */
  int height_;

  /** \brief Disparity range used for stereo processing. */
  int max_disp_;

  /** \brief Horizontal displacemente (x offset) used for stereo processing */
  int x_off_;

  /** \brief Threshold for the ratio filter, \f$\in [0 100]\f$ */
  int ratio_filter_;

  /** \brief Threshold for the peak filter, \f$\in [0 \infty]\f$ */
  int peak_filter_;

  /** \brief toggle for the activation of the pre-processing stage */
  bool is_pre_proc_;

  /** \brief toggle for the activation of the left-right consistency check stage */
  bool is_lr_check_;

  /** \brief Threshold for the left-right consistency check, typically either 0 or 1 */
  int lr_check_th_;

  virtual void
  preProcessing(unsigned char *img, unsigned char *pp_img) = 0;

  virtual void
  imgFlip(unsigned char * & img) = 0;

  virtual void
  compute_impl(unsigned char* ref_img, unsigned char* trg_img) = 0;

  void
  leftRightCheck();

  inline short int
  computeStereoSubpixel(int dbest, int s1, int s2, int s3)
  {
    int den = (s1 + s3 - 2 * s2);
    if (den != 0)
      return (static_cast<short int>(16 * dbest + (((s1 - s3) * 8) / den)));
    else
      return (static_cast<short int>(dbest * 16));
  }

  inline short int
  computeStereoSubpixel(int dbest, float s1, float s2, float s3)
  {
    float den = (s1 + s3 - 2 * s2);
    if (den != 0)
      return (static_cast<short int>(16 * dbest + floor(.5 + (((s1 - s3) * 8) / den))));
    else
      return (static_cast<short int>(dbest * 16));
  }

  inline short int
  doStereoRatioFilter(int *acc, short int dbest, int sad_min, int ratio_filter, int maxdisp, int precision = 100)
  {
    int sad_second_min = std::numeric_limits<int>::max();

    for (int d = 0; d < dbest - 1; d++)
      if (acc[d] < sad_second_min)
        sad_second_min = acc[d];

    for (int d = dbest + 2; d < maxdisp; d++)
      if (acc[d] < sad_second_min)
        sad_second_min = acc[d];

    if (sad_min * precision > (precision - ratio_filter) * sad_second_min)
      return (-2);
    else
      return (dbest);
  }

  inline short int
  doStereoRatioFilter(float *acc, short int dbest, float sad_min, int ratio_filter, int maxdisp, int precision = 100)
  {
    float sad_second_min = std::numeric_limits<float>::max();

    for (int d = 0; d < dbest - 1; d++)
      if (acc[d] < sad_second_min)
        sad_second_min = acc[d];

    for (int d = dbest + 2; d < maxdisp; d++)
      if (acc[d] < sad_second_min)
        sad_second_min = acc[d];

    if (sad_min * static_cast<float>(precision) > static_cast<float>(precision - ratio_filter) * sad_second_min)
      return (-2);
    else
      return (dbest);
  }

  inline short int
  doStereoPeakFilter(int *acc, short int dbest, int peak_filter, int maxdisp)
  {
    int da = (dbest > 1) ? (acc[dbest - 2] - acc[dbest]) : (acc[dbest + 2] - acc[dbest]);
    int db = (dbest < maxdisp - 2) ? (acc[dbest + 2] - acc[dbest]) : (acc[dbest - 2] - acc[dbest]);

    if (da + db < peak_filter)
      return (-4);
    else
      return (dbest);
  }

  inline short int
  doStereoPeakFilter(float *acc, short int dbest, int peak_filter, int maxdisp)
  {
    float da = (dbest > 1) ? (acc[dbest - 2] - acc[dbest]) : (acc[dbest + 2] - acc[dbest]);
    float db = (dbest < maxdisp - 2) ? (acc[dbest + 2] - acc[dbest]) : (acc[dbest - 2] - acc[dbest]);

    if (da + db < peak_filter)
      return (-4);
    else
      return (dbest);
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
