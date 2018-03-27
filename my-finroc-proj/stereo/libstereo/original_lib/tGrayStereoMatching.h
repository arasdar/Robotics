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
/*!\file    projects/icarus/sensor_processing/libstereo/tGrayStereoMatching.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-27
 *
 * \brief   Contains tGrayStereoMatching
 *
 * \b tGrayStereoMatching
 *
 * This is Stereo Matching abstract class for Grayscale images.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo__tGrayStereoMatching_h__
#define __projects__icarus__sensor_processing__libstereo__tGrayStereoMatching_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tStereoMatching.h"
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
 * This class implements some functionalities of the abstract class tStereoMatching specific for grayscale stereo processing such as
 * such as image pre-processing, image flipping
 */
class GrayStereoMatching: public StereoMatching
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  GrayStereoMatching();

  virtual ~GrayStereoMatching();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  /** \brief stereo processing, it computes a disparity map stored internally by the class
    *
    * \param[in] ref_img reference array of image pixels (left image), has to be grayscale single channel
    * \param[in] trg_img target array of image pixels (right image), has to be grayscale single channel
    * \param[in] width number of elements per row for both input arrays
    * \param[in] height number of elements per column for both input arrays
    */
  virtual void
  compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height);

  /** \brief stereo processing, it computes a disparity map stored internally by the class
    *
    * \param[in] ref point cloud of pcl::RGB type containing the pixels of the reference image (left image)
    * the pcl::RGB triplets are automatically converted to grayscale upon call of the method
    * \param[in] trg point cloud of pcl::RGB type containing the pixels of the target image (right image)
    * the pcl::RGB triplets are automatically converted to grayscale upon call of the method
    */
  virtual void
  compute(pcl::PointCloud<pcl::RGB> &ref, pcl::PointCloud<pcl::RGB> &trg);

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
  virtual void
  compute_impl(unsigned char* ref_img, unsigned char* trg_img) = 0;

  virtual void
  preProcessing(unsigned char *img, unsigned char *pp_img);

  virtual void
  imgFlip(unsigned char * & img);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
