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
/*!\file    projects/icarus/sensor_processing/libstereo/tStereoVisionProcBB.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 * \brief   Contains tStereoVisionProcBB
 *
 * \b tStereoVisionProcBB
 *
 * This is the class for initializing the BlockBased stereo matching technique.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo__tStereoVisionProcBB_h__
#define __projects__icarus__sensor_processing__libstereo__tStereoVisionProcBB_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libstereo/tStereoVisionProcessing.h"
#include "projects/icarus/sensor_processing/libstereo/tBlockBasedStereoMatching.h"

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
 * This is the class for initializing the BlockBased stereo matching technique.
 */
class StereoVisionProcBB : public StereoVisionProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  StereoVisionProcBB(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                     const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename);

  ~StereoVisionProcBB();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*);

  void
  processStereoPair(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud, pcl::PointCloud<pcl::RGB>::Ptr& texture);

  void
  processStereoPair(const pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud);

  void run();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  BlockBasedStereoMatching stereo;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
