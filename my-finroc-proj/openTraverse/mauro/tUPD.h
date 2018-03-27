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
/*!\file    projects/stereo_traversability_experiments/openTraverse/mauro/tUPD.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-10-20
 *
 * \brief   Contains tUPD
 *
 * \b tUPD
 *
 * Thsi si the unevenness point descriptor implemtation for terrain traversability analysis.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__openTraverse__mauro__tUPD_h__
#define __projects__stereo_traversability_experiments__openTraverse__mauro__tUPD_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

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
namespace openTraverse
{
namespace mauro
{

using namespace pcl;
using namespace std;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Thsi si the unevenness point descriptor implemtation for terrain traversability analysis.
 */
class tUPD
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tUPD();

  ~tUPD();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

//  Here is the right place for your public methods. Replace this line by your declarations!
  uint32_t GiveRainbowColor(float position);
  int  upd();
  void upd_filter();
  void upd_normals_extraction();
  void upd_trav();
  void upd_class();

  CloudPtr image_cloud;    //define input cloud
  CloudPtr colored_cloud;
  PointCloud<Normal>::Ptr r_cloud;
  PointCloud<Normal>::Ptr normals;
  float normalRadiusSearch = 0.4;   // is the radius of the round in meter 1.0 for outside acquisition, 0.3 is good for the kinect sensor acquisition

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}


#endif
