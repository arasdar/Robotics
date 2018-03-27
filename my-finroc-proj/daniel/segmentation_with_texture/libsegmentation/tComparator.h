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
/*!\file    projects/icarus/sensor_processing/libsegmentation/tComparator.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-29
 *
 * \brief   Contains tComparator
 *
 * \b tComparator
 *
 * This is a base comparator class that compare two points using a given function.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__daniel__segmentation_with_texture__libsegmentation__tComparator_h__
#define __projects__stereo_traversability_experiments__daniel__segmentation_with_texture__libsegmentation__tComparator_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/point_cloud.h>

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
namespace daniel
{
namespace segmentation_with_texture
{
namespace libsegmentation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is a base comparator class that compare two points using a given function.
 * Comparator is the base class for comparators that compare two points given some function.
 */
template <typename PointT>
class Comparator
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef boost::shared_ptr<Comparator<PointT>> Ptr;
  typedef boost::shared_ptr<const Comparator<PointT>> ConstPtr;

  /** \brief Empty constructor for comparator. */
  Comparator() : input_() {}

  /** \brief Empty destructor for comparator. */
  virtual ~Comparator() {}  //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /** \brief Set the input cloud for the comparator.
    * \param[in] cloud the point cloud this comparator will operate on
    */
  virtual void
  setInputCloud(const PointCloudConstPtr& cloud)
  {
    input_ = cloud;
  }

  /** \brief Compares the two points in the input cloud designated by these two indices.
    * This is pure virtual and must be implemented by subclasses with some comparison function.
    * \param[in] idx1 the index of the first point.
    * \param[in] idx2 the index of the second point.
    */
  virtual bool
  compare(int idx1, int idx2) const = 0;

//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:
  PointCloudConstPtr input_;

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
