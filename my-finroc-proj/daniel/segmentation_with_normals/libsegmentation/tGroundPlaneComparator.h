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
/*!\file    projects/icarus/sensor_processing/libsegmentation/tGroundPlaneComparator.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-29
 *
 * \brief   Contains tGroundPlaneComparator
 *
 * \b tGroundPlaneComparator
 *
 * This is comparator class which is used to compare ground plane with other points.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__daniel__segmentation_with_normals__libsegmentation__tGroundPlaneComparator_h__
#define __projects__stereo_traversability_experiments__daniel__segmentation_with_normals__libsegmentation__tGroundPlaneComparator_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/common/angles.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/daniel/segmentation_with_normals/libsegmentation/tComparator.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace daniel
{
namespace segmentation_with_normals
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
 * This is comparator class which is used to compare ground plane with other points.
 *
 * GroundPlaneComparator is a Comparator for detecting traversable (smooth enough) surfaces in a terrain which suitable for navigation (driving).
 * This comparator allows traversable (smooth) ground planes or surfaces to be segmented from point clouds.
 */
template<typename PointT, typename PointNT>
class GroundPlaneComparator : public Comparator<PointT>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename Comparator<PointT>::PointCloud PointCloud;
  typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;

  typedef typename pcl::PointCloud<PointNT> PointCloudN;
  typedef typename PointCloudN::Ptr PointCloudNPtr;
  typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

  typedef boost::shared_ptr<GroundPlaneComparator<PointT, PointNT>> Ptr;
  typedef boost::shared_ptr<const GroundPlaneComparator<PointT, PointNT>> ConstPtr;

  using Comparator<PointT>::input_;

  /** \brief Empty constructor for GroundPlaneComparator. */
  GroundPlaneComparator()
    : normals_()
    , angular_threshold_(cosf(pcl::deg2rad(1.0f))) //default angular threshold
  {}

  /** \brief Destructor for GroundPlaneComparator. */
  virtual
  ~GroundPlaneComparator() {}  //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  /** \brief Provide the input cloud.
    * \param[in] cloud the input point cloud.
    */
  virtual void
  setInputCloud(const PointCloudConstPtr& cloud)
  {
    input_ = cloud;
  }

  /** \brief Provide a pointer to the input normals.
    * \param[in] normals the input normal cloud.
    */
  inline void
  setInputNormals(const PointCloudNConstPtr &normals)
  {
    normals_ = normals;
  }

  /** \brief Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
    * \param[in] angular_threshold the tolerance in radians
    */
  virtual void
  setAngularThreshold(float angular_threshold)
  {
    angular_threshold_ = cosf(angular_threshold);
  }

  /** \brief Compare points at two indices by their plane equations.  True if the angle between the normals is less than the angular threshold,
    * and the difference between the d component of the normals is less than distance threshold, else false
    * \param idx1 The first index for the comparison
    * \param idx2 The second index for the comparison
    */
  virtual bool
  compare(int idx1, int idx2) const
  {
    return (normals_->points[idx1].getNormalVector3fMap().dot(normals_->points[idx2].getNormalVector3fMap()) > angular_threshold_);
  }

//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:
  PointCloudNConstPtr normals_;
  float angular_threshold_;

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
