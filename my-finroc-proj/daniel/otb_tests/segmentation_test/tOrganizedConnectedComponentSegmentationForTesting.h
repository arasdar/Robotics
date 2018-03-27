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
/*!\file    projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentationForTesting.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-29
 *
 * \brief   Contains tOrganizedConnectedComponentSegmentationForTesting
 *
 * \b tOrganizedConnectedComponentSegmentationForTesting
 *
 * This class OrganizedConnectedComponentSegmentationForTesting allows connected components to be found within organized point cloud data.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__daniel__otb_tests__tOrganizedConnectedComponentSegmentationForTesting_h__
#define __projects__stereo_traversability_experiments__daniel__otb_tests__tOrganizedConnectedComponentSegmentationForTesting_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/daniel/libsegmentation/tComparator.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace daniel
{
namespace otb_tests
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This class OrganizedConnectedComponentSegmentationForTesting allows connected
 * components to be found within organized point cloud data given a comparison function.
  * Given an input cloud and a comparator, it will
  * output a PointCloud of labels, giving each connected component a unique
  * id, along with a vector of PointIndices corresponding to each component.
  *
 */
template <typename PointT, typename PointLT>
class OrganizedConnectedComponentSegmentationForTesting : public pcl::PCLBase<PointT>
{

  using pcl::PCLBase<PointT>::input_;
  using pcl::PCLBase<PointT>::indices_;
  using pcl::PCLBase<PointT>::initCompute;
  using pcl::PCLBase<PointT>::deinitCompute;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef typename pcl::PointCloud<PointLT> PointCloudL;
  typedef typename PointCloudL::Ptr PointCloudLPtr;
  typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

  typedef typename Comparator<PointT>::Ptr ComparatorPtr;
  typedef typename Comparator<PointT>::ConstPtr ComparatorConstPtr;

  /** \brief Constructor for OrganizedConnectedComponentSegmentationForTesting
    * \param[in] compare A pointer to the comparator to be used for segmentation.  Must be an instance of pcl::Comparator.
    */
  OrganizedConnectedComponentSegmentationForTesting(const ComparatorConstPtr& compare)
    : compare_(compare)
  {}

  /** \brief Destructor for OrganizedConnectedComponentSegmentationForTesting. */
  virtual ~OrganizedConnectedComponentSegmentationForTesting() {}  //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /** \brief Perform the connected component segmentation.
    * \param[out] labels a PointCloud of labels: each connected component will have a unique id.
    * \param[out] label_indices a vector of PointIndices corresponding to each label / component id.
    */
  void
  segment(pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const;

//----------------------------------------------------------------------
// Protected methods and typedefs
//----------------------------------------------------------------------
protected:
  ComparatorConstPtr compare_;

  inline unsigned
  findRoot(const std::vector<unsigned>& runs, unsigned index) const
  {
    register unsigned idx = index;
    while (runs[idx] != idx)
      idx = runs[idx];

    return (idx);
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------

}
}
}
}

#include "projects/stereo_traversability_experiments/daniel/otb_tests/segmentation_test/tOrganizedConnectedComponentSegmentationForTesting.cpp"

#endif
