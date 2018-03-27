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
#ifndef __projects__stereo_traversability_experiments__aras__libsegmentation__tGroundPlaneComparator_h__
#define __projects__stereo_traversability_experiments__aras__libsegmentation__tGroundPlaneComparator_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/common/angles.h>
#include <boost/make_shared.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libsegmentation/tComparator.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace aras
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

  //using pcl::Comparator<PointT>::input_;
  using Comparator<PointT>::input_;

  /** \brief Empty constructor for GroundPlaneComparator. */
  GroundPlaneComparator()
    : normals_()
    , plane_coeff_d_()
    , angular_threshold_(cosf(pcl::deg2rad(2.0f)))
    , road_angular_threshold_(cosf(pcl::deg2rad(10.0f)))
    , distance_threshold_(0.1f)
    , depth_dependent_(true)
    , z_axis_(Eigen::Vector3f(0.0, 0.0, 1.0))
    , desired_road_axis_(Eigen::Vector3f(0.0, -1.0, 0.0))
  {
  }

  /** \brief Constructor for GroundPlaneComparator.
    * \param[in] plane_coeff_d a reference to a vector of d coefficients of plane equations.  Must be the same size as the input cloud and input normals.  a, b, and c coefficients are in the input normals.
    */
  GroundPlaneComparator(boost::shared_ptr<std::vector<float>>& plane_coeff_d)
    : normals_()
    , plane_coeff_d_(plane_coeff_d)
    , angular_threshold_(cosf(pcl::deg2rad(3.0f)))
    , distance_threshold_(0.1f)
    , depth_dependent_(true)
    , z_axis_(Eigen::Vector3f(0.0f, 0.0f, 1.0f))
    , road_angular_threshold_(cosf(pcl::deg2rad(40.0f)))
    , desired_road_axis_(Eigen::Vector3f(0.0, -1.0, 0.0))
  {
  }

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

  /** \brief Get the input normals. */
  inline PointCloudNConstPtr
  getInputNormals() const
  {
    return (normals_);
  }

  /** \brief Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.  a, b, and c are provided by the normal cloud.
    * \param[in] plane_coeff_d a pointer to the plane coefficients.
    */
  void
  setPlaneCoeffD(boost::shared_ptr<std::vector<float>>& plane_coeff_d)
  {
    plane_coeff_d_ = plane_coeff_d;
  }

  /** \brief Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.  a, b, and c are provided by the normal cloud.
    * \param[in] plane_coeff_d a pointer to the plane coefficients.
    */
  void
  setPlaneCoeffD(std::vector<float>& plane_coeff_d)
  {
    plane_coeff_d_ = boost::make_shared<std::vector<float>>(plane_coeff_d);
  }

  /** \brief Get a pointer to the vector of the d-coefficient of the planes' hessian normal form. */
  const std::vector<float>&
  getPlaneCoeffD() const
  {
    return (plane_coeff_d_);
  }

  /** \brief Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
    * \param[in] angular_threshold the tolerance in radians
    */
  virtual void
  setAngularThreshold(float angular_threshold)
  {
    angular_threshold_ = cosf(angular_threshold);
  }

  /** \brief Set the tolerance in radians for difference in normal direction between a point and the expected ground normal.
    * \param[in] angular_threshold the
    */
  virtual void
  setGroundAngularThreshold(float angular_threshold)
  {
    road_angular_threshold_ = cosf(angular_threshold);
  }

  /** \brief Set the expected ground plane normal with respect to the sensor.  Pixels labeled as ground must be within ground_angular_threshold radians of this normal to be labeled as ground.
    * \param[in] normal The normal direction of the expected ground plane.
    */
  void
  setExpectedGroundNormal(Eigen::Vector3f normal)
  {
    desired_road_axis_ = normal;
  }


  /** \brief Get the angular threshold in radians for difference in normal direction between neighboring points, to be considered part of the same plane. */
  inline float
  getAngularThreshold() const
  {
    return (acosf(angular_threshold_));
  }

  /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
    * \param[in] distance_threshold the tolerance in meters (at 1m)
    * \param[in] depth_dependent whether to scale the threshold based on range from the sensor (default: false)
    */
  void
  setDistanceThreshold(float distance_threshold,
                       bool depth_dependent = false)
  {
    distance_threshold_ = distance_threshold;
    depth_dependent_ = depth_dependent;
  }

  /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
  inline float
  getDistanceThreshold() const
  {
    return distance_threshold_;
  }

  /** \brief Compare points at two indices by their plane equations.  True if the angle between the normals is less than the angular threshold,
    * and the difference between the d component of the normals is less than distance threshold, else false
    * \param idx1 The first index for the comparison
    * \param idx2 The second index for the comparison
    */
  virtual bool
  compare_orig(int idx1, int idx2) const
  {
    // Normal must be similar to neighbor
    // Normal must be similar to expected normal
    /*
        float threshold = distance_threshold_;
        if (depth_dependent_)
        {
          Eigen::Vector3f vec = input_->points[idx1].getVector3fMap();

          float z = vec.dot(z_axis_);
          threshold *= z * z;
        }
    */

    return ((normals_->points[idx1].getNormalVector3fMap().dot(desired_road_axis_) > road_angular_threshold_) &&
            (normals_->points[idx1].getNormalVector3fMap().dot(normals_->points[idx2].getNormalVector3fMap()) > angular_threshold_));

    /*
        // Euclidean proximity of neighbors does not seem to be required -- pixel adjacency handles this well enough
        //return ( (normals_->points[idx1].getNormalVector3fMap ().dot (desired_road_axis_) > road_angular_threshold_) &&
        //          (normals_->points[idx1].getNormalVector3fMap ().dot (normals_->points[idx2].getNormalVector3fMap () ) > angular_threshold_ ) &&
        //         (pcl::euclideanDistance (input_->points[idx1], input_->points[idx2]) < distance_threshold_ ));
    */
  }

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
  boost::shared_ptr<std::vector<float>> plane_coeff_d_;
  float angular_threshold_;
  float road_angular_threshold_;
  float distance_threshold_;
  bool depth_dependent_;
  Eigen::Vector3f z_axis_;
//  Eigen::VectorXf z_axis_;
  Eigen::Vector3f desired_road_axis_;
//  Eigen::VectorXf desired_road_axis_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


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
