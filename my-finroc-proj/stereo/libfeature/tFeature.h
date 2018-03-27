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
/*!\file    projects/icarus/sensor_processing/libfeature/tFeature.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-29
 *
 * \brief   Contains tFeature
 *
 * \b tFeature
 *
 * Feature represents the base feature class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__aras__libfeature__tFeature_h__
#define __projects__stereo_traversability_experiments__aras__libfeature__tFeature_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
//#if defined __GNUC__
//#  pragma GCC system_header
//#endif

#include <boost/function.hpp>
#include <boost/bind.hpp>
// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>

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
namespace libfeature
{

using namespace pcl;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * Feature represents the base feature class.
 * Some generic 3D operations that are applicable to all features are defined here as static methods.
 *
 *  *
  * \attention
  * The convention for a feature descriptor is:
  *   - if the nearest neighbors for the query point at which the descriptor is to be computed cannot be
  *     determined, the descriptor values will be set to NaN (not a number)
  *   - it is impossible to estimate a feature descriptor for a point that doesn't have finite 3D coordinates.
  *     Therefore, any point that has NaN data on x, y, or z, will most likely have its descriptor set to NaN.
  *
 *
 */
template <typename PointInT, typename PointOutT>
class tFeature : public PCLBase<PointInT>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /** \brief Empty constructor. */
  tFeature() :
    feature_name_(), search_method_surface_(),
    surface_(), tree_(),
    search_parameter_(0), search_radius_(0), k_(0),
    fake_surface_(false)
  {}

  /** \brief Empty destructor */
  virtual ~tFeature() {}   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  using PCLBase<PointInT>::indices_;
  using PCLBase<PointInT>::input_;

  typedef PCLBase<PointInT> BaseClass;

  typedef boost::shared_ptr<tFeature<PointInT, PointOutT>> Ptr;
  typedef boost::shared_ptr<const tFeature<PointInT, PointOutT>> ConstPtr;

  typedef typename pcl::search::Search<PointInT> KdTree;
  typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;

  typedef pcl::PointCloud<PointInT> PointCloudIn;
  typedef typename PointCloudIn::Ptr PointCloudInPtr;
  typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

  typedef pcl::PointCloud<PointOutT> PointCloudOut;

  typedef boost::function<int (size_t, double, std::vector<int> &, std::vector<float> &)> SearchMethod;
  typedef boost::function<int (const PointCloudIn &cloud, size_t index, double, std::vector<int> &, std::vector<float> &)> SearchMethodSurface;

  /** \brief Provide a pointer to a dataset to add additional information
    * to estimate the features for every point in the input dataset.  This
    * is optional, if this is not set, it will only use the data in the
    * input cloud to estimate the features.  This is useful when you only
    * need to compute the features for a downsampled cloud.
    * \param[in] cloud a pointer to a PointCloud message
    */
  inline void
  setSearchSurface(const PointCloudInConstPtr &cloud)
  {
    surface_ = cloud;
    fake_surface_ = false;
    //use_surface_  = true;
  }

  /** \brief Get a pointer to the surface point cloud dataset. */
  inline PointCloudInConstPtr
  getSearchSurface() const
  {
    return (surface_);
  }

  /** \brief Provide a pointer to the search object.
    * \param[in] tree a pointer to the spatial search object.
    */
  inline void
  setSearchMethod(const KdTreePtr &tree)
  {
    tree_ = tree;
  }

  /** \brief Get a pointer to the search method used. */
  inline KdTreePtr
  getSearchMethod() const
  {
    return (tree_);
  }

  /** \brief Get the internal search parameter. */
  inline double
  getSearchParameter() const
  {
    return (search_parameter_);
  }

  /** \brief Set the number of k nearest neighbors to use for the feature estimation.
    * \param[in] k the number of k-nearest neighbors
    */
  inline void
  setKSearch(int k)
  {
    k_ = k;
  }

  /** \brief get the number of k nearest neighbors used for the feature estimation. */
  inline int
  getKSearch() const
  {
    return (k_);
  }

  /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
    * estimation.
    * \param[in] radius the sphere radius used as the maximum distance to consider a point a neighbor
    */
  inline void
  setRadiusSearch(double radius)
  {
    search_radius_ = radius;
  }

  /** \brief Get the sphere radius used for determining the neighbors. */
  inline double
  getRadiusSearch() const
  {
    return (search_radius_);
  }

  /** \brief Base method for feature estimation for all points given in
    * <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
    * and the spatial locator in setSearchMethod ()
    * \param[out] output the resultant point cloud model dataset containing the estimated features
    */
  void
  compute(PointCloudOut &output);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  /** \brief Abstract feature estimation method.
    * \param[out] output the resultant features
    */
  virtual void
  computeFeature(PointCloudOut &output) = 0;

//----------------------------------------------------------------------
// Protected fields and methods
//----------------------------------------------------------------------
protected:
  /** \brief The feature name. */
  std::string feature_name_;

  /** \brief The search method template for points. */
  SearchMethodSurface search_method_surface_;

  /** \brief An input point cloud describing the surface that is to be used
    * for nearest neighbors estimation.
    */
  PointCloudInConstPtr surface_;

  /** \brief A pointer to the spatial search object. */
  KdTreePtr tree_;

  /** \brief The actual search parameter (from either \a search_radius_ or \a k_). */
  double search_parameter_;

  /** \brief The nearest neighbors search radius for each point. */
  double search_radius_;

  /** \brief The number of K nearest neighbors to use for each point. */
  int k_;

  /** \brief Get a string representation of the name of this class. */
  inline const std::string&
  getClassName() const
  {
    return (feature_name_);
  }

  /** \brief This method should get called before starting the actual computation. */
  virtual bool
  initCompute();

  /** \brief This method should get called after ending the actual computation. */
  virtual bool
  deinitCompute();

  /** \brief If no surface is given, we use the input PointCloud as the surface. */
  bool fake_surface_;

  /** \brief Search for k-nearest neighbors using the spatial locator from
    * \a setSearchmethod, and the given surface from \a setSearchSurface.
    * \param[in] index the index of the query point
    * \param[in] parameter the search parameter (either k or radius)
    * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
    * \param[out] distances the resultant vector of distances representing the distances from the query point to the
    * k-nearest neighbors
    *
    * \return the number of neighbors found. If no neighbors are found or an error occurred, return 0.
    */
  inline int
  searchForNeighbors(size_t index, double parameter,
                     std::vector<int> &indices, std::vector<float> &distances) const
  {
    return (search_method_surface_(*input_, index, parameter, indices, distances));
  }

  /** \brief Search for k-nearest neighbors using the spatial locator from
    * \a setSearchmethod, and the given surface from \a setSearchSurface.
    * \param[in] cloud the query point cloud
    * \param[in] index the index of the query point in \a cloud
    * \param[in] parameter the search parameter (either k or radius)
    * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
    * \param[out] distances the resultant vector of distances representing the distances from the query point to the
    * k-nearest neighbors
    *
    * \return the number of neighbors found. If no neighbors are found or an error occurred, return 0.
    */
  inline int
  searchForNeighbors(const PointCloudIn &cloud, size_t index, double parameter,
                     std::vector<int> &indices, std::vector<float> &distances) const
  {
    return (search_method_surface_(cloud, index, parameter, indices, distances));
  }
};



/** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
  * plane normal and surface curvature.
  * \param covariance_matrix the 3x3 covariance matrix
  * \param point a point lying on the least-squares plane (SSE aligned)
  * \param plane_parameters the resultant plane parameters as: a, b, c, d (ax + by + cz + d = 0)
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
inline void
solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix,
                     const Eigen::Vector4f &point,
                     Eigen::Vector4f &plane_parameters, float &curvature);

/** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
  * plane normal and surface curvature.
  * \param covariance_matrix the 3x3 covariance matrix
  * \param nx the resultant X component of the plane normal
  * \param ny the resultant Y component of the plane normal
  * \param nz the resultant Z component of the plane normal
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
inline void
solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix,
                     float &nx, float &ny, float &nz, float &curvature);
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}

#include "projects/stereo_traversability_experiments/aras/libfeature/tFeature.hpp"

#endif
