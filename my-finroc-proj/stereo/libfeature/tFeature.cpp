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
/*!\file    projects/icarus/sensor_processing/libfeature/tFeature.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-29
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/search/pcl_search.h> //pcl::eigen33

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libfeature/tFeature.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix,
                     const Eigen::Vector4f &point,
                     Eigen::Vector4f &plane_parameters, float &curvature)
{
  solvePlaneParameters(covariance_matrix, plane_parameters [0], plane_parameters [1], plane_parameters [2], curvature);

  plane_parameters[3] = 0;
  /* Hessian form (D = nc . p_plane (centroid here) + p)*/ //originally commented out
  plane_parameters[3] = -1 * plane_parameters.dot(point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
inline void
solvePlaneParameters(const Eigen::Matrix3f &covariance_matrix,
                     float &nx, float &ny, float &nz, float &curvature)
{
  /*//originally commneted out
    // Avoid getting hung on Eigen's optimizers
    for (int i = 0; i < 9; ++i)
      if (!pcl_isfinite (covariance_matrix.coeff (i)))
      {
        //PCL_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!\n");
        nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
        return;
      }
  */

  // Extract the smallest eigenvalue and its eigenvector
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);

  nx = eigen_vector [0];
  ny = eigen_vector [1];
  nz = eigen_vector [2];

  // Compute the curvature surface change
  float eig_sum = covariance_matrix.coeff(0) + covariance_matrix.coeff(4) + covariance_matrix.coeff(8);
  if (eig_sum != 0)
    curvature = fabsf(eigen_value / eig_sum);
  else
    curvature = 0;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
