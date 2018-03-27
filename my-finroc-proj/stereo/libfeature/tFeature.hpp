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
/*!\file    projects/icarus/sensor_processing/libfeature/tFeature.hpp
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
#include <pcl/search/pcl_search.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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


////----------------------------------------------------------------------
//// tFeature constructors
////----------------------------------------------------------------------
//template <>
//tFeature<>::tFeature() //:
//  /*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
//{}
//
////----------------------------------------------------------------------
//// tFeature destructor
////----------------------------------------------------------------------
//template <>
//tFeature<>::~tFeature()
//{}
//
////----------------------------------------------------------------------
//// tFeature SomeExampleMethod
////----------------------------------------------------------------------
//template <>
//void tFeature<>::SomeExampleMethod()
//{
//  This is an example for a method. Replace it by your own methods!
//}

template <typename PointInT, typename PointOutT>
bool tFeature<PointInT, PointOutT>::initCompute()
{
  if (!PCLBase<PointInT>::initCompute())
  {
    PCL_ERROR("[pcl::%s::initCompute] Init failed.\n", getClassName().c_str());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty())
  {
    PCL_ERROR("[pcl::%s::compute] input_ is empty!\n", getClassName().c_str());
    // Cleanup
    deinitCompute();
    return (false);
  }

  // If no search surface has been defined, use the input dataset as the search surface itself
  if (!surface_)
  {
    fake_surface_ = true;
    surface_ = input_;
  }

  // Check if a space search locator was given
  if (!tree_)
  {
    if (surface_->isOrganized() && input_->isOrganized())
      tree_.reset(new pcl::search::OrganizedNeighbor<PointInT> ());
    else
      tree_.reset(new pcl::search::KdTree<PointInT> (false));
  }

  if (tree_->getInputCloud() != surface_)  // Make sure the tree searches the surface
    tree_->setInputCloud(surface_);


  // Do a fast check to see if the search parameters are well defined
  if (search_radius_ != 0.0)
  {
    if (k_ != 0)
    {
      PCL_ERROR("[pcl::%s::compute] ", getClassName().c_str());
      PCL_ERROR("Both radius (%f) and K (%d) defined! ", search_radius_, k_);
      PCL_ERROR("Set one of them to zero first and then re-run compute ().\n");
      // Cleanup
      deinitCompute();
      return (false);
    }
    else // Use the radiusSearch () function
    {
      search_parameter_ = search_radius_;
      // Declare the search locator definition
      int (KdTree::*radiusSearchSurface)(const PointCloudIn & cloud, int index, double radius,
                                         std::vector<int> &k_indices, std::vector<float> &k_distances,
                                         unsigned int max_nn) const = &pcl::search::Search<PointInT>::radiusSearch;
      search_method_surface_ = boost::bind(radiusSearchSurface, boost::ref(tree_), _1, _2, _3, _4, _5, 0);
    }
  }
  else
  {
    if (k_ != 0) // Use the nearestKSearch () function
    {
      search_parameter_ = k_;
      // Declare the search locator definition
      int (KdTree::*nearestKSearchSurface)(const PointCloudIn & cloud, int index, int k, std::vector<int> &k_indices,
                                           std::vector<float> &k_distances) const = &KdTree::nearestKSearch;
      search_method_surface_ = boost::bind(nearestKSearchSurface, boost::ref(tree_), _1, _2, _3, _4, _5);
    }
    else
    {
      PCL_ERROR("[pcl::%s::compute] Neither radius nor K defined! ", getClassName().c_str());
      PCL_ERROR("Set one of them to a positive number first and then re-run compute ().\n");
      // Cleanup
      deinitCompute();
      return (false);
    }
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
bool tFeature<PointInT, PointOutT>::deinitCompute()
{
  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset();
    fake_surface_ = false;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
void tFeature<PointInT, PointOutT>::compute(PointCloudOut &output)
{
  if (!initCompute())
  {
    output.width = output.height = 0;
    output.points.clear();
    return;
  }

  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  if (output.points.size() != indices_->size())
    output.points.resize(indices_->size());

  // Check if the output will be computed for all points or only a subset
  // If the input width or height are not set, set output width as size
  if (indices_->size() != input_->points.size() || input_->width * input_->height == 0)
  {
    output.width = static_cast<uint32_t>(indices_->size());
    output.height = 1;
  }
  else
  {
    output.width = input_->width;
    output.height = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Perform the actual feature computation
  computeFeature(output);

  deinitCompute();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
