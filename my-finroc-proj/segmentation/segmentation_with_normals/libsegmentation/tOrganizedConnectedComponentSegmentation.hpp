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
/*!\file    projects/icarus/sensor_processing/libsegmentation/tOrganizedConnectedComponentSegmentation.hpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-29
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
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
namespace segmentation_with_normals
{
namespace libsegmentation
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

/**
 *  Directions: 1 2 3
 *              0 x 4
 *              7 6 5
 * e.g. direction y means we came from pixel with label y to the center pixel x
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename PointLT> void
OrganizedConnectedComponentSegmentation<PointT, PointLT>::segment(pcl::PointCloud<PointLT>& labels, std::vector<pcl::PointIndices>& label_indices) const
{
  std::vector<unsigned> run_ids;

  unsigned invalid_label = std::numeric_limits<unsigned>::max();
  pcl::Label invalid_pt;
  invalid_pt.label = invalid_label; //std::numeric_limits<unsigned>::max();
  labels.points.resize(input_->points.size(), invalid_pt);
  labels.width = input_->width;
  labels.height = input_->height;
  unsigned int clust_id = 0;

  //First pixel
  if (pcl_isfinite(input_->points[0].x))
  {
    labels[0].label = clust_id++;
    run_ids.push_back(labels[0].label);
  }

  // First row
  for (int colIdx = 1; colIdx < static_cast<int>(input_->width); ++colIdx)
  {
    if (!pcl_isfinite(input_->points[colIdx].x))
      continue;
    else if (compare_->compare(colIdx, colIdx - 1))
    {
      labels[colIdx].label = labels[colIdx - 1].label;
    }
    else
    {
      labels[colIdx].label = clust_id++;
      run_ids.push_back(labels[colIdx].label);
    }
  }

  // Everything else
  unsigned int current_row = 0; //input_->width;
  unsigned int previous_row = 0;
  for (size_t rowIdx = 1; rowIdx < input_->height; ++rowIdx, previous_row = current_row, current_row += input_->width)
  {
    // First pixel of the row
    if (pcl_isfinite(input_->points[current_row].x))
    {
      if (compare_->compare(current_row, previous_row))
      {
        labels[current_row].label = labels[previous_row].label;
      }
      else
      {
        labels[current_row].label = clust_id++;
        run_ids.push_back(labels[current_row].label);
      }
    }

    // Rest of the pixels in the row
    for (int colIdx = 1; colIdx < static_cast<int>(input_->width); ++colIdx)
    {
      if (pcl_isfinite(input_->points[current_row + colIdx].x))
      {
        //compare with previous pixel in the same row
        if (compare_->compare(current_row + colIdx, current_row + colIdx - 1))
        {
          if (labels[current_row + colIdx].label == invalid_label)
            labels[current_row + colIdx].label = labels[current_row + colIdx - 1].label;
          else
          {
            unsigned root1 = findRoot(run_ids, labels[current_row + colIdx].label);
            unsigned root2 = findRoot(run_ids, labels[current_row + colIdx - 1].label);

            if (root1 < root2)
              run_ids[root2] = root1;
            else
              run_ids[root1] = root2;
          }
        }

        //compare with pixel in previous row
        if (compare_->compare(current_row + colIdx, previous_row + colIdx))
        {
          if (labels[current_row + colIdx].label == invalid_label)
            labels[current_row + colIdx].label = labels[previous_row + colIdx].label;
          else
          {
            unsigned root1 = findRoot(run_ids, labels[current_row + colIdx].label);
            unsigned root2 = findRoot(run_ids, labels[previous_row + colIdx].label);

            if (root1 < root2)
              run_ids[root2] = root1;
            else
              run_ids[root1] = root2;
          }
        }

        // if not already labeled at all
        if (labels[current_row + colIdx].label == invalid_label)
        {
          labels[current_row + colIdx].label = clust_id++;
          run_ids.push_back(labels[current_row + colIdx].label);
        }

      }
    }
  }

  std::vector<unsigned> map(clust_id);
  unsigned max_id = 0;
  for (unsigned runIdx = 0; runIdx < run_ids.size(); ++runIdx)
  {
    // if it is its own root -> new region
    if (run_ids[runIdx] == runIdx)
      map[runIdx] = max_id++;
    else // assign this sub-segment to the region (root) it belongs
      map [runIdx] = map [findRoot(run_ids, runIdx)];
  }

  label_indices.resize(max_id + 1);
  for (unsigned idx = 0; idx < input_->points.size(); idx++)
  {
    if (labels[idx].label != invalid_label)
    {
      labels[idx].label = map[labels[idx].label];
      label_indices[labels[idx].label].indices.push_back(idx);
    }
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}

