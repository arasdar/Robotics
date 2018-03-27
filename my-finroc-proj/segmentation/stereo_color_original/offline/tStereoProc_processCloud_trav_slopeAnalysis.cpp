/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_original/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_original::offline;

void tStereoProcessing::processCloud_trav_slopeAnalysis()
{
  vector<PointIndices> segment_indices(prev_cloud_segments);
  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;


  for (unsigned i = 0; i < segment_indices.size(); ++i)
  {
    if (segment_indices[i].indices.size() > thresh_segments)
    {
      Eigen::Vector4f plane = prev_cloud_planes[i];
      Eigen::Vector4f gravity(0, -1, 0, 0);

      float cos_theta = abs(plane.dot(gravity));
      //cout << "cos_theta: "<< cos_theta << endl;

      for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
      {
        if (cos_theta > 0.7)  // the angle is small
        {
          cloud->points[segment_indices[i].indices[j]].g = 255;
          prev_segm_sizes_trav.push_back(segment_indices[i].indices.size());
        }

        if (cos_theta > 0.3 && cos_theta <= 0.7)
        {
          cloud->points[segment_indices[i].indices[j]].b = 255;
          prev_segm_sizes_semi.push_back(segment_indices[i].indices.size());
        }

        if (cos_theta <= 0.3)
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
          prev_segm_sizes_non.push_back(segment_indices[i].indices.size());
        }

        prev_segm_sizes.push_back(segment_indices[i].indices.size());
      }
    }
  }

  prev_cloud_trav_slopeAnalysis = cloud;
}




