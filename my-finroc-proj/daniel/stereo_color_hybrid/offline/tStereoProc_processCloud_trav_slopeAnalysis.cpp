/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_trav_slopeAnalysis()
{
  vector<PointIndices> segment_indices(prev_cloud_segments);
  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;
  segments_geom_slope.clear();
  segments_slope_geom_discrete.clear();


  for (unsigned i = 0; i < segment_indices.size(); ++i)
  {
    if (segment_indices[i].indices.size() > thresh_segments)
    {
      Eigen::Vector4f plane = prev_cloud_planes[i];
      Eigen::Vector4f gravity(0, -1, 0, 0);

      float cos_theta = abs(plane.dot(gravity));
      //cout << "cos_theta: "<< cos_theta << endl;

      segments_geom_slope.push_back(cos_theta);
      if (cos_theta > thresh_slope_upper)  // the angle is small
      {
        segments_slope_geom_discrete.push_back(code_trav);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].g = 255;
          prev_segm_sizes_trav.push_back(segment_indices[i].indices.size());
          prev_segm_sizes.push_back(segment_indices[i].indices.size());
        }
      }
      else if (cos_theta > thresh_slope_lower && cos_theta <= thresh_slope_upper)
      {
        segments_slope_geom_discrete.push_back(code_semi);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].b = 255;
          prev_segm_sizes_semi.push_back(segment_indices[i].indices.size());
          prev_segm_sizes.push_back(segment_indices[i].indices.size());
        }
      }
      else if (cos_theta <= thresh_slope_lower)
      {
        segments_slope_geom_discrete.push_back(code_non);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
          prev_segm_sizes_non.push_back(segment_indices[i].indices.size());
          prev_segm_sizes.push_back(segment_indices[i].indices.size());
        }
      }
    } // big enough
    else
    {
      segments_geom_slope.push_back(invalid_slope);
    }
  }

  drawContoursAroundSegments(cloud, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  if (display_plane_normals)
  {
    drawSurfaceNormals(viewer_proc_trav_slopeAnalysis, prev_cloud_segments, prev_cloud_planes, prev_cloud_planes_cent_pt, prev_idx_normal);
  }
  prev_cloud_trav_slopeAnalysis = cloud;
}




