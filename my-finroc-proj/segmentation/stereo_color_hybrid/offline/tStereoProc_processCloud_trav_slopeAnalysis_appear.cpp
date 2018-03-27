/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_trav_slopeAnalysis_appear()
{
  vector<PointIndices> segment_indices(prev_cloud_segments_appear);
  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;
  segments_appear_slope.clear();
  segments_slope_appear_discrete.clear();

  double uncertainty_threshold = thresh_uncertainty;

  for (unsigned i = 0; i < segment_indices.size(); ++i)
  {
    if (True) //segment_indices[i].indices.size() > thresh_segments)
    {
      Eigen::Vector4f plane = prev_cloud_planes_appear[i];
      Eigen::Vector4f gravity(0, -1, 0, 0);

      float cos_theta = abs(plane.dot(gravity));
      //cout << "cos_theta: "<< cos_theta << endl;

      double num_infinite = 0.0;
      for (unsigned j = 0; j < segment_indices[i].indices.size(); j++)
      {
        if (!pcl_isfinite(prev_cloud->points[segment_indices[i].indices[j]].x))
        {
          num_infinite += 1;
        }
      }


      if (num_infinite / segment_indices[i].indices.size() > uncertainty_threshold)
      {
        segments_slope_appear_discrete.push_back(code_invalid);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = fmin(3 * cloud->points[segment_indices[i].indices[j]].b, 255);
          cloud->points[segment_indices[i].indices[j]].g = fmin(3 * cloud->points[segment_indices[i].indices[j]].b, 255);
        }
        segments_appear_slope.push_back(invalid_slope);
      }
      else if (cos_theta > thresh_slope_upper)      // the angle is small
      {
        segments_slope_appear_discrete.push_back(code_trav);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].g = fmin(3 * fmax(cloud->points[segment_indices[i].indices[j]].r, cloud->points[segment_indices[i].indices[j]].b), 255);
          //prev_segm_sizes_trav.push_back(segment_indices[i].indices.size());
        }
        segments_appear_slope.push_back(cos_theta);
      }
      else  if (cos_theta > thresh_slope_lower && cos_theta <= thresh_slope_upper)
      {
        segments_slope_appear_discrete.push_back(code_semi);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].b = fmin(3 * fmax(cloud->points[segment_indices[i].indices[j]].r, cloud->points[segment_indices[i].indices[j]].g), 255);
          //prev_segm_sizes_semi.push_back(segment_indices[i].indices.size());
        }
        segments_appear_slope.push_back(cos_theta);
      }
      else if (cos_theta <= thresh_slope_lower)
      {
        segments_slope_appear_discrete.push_back(code_non);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = fmin(3 * fmax(cloud->points[segment_indices[i].indices[j]].g, cloud->points[segment_indices[i].indices[j]].b), 255);
          //prev_segm_sizes_non.push_back(segment_indices[i].indices.size());
        }
        segments_appear_slope.push_back(cos_theta);
      }

      //prev_segm_sizes.push_back(segment_indices[i].indices.size());

    } // if True
  } // for segments
  drawContoursAroundSegments(cloud, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  if (display_plane_normals)
  {
    drawSurfaceNormals(viewer_proc_trav_slopeAnalysis_appear, prev_cloud_segments_appear, prev_cloud_planes_appear, prev_cloud_planes_cent_pt_appear, prev_idx_appear);
  }
  prev_cloud_trav_slopeAnalysis_appear = cloud;
}




