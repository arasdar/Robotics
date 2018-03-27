/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_trav_fuse_pessimistically()
{

  CloudPtr cloud_slope(new Cloud);
  *cloud_slope = *prev_cloud;
  CloudPtr cloud_step(new Cloud);
  *cloud_step = *prev_cloud;
  CloudPtr cloud_final(new Cloud);
  *cloud_final = *prev_cloud;
  segments_slope_fused_discrete.clear();
  segments_step_fused_discrete.clear();
  segments_final_fused_discrete.clear();



  for (unsigned i = 0; i < prev_cloud_segments_appear.size(); i++)
  {
    segments_slope_fused_discrete.push_back(fmin(segments_slope_mapped_discrete[i], segments_slope_appear_discrete[i]));
    segments_step_fused_discrete.push_back(fmin(segments_step_mapped_discrete[i], segments_step_appear_discrete[i]));
    segments_final_fused_discrete.push_back(fmin(segments_slope_fused_discrete[i], segments_step_fused_discrete[i]));
    //if (segments_step_fused_discrete[i] == code_trav)
    //{
    //  segments_final_fused_discrete[i] = code_trav;
    //}

    // visualize

    // slope fused
    if (segments_slope_fused_discrete[i] == code_invalid)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
      }
    }
    else if (segments_slope_fused_discrete[i] == code_unknown) // actually cannot occur
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
      }
    }
    else if (segments_slope_fused_discrete[i] == code_trav)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
      }
    }
    else if (segments_slope_fused_discrete[i] == code_semi)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
      }
    }
    else if (segments_slope_fused_discrete[i] == code_non)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
      }
    }

    // step fused
    if (segments_step_fused_discrete[i] == code_invalid)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
      }
    }
    else if (segments_step_fused_discrete[i] == code_unknown) // actually cannot occur
    {
std:
      cerr << "Errror!! Code unknown in step_fused" << std::endl;
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
      }
    }
    else if (segments_step_fused_discrete[i] == code_trav)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
      }
    }
    else if (segments_step_fused_discrete[i] == code_semi)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
      }
    }
    else if (segments_step_fused_discrete[i] == code_non)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
      }
    }


    // step and slope fused
    if (segments_final_fused_discrete[i] == code_invalid)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_final->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_final->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
      }
    }
    else if (segments_final_fused_discrete[i] == code_unknown) // actually cannot occur
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_final->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * cloud_final->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
      }
    }
    else if (segments_final_fused_discrete[i] == code_trav)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_final->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_final->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
      }
    }
    else if (segments_final_fused_discrete[i] == code_semi)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_final->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_final->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
      }
    }
    else if (segments_final_fused_discrete[i] == code_non)
    {
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_final->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_final->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_final->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
      }
    }
  }

  drawContoursAroundSegments(cloud_slope, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  drawContoursAroundSegments(cloud_step, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  drawContoursAroundSegments(cloud_final, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  prev_cloud_trav_slopeFusion_discrete = cloud_slope;
  prev_cloud_trav_stepFusion_discrete = cloud_step;
  prev_cloud_trav_finalFusion_discrete = cloud_final;
}




