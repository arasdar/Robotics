/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_trav_slope_and_step_mapping_discrete()
{

  CloudPtr cloud_slope(new Cloud);
  *cloud_slope = *prev_cloud;
  CloudPtr cloud_step(new Cloud);
  *cloud_step = *prev_cloud;
  segments_slope_mapped_discrete.clear();
  segments_step_mapped_discrete.clear();

  float alpha_non = weight_non, alpha_semi = weight_semi, alpha_trav = weight_trav;
  float mapping_threshold = thresh_mapping; // set minimum to the fraction of app. segemnt pixels that also belong to a normal segment; lower shares will not have an influence
  float uncertainty_threshold = thresh_uncertainty;
  vector<double> slope_non_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> slope_semi_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> slope_trav_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> step_non_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> step_semi_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> step_trav_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> invalid_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> unknown_count(prev_cloud_segments_appear.size(), 0.0);

  for (unsigned pi = 0; pi < prev_cloud->width * prev_cloud->height; ++pi)
  {
    unsigned label_geom = prev_cloud_segment_labels[pi].label;
    unsigned label_appear = prev_cloud_segment_labels_appear[pi].label;
    if (label_geom == invalid_label)
    {
      invalid_count[label_appear] += 1;
    }
    else if (prev_cloud_segments[label_geom].indices.size() <= thresh_segments)
    {
      unknown_count[label_appear] += 1;
    }
    else
    {
      if (segments_geom_slope[label_geom] > thresh_slope_upper)
      {
        slope_trav_count[label_appear] += 1;
      }
      else if (segments_geom_slope[label_geom] <= thresh_slope_upper && segments_geom_slope[label_geom] > thresh_slope_lower)
      {
        slope_semi_count[label_appear] += 1;
      }
      else if (segments_geom_slope[label_geom] <= thresh_slope_lower)
      {
        slope_non_count[label_appear] += 1;
      }
      if (segments_geom_step[label_geom] < thresh_step_below_ground)
      {
        step_semi_count[label_appear] += 1;
      }
      else if (segments_geom_step[label_geom] < 0 && segments_geom_step[label_geom] >= thresh_step_below_ground)
      {
        step_trav_count[label_appear] += 1;
      }
      else if (segments_geom_step[label_geom] < thresh_step_above_ground_lower && segments_geom_step[label_geom] >= 0)
      {
        step_trav_count[label_appear] += 1;
      }
      else if (segments_geom_step[label_geom] < thresh_step_above_ground_upper && segments_geom_step[label_geom] >= thresh_step_above_ground_lower)
      {
        step_semi_count[label_appear] += 1;
      }
      else if (segments_geom_step[label_geom] >= thresh_step_above_ground_upper)
      {
        step_non_count[label_appear] += 1;
      }
    }
  } // for pi

  for (unsigned i = 0; i < prev_cloud_segments_appear.size(); i++)
  {

    if (invalid_count[i] / prev_cloud_segments_appear[i].indices.size() > uncertainty_threshold) // yellow segment indicates high degree of uncertainty
    {
      segments_slope_mapped_discrete.push_back(code_invalid);
      segments_step_mapped_discrete.push_back(code_invalid);
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
      }
    }
    else if (slope_trav_count[i] + slope_semi_count[i] + slope_non_count[i] < mapping_threshold * prev_cloud_segments_appear[i].indices.size())    // too little coverage -> dont rely on it; purple
    {
      segments_slope_mapped_discrete.push_back(code_unknown);
      segments_step_mapped_discrete.push_back(code_unknown);
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
      }
    }
    else
    {
      // slope
      if (alpha_non * slope_non_count[i] >= fmax(alpha_semi * slope_semi_count[i], alpha_trav * slope_trav_count[i]))
      {
        segments_slope_mapped_discrete.push_back(code_non);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
      else if (alpha_semi * slope_semi_count[i] >= fmax(alpha_non * slope_non_count[i], alpha_trav * slope_trav_count[i]))
      {
        segments_slope_mapped_discrete.push_back(code_semi);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
        }
      }
      else if (alpha_trav * slope_trav_count[i] >= fmax(alpha_semi * slope_semi_count[i], alpha_non * slope_non_count[i]))
      {
        segments_slope_mapped_discrete.push_back(code_trav);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_slope->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
      // step
      if (alpha_non * step_non_count[i] >= fmax(alpha_semi * step_semi_count[i], alpha_trav * step_trav_count[i]))
      {
        segments_step_mapped_discrete.push_back(code_non);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
      else if (alpha_semi * step_semi_count[i] >= fmax(alpha_non * step_non_count[i], alpha_trav * step_trav_count[i]))
      {
        segments_step_mapped_discrete.push_back(code_semi);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
        }
      }
      else if (alpha_trav * step_trav_count[i] >= fmax(alpha_semi * step_semi_count[i], alpha_non * step_non_count[i]))
      {
        segments_step_mapped_discrete.push_back(code_trav);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_step->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_step->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_step->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
    }
  }

  drawContoursAroundSegments(cloud_slope, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  drawContoursAroundSegments(cloud_step, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  prev_cloud_trav_slopeMapping_discrete = cloud_slope;
  prev_cloud_trav_stepMapping_discrete = cloud_step;
}




