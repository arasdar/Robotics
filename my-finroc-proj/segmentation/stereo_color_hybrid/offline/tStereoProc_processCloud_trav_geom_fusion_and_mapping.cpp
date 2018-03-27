/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_trav_geom_fuse_and_map()
{

  CloudPtr cloud_fused(new Cloud);
  *cloud_fused = *prev_cloud;
  CloudPtr cloud_mapped(new Cloud);
  *cloud_mapped = *prev_cloud;
  CloudPtr cloud_mapped_uh(new Cloud);
  *cloud_mapped_uh = *prev_cloud;
  segments_fused_geom_discrete.clear();
  segments_mapped_geom_discrete.clear();
  segments_mapped_geom_discrete_unknown_handled.clear();


  float alpha_non = weight_non, alpha_semi = weight_semi, alpha_trav = weight_trav;
  float mapping_threshold = thresh_mapping; // set minimum to the fraction of app. segemnt pixels that also belong to a normal segment; lower shares will not have an influence
  float uncertainty_threshold = thresh_uncertainty;
  vector<double> non_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> semi_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> trav_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> invalid_count(prev_cloud_segments_appear.size(), 0.0);
  vector<double> unknown_count(prev_cloud_segments_appear.size(), 0.0);

  // fusion of geom. slope and step + visualization + count trav. class indicators
  unsigned index = 0;
  for (unsigned i = 0; i < prev_cloud_segments.size(); i++)
  {
    if (prev_cloud_segments[i].indices.size() > thresh_segments)
    {
      unsigned code = fmin(segments_slope_geom_discrete[index], segments_step_geom_discrete[index]);
      segments_fused_geom_discrete.push_back(code);
      unsigned label_appear;
      if (code == code_trav)
      {
        for (unsigned j = 0; j < prev_cloud_segments[i].indices.size(); ++j)
        {
          label_appear = prev_cloud_segment_labels_appear[prev_cloud_segments[i].indices[j]].label;
          trav_count[label_appear] += 1;
          cloud_fused->points[prev_cloud_segments[i].indices[j]].g = 255; //fmin(3 * fmax(cloud_fused->points[prev_cloud_segments[i].indices[j]].r, cloud_fused->points[prev_cloud_segments[i].indices[j]].b), 255);
        }
      }
      else if (code == code_semi)
      {
        for (unsigned j = 0; j < prev_cloud_segments[i].indices.size(); ++j)
        {
          label_appear = prev_cloud_segment_labels_appear[prev_cloud_segments[i].indices[j]].label;
          semi_count[label_appear] += 1;
          cloud_fused->points[prev_cloud_segments[i].indices[j]].b = 255; //fmin(3 * fmax(cloud_fused->points[prev_cloud_segments[i].indices[j]].g, cloud_fused->points[prev_cloud_segments[i].indices[j]].r), 255);
        }
      }
      else if (code == code_non)
      {
        for (unsigned j = 0; j < prev_cloud_segments[i].indices.size(); ++j)
        {
          label_appear = prev_cloud_segment_labels_appear[prev_cloud_segments[i].indices[j]].label;
          non_count[label_appear] += 1;
          cloud_fused->points[prev_cloud_segments[i].indices[j]].r = 255; //fmin(3 * fmax(cloud_fused->points[prev_cloud_segments[i].indices[j]].g, cloud_fused->points[prev_cloud_segments[i].indices[j]].b), 255);
        }
      }
      index ++;
    }
  } // for i

  index = 0;
  // count invalid class indicators
  for (unsigned pi = 0; pi < prev_cloud->width * prev_cloud->height; ++pi)
  {
    unsigned label_geom = prev_cloud_segment_labels[pi].label;
    unsigned label_appear = prev_cloud_segment_labels_appear[pi].label;
    if (label_geom == invalid_label)
    {
      invalid_count[label_appear] += 1;
    }
  }



  // determine mapped trav. class, visualize
  for (unsigned i = 0; i < prev_cloud_segments_appear.size(); i++)
  {

    if (invalid_count[i] / prev_cloud_segments_appear[i].indices.size() > uncertainty_threshold) // yellow segment indicates high degree of uncertainty
    {
      segments_mapped_geom_discrete.push_back(code_invalid);
      segments_mapped_geom_discrete_unknown_handled.push_back(code_invalid);
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
        cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b, 255);
      }
    }
    else if (trav_count[i] + semi_count[i] + non_count[i] < mapping_threshold * prev_cloud_segments_appear[i].indices.size())    // too little coverage -> dont rely on it; purple
    {
      segments_mapped_geom_discrete.push_back(code_unknown);
      unsigned code_from_appear = segments_slope_appear_discrete[i];
      segments_mapped_geom_discrete_unknown_handled.push_back(code_from_appear);
      for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
      {
        cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
        cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].g, 255);
      }
      if (code_from_appear == code_trav)
      {
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
      else if (code_from_appear == code_semi)
      {
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r), 255);
        }
      }
      else if (code_from_appear == code_non)
      {
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
    }
    else
    {
      if (alpha_non * non_count[i] >= fmax(alpha_semi * semi_count[i], alpha_trav * trav_count[i]))
      {
        segments_mapped_geom_discrete.push_back(code_non);
        segments_mapped_geom_discrete_unknown_handled.push_back(code_non);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
          cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r = fmin(3 * fmax(cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g, cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
      else if (alpha_semi * semi_count[i] >= fmax(alpha_non * non_count[i], alpha_trav * trav_count[i]))
      {
        segments_mapped_geom_discrete.push_back(code_semi);
        segments_mapped_geom_discrete_unknown_handled.push_back(code_semi);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
          cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b = fmin(3 * fmax(cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g), 255);
        }
      }
      else if (alpha_trav * trav_count[i] >= fmax(alpha_semi * semi_count[i], alpha_non * non_count[i]))
      {
        segments_mapped_geom_discrete.push_back(code_trav);
        segments_mapped_geom_discrete_unknown_handled.push_back(code_trav);
        for (unsigned j = 0; j < prev_cloud_segments_appear[i].indices.size(); ++j)
        {
          cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_mapped->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
          cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].g = fmin(3 * fmax(cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].r, cloud_mapped_uh->points[prev_cloud_segments_appear[i].indices[j]].b), 255);
        }
      }
    } // else
  } // for i



  drawContoursAroundSegments(cloud_fused, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  drawContoursAroundSegments(cloud_mapped, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  drawContoursAroundSegments(cloud_mapped_uh, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  prev_cloud_trav_geomFusion_discrete = cloud_fused;
  prev_cloud_trav_fusedMapping_discrete = cloud_mapped;
  prev_cloud_trav_fusedMapping_discrete_uh = cloud_mapped_uh;
}




