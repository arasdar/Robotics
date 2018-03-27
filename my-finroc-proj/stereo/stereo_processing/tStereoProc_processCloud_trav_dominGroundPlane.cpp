/*
 * tStereoProc_dominGroundPlane.cpp
 *
 *  Created on: Aug 25, 2014
 *      Author: aras
 */



#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void mStereoProcessing::processCloud_trav_dominGroundPlane()
{
  vector<PointIndices> segment_indices(prev_cloud_segments);

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;


  unsigned segm_sizes_max  = 0;
  if (prev_segm_sizes.size() > 0)
  {
    segm_sizes_max = *max_element(prev_segm_sizes.begin(), prev_segm_sizes.end());
    prev_segm_sizes.clear();
  }

  unsigned segm_sizes_max_trav = 0;
  if (prev_segm_sizes_trav.size() > 0)
  {
    segm_sizes_max_trav = *max_element(prev_segm_sizes_trav.begin(), prev_segm_sizes_trav.end());
    prev_segm_sizes_trav.clear();
  }

  unsigned segm_sizes_max_non = 0;
  if (prev_segm_sizes_non.size() > 0)
  {
    segm_sizes_max_non = *max_element(prev_segm_sizes_non.begin(), prev_segm_sizes_non.end());
    prev_segm_sizes_non.clear();
  }

  for (unsigned i = 0; i < segment_indices.size(); ++i)
  {
    if (segm_sizes_max_trav > 0 && segment_indices[i].indices.size() == segm_sizes_max_trav)
    {
      if (segm_sizes_max_trav == segm_sizes_max)
      {
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
          cloud->points[segment_indices[i].indices[j]].g = 255;
          cloud->points[segment_indices[i].indices[j]].b = 255;
        }
      }
      else
      {
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].g = 255;
        }
      }

      prev_dominant_ground_plane = prev_cloud_planes[i];
      prev_dominant_ground_plane_cent = prev_cloud_planes_cent_pt[i];
    }//trav

    if (segm_sizes_max_non > 0 && segment_indices[i].indices.size() == segm_sizes_max_non)
    {
      if (segm_sizes_max_non == segm_sizes_max)
      {
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
          cloud->points[segment_indices[i].indices[j]].g = 255;
          cloud->points[segment_indices[i].indices[j]].b = 255;
        }
      }
      else
      {
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
        }
      }

      prev_dominant_obstacle_plane = prev_cloud_planes[i];
    }//non
  }// for i

  /*
   * point cloud quality check based on the size of dominant ground segment
   */
//  unsigned thresh = cloud->width * cloud->height * 0.15; //min inlying point in the dominant segment
  //    unsigned int threshold_dominant_plane_cloud = (cloud->width / 10) * (cloud->height / 10) * 1.5 * 10;
//  unsigned thresh = cloud->width * cloud->height * 0.1 * 0.5;
  unsigned thresh = cloud->size() * 0.01;
  float camera_height = 1.0f; //meters

  bool badFrame = false;

//  cout << "dominant segment size is : " << segm_sizes_max_trav << endl;
//  cout << "threshold size: " << thresh << endl;
////  cout << "dominant ground plane height - plane normal Y: " << prev_dominant_ground_plane[1] << endl;
//  cout << "prev_dominant plane centroid point - Y: " <<   prev_dominant_ground_plane_cent.y << endl;

  if (segm_sizes_max_trav < thresh)
  {
//    if (prev_dominant_ground_plane[1] > 0 && prev_dominant_ground_plane_cent.y < camera_height)
//    {   }

    badFrame = true;
    cout << "bad point cloud ......" << endl;
  }

//  drawContoursAroundSegments(cloud, prev_cloud_segment_labels, prev_cloud_segments);
  prev_cloud_trav_dominGroundPlane = cloud;
  prev_frame_isBad = badFrame;
}



