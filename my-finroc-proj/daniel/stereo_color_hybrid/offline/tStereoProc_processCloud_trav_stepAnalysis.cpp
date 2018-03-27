/*
 * tStereoProc_processCloud_trav_stepAnalysis.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: aras
 */

#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

#include <pcl/sample_consensus/sac_model_plane.h> //pointToPlaneDistance

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;


void tStereoProcessing::processCloud_trav_stepAnalysis()
{

  vector<PointIndices> segment_indices(prev_cloud_segments);

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

  CloudPtr cloud_final(new Cloud);
  *cloud_final = *prev_cloud;

  segments_geom_step.clear();
  segments_step_geom_discrete.clear();
  /*
    //dominant plane distant from the camera or the origin
    PointXYZ left_camera(0, 0, 0);
    double cameraToDominantPlaneDist = pointToPlaneDistanceSigned(left_camera, prev_dominant_ground_plane);
    cout << "dominant ground plane distance from the left camera: " << cameraToDominantPlaneDist << endl;

  //  double prev_dominant_ground_plane_D = 1; //prev_dominant_ground_plane[3];
    cout << "dominant plane D: " << prev_dominant_ground_plane[3] << endl;
    cout << "dmonant plane centroid height - Y: "<< prev_dominant_ground_plane[1]<< endl;
  */


  //applying the point cloud quality check and dominant ground plane check

  //if (!prev_frame_isBad) // if not bad
  // {

  for (unsigned i = 0; i < segment_indices.size(); ++i)
  {
    if (segment_indices[i].indices.size() > thresh_segments)
    {
      float pointToPlaneDist = pointToPlaneDistanceSigned(prev_cloud_planes_cent_pt[i], prev_dominant_ground_plane);
      //cout << pointToPlaneDist << "  : point to plane distance" << endl;
      segments_geom_step.push_back(pointToPlaneDist);
      //if the segments are below the dominant segment
      if (pointToPlaneDist < 0)
      {
        if (pointToPlaneDist < thresh_step_below_ground)
        {
          segments_step_geom_discrete.push_back(code_semi);
          for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
          {
            cloud->points[segment_indices[i].indices[j]].b = 255;
            cloud_final->points[segment_indices[i].indices[j]].b = 255;
          }
        }
        else
        {
          segments_step_geom_discrete.push_back(code_trav);
          for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
          {
            cloud->points[segment_indices[i].indices[j]].g = 255;
            cloud_final->points[segment_indices[i].indices[j]].g = 255;
          }
        }
      }
      if (pointToPlaneDist >= 0 && pointToPlaneDist < thresh_step_above_ground_lower)   //trav if height limit of the robot or step limit of the robot
      {
        segments_step_geom_discrete.push_back(code_trav);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].g = 255;
          cloud_final->points[segment_indices[i].indices[j]].g = 255;
        }
      }
      if (pointToPlaneDist >= thresh_step_above_ground_lower && pointToPlaneDist < thresh_step_above_ground_upper)//semi
      {
        segments_step_geom_discrete.push_back(code_semi);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].b = 255;
          cloud_final->points[segment_indices[i].indices[j]].b = 255;
        }
      }
      if (pointToPlaneDist >= thresh_step_above_ground_upper) //non
      {
        segments_step_geom_discrete.push_back(code_non);
        for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
          cloud_final->points[segment_indices[i].indices[j]].r = 255;

        }
      }

    }// if thresh

    else
    {
      segments_geom_step.push_back(invalid_step);
      for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
      {
        cloud_final->points[segment_indices[i].indices[j]].r = 255;
        cloud_final->points[segment_indices[i].indices[j]].g = 255;
        cloud_final->points[segment_indices[i].indices[j]].b = 255;
      }
    }

  }// for i

  // unknown area or area without disparity value or distance value
  for (unsigned i = 0; i < cloud->points.size(); ++i)
  {
    if (!isFinite(cloud->points[i]))
    {
//        cloud->points[i].r = 0;
//        cloud->points[i].g = 0;
//        cloud->points[i].b = 0;

      cloud_final->points[i].r = 0;
      cloud_final->points[i].g = 0;
      cloud_final->points[i].b = 0;

    }
  }
  //}// if frame is bad
  drawContoursAroundSegments(cloud, prev_cloud_segment_labels_appear, prev_cloud_segments_appear);
  drawContoursAroundSegments(cloud_final, prev_cloud_segment_labels, prev_cloud_segments);
  prev_cloud_trav_stepAnalysis = cloud;
  prev_cloud_trav_final = cloud_final;
}


