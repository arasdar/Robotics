/*
 * tStereoProc_processCloud_trav_stepAnalysis.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: aras
 */

#include "projects/icarus/sensor_processing/pointCloudProcessing/tStereoProcessing.h"

#include <pcl/sample_consensus/sac_model_plane.h> //pointToPlaneDistance

using namespace finroc::icarus::sensor_processing::pointCloudProcessing;

void tStereoProcessing::processCloud_trav_stepAnalysis()
{

  vector<PointIndices> segment_indices(prev_cloud_segments);

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

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
  if (!prev_frame_isBad) // if not bad
  {

    for (unsigned i = 0; i < segment_indices.size(); ++i)
    {
      if (segment_indices[i].indices.size() > thresh_segments)
      {
        Eigen::Vector4f plane = prev_cloud_planes[i];
        Eigen::Vector4f gravity(0, -1, 0, 0);

        float cos_theta = abs(plane.dot(gravity));

        if (cos_theta > 0.7)
        {
          float pointToPlaneDist = pointToPlaneDistanceSigned(prev_cloud_planes_cent_pt[i], prev_dominant_ground_plane);
          //cout << pointToPlaneDist << "  : point to plane distance" << endl;

          //if the segments are below the dominant segment
          if (pointToPlaneDist < 0)
          {
            if (pointToPlaneDist < -0.2)
            {
              for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
              {
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
          }
          if (pointToPlaneDist >= 0 && pointToPlaneDist < 0.5)   //trav if height limit of the robot or step limit of the robot
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].g = 255;
            }
          }
          if (pointToPlaneDist >= 0.5 && pointToPlaneDist < 1)//semi
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].b = 255;
            }
          }
          if (pointToPlaneDist >= 1) //non
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].r = 255;
            }
          }

        }// if trav

        if (cos_theta <= 0.7 && cos_theta > 0.3)
        {
          float pointToPlaneDist = pointToPlaneDistanceSigned(prev_cloud_planes_cent_pt[i], prev_dominant_ground_plane);

          //if the segments are below the dominant segment
          if (pointToPlaneDist < 0)
          {
            if (pointToPlaneDist < -0.2)
            {
              for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
              {
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
          }
          if (pointToPlaneDist >= 0 && pointToPlaneDist < 0.5) //trav
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].g = 255; //traversable
            }
          }
          if (pointToPlaneDist >= 0.5 && pointToPlaneDist < 1.5) //TODO seg fault
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].b = 255;
            }
          }
          if (pointToPlaneDist >= 1.5)//non
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].r = 255; //non-traversable or obstacle
            }
          }
        }//if semi-trav

        if (cos_theta <= 0.3) //non slope
        {
          float pointToPlaneDist = pointToPlaneDistanceSigned(prev_cloud_planes_cent_pt[i], prev_dominant_ground_plane);

          //if the segments are below the dominant segment
          if (pointToPlaneDist < 0)
          {
            if (pointToPlaneDist < -0.2)
            {
              for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
              {
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
          }
          if (pointToPlaneDist >= 0 && pointToPlaneDist < 0.5) // semi
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].b = 255;
            }
          }
          if (pointToPlaneDist >= 0.5) //non
          {
            for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
            {
              cloud->points[segment_indices[i].indices[j]].r = 255;
            }
          }
        }
      }// if thresh
    }// if i

    // unknown area or area without disparity value or distance value
    for (unsigned i = 0; i < cloud->points.size(); ++i)
    {
      if (!isFinite(cloud->points[i]))
      {
        cloud->points[i].r = 0;
        cloud->points[i].g = 0;
        cloud->points[i].b = 0;
      }
    }
  }// if frame is bad

  prev_cloud_trav_stepAnalysis = cloud;
}


