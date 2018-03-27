/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_invalid_points()
{
  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;



  for (unsigned i = 0; i < cloud->width * cloud->height; i ++)
  {

    if ((!pcl_isfinite(prev_normal_cloud->points[i].data_c[0]) ||
         !pcl_isfinite(prev_normal_cloud->points[i].data_c[1]) ||
         !pcl_isfinite(prev_normal_cloud->points[i].data_c[2])) &&
        pcl_isfinite(prev_cloud->points[i].x))
    {
      // red indicates pixels with 3D coordinates but no normal
      cloud->points[i].b = 0;
      cloud->points[i].r = 255;
      cloud->points[i].g = 0;
    }
    else if ((!pcl_isfinite(prev_normal_cloud->points[i].data_c[0]) ||
              !pcl_isfinite(prev_normal_cloud->points[i].data_c[1]) ||
              !pcl_isfinite(prev_normal_cloud->points[i].data_c[2])) &&
             !pcl_isfinite(prev_cloud->points[i].x))
    {
      // yellow indicates pixels with neither 3D coordinates and (consequently) no normals
      cloud->points[i].b = 0;
      cloud->points[i].r = 255;
      cloud->points[i].g = 255;
    }
    else if ((pcl_isfinite(prev_normal_cloud->points[i].data_c[0]) &&
              pcl_isfinite(prev_normal_cloud->points[i].data_c[1]) &&
              pcl_isfinite(prev_normal_cloud->points[i].data_c[2])) &&
             !pcl_isfinite(prev_cloud->points[i].x))
    {
      // green indicates pixels wih normals but no 3Dcoordinates; actually not possible, so no pixels should be green
      cloud->points[i].b = 0;
      cloud->points[i].r = 0;
      cloud->points[i].g = 255;
    }

  }

  prev_cloud_invalid_points = cloud;
}




