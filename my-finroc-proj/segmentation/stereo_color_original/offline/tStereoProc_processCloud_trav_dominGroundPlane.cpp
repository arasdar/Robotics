/*
 * tStereoProc_dominGroundPlane.cpp
 *
 *  Created on: Aug 25, 2014
 *      Author: aras
 */



#include "projects/stereo_traversability_experiments/daniel/stereo_color_original/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_original::offline;

void tStereoProcessing::processCloud_trav_dominGroundPlane()
{
  vector<PointIndices> segment_indices(prev_cloud_segments);

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

  unsigned segm_sizes_max = *max_element(prev_segm_sizes.begin(), prev_segm_sizes.end());
  prev_segm_sizes.clear();

  unsigned segm_sizes_max_trav = *max_element(prev_segm_sizes_trav.begin(), prev_segm_sizes_trav.end());
  prev_segm_sizes_trav.clear();

  unsigned segm_sizes_max_semi = 0;
  if (prev_segm_sizes_semi.size() > 0)
  {
    segm_sizes_max_semi = *max_element(prev_segm_sizes_semi.begin(), prev_segm_sizes_semi.end());
    prev_segm_sizes_semi.clear();
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

    if (segm_sizes_max_semi > 0 && segment_indices[i].indices.size() == segm_sizes_max_semi)
    {
      if (segm_sizes_max_semi == segm_sizes_max)
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
          cloud->points[segment_indices[i].indices[j]].b = 255;
        }
      }
    }// semi

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
  unsigned thresh = cloud->width * cloud->height * 0.1 * 0.5;

  bool badFrame = false;

  viewer_proc_trav->removeText3D("bad cloud");
  image_viewer_proc_trav->removeLayer("bad cloud line");

  cout << "dominant segment size is : " << segm_sizes_max_trav << endl;
  cout << "threshold size: " << thresh << endl;
  cout << "dominant ground plane height - plane normal Y: " << prev_dominant_ground_plane[1] << endl;

  cout << "prev_dominant plane centroid point - Y: " <<   prev_dominant_ground_plane_cent << endl;

  if (segm_sizes_max_trav < thresh || prev_dominant_ground_plane[1] > 0 || prev_dominant_ground_plane_cent.y < -0.5)
  {
    badFrame = true;

    cout << "bad point cloud ......" << endl;

    /*! add text to the point cloud */
    std::stringstream ss;
    ss << "BAD FRAME" ;
    double  textScale = 0.5; //0.3;  //0.4; //0.1; //1.0;
    double  r = 1.0;
    double  g = 0.0;
    double  b = 0.0;
    PointT position; //(centroids[0])
    position.x = -2; //centroids[0][0]; //0;
    position.y = 0; //centroids[0][1];//0;
    position.z = 5; //centroids[0][2];//1;
    viewer_proc_trav->addText3D(ss.str(), position, textScale, r, g, b,  "bad cloud");

    unsigned int x_min = 0, y_min = 0, x_max = cloud->width, y_max = cloud->height; //image.width=800, image.height=600
    double opacity = 1.0;
    image_viewer_proc_trav->addLine(x_min, y_min, x_max, y_max, r, g, b, "bad cloud line", opacity);
    x_min = cloud->width;
    y_min = 0;
    x_max = 0;
    y_max = cloud->height;
    image_viewer_proc_trav->addLine(x_min, y_min, x_max, y_max, r, g, b, "bad cloud line", opacity);
  }

  prev_cloud_trav_dominGroundPlane = cloud;
  prev_frame_isBad = badFrame;
}



