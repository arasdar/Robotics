/*
 * mStereoProc_pointPickingCallback.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */

#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_texture;

void tStereoProcessing::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
    case ' ': //forward
      trigger = true;
      break;
    case 'b': //backward
      bwd = true;
      break;
    case '1':
      smooth_strong -= 5;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '2':
      smooth_strong += 5;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '3':
      smooth_weak -= 5;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    case '4':
      smooth_weak += 5;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    case 'c':
      continuous = !continuous;
      break;
    case 'n':
      display_normals = !display_normals;
      break;

    }
  }
}//keyboard callback

void tStereoProcessing::pointPickingcallback(const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex() == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);


  /*Draw clicked points in red:*/
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 0, 0, 255); //blue now
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

  /*Draw clicked points in red on viewer_2:*/
  data->viewerPtr_2->removePointCloud("clicked_points_2");
  data->viewerPtr_2->addPointCloud(data->clicked_points_3d, red, "clicked_points_2");
  data->viewerPtr_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_2");

  /*Draw clicked points in red on viewer_3:*/
  data->viewerPtr_3->removePointCloud("clicked_points_3");
  data->viewerPtr_3->addPointCloud(data->clicked_points_3d, red, "clicked_points_3");
  data->viewerPtr_3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_3");

  /*Draw clicked points in red on viewer_4:*/
  data->viewerPtr_4->removePointCloud("clicked_points_4");
  data->viewerPtr_4->addPointCloud(data->clicked_points_3d, red, "clicked_points_4");
  data->viewerPtr_4->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_4");

  std::cout << "(X, Y, Z) -------> X: " << current_point.x << " -- Y: " << current_point.y << " -- Z: " << current_point.z << std::endl;
  std::cout << "(R, G, B) -------> R: " << current_point.r << " -- G: " << current_point.g << " -- B: " << current_point.b << std::endl;


  /* add text:*/
  std::stringstream ss;
  ss << current_point.z;

  double  textScale = 0.3;  //0.4; //0.1; //1.0;
  double  r = 1.0;
  double  g = 1.0;
  double  b = 1.0;

  char str[512];
  sprintf(str, "text_id_%03d", text_id);
  text_id_str [text_id] = str;
  data->viewerPtr->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
  data->viewerPtr_2->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
  data->viewerPtr_3->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
  data->viewerPtr_4->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
  cout << str <<  endl;
  text_id ++;
} //point picking callback
