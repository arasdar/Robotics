/*
 * mStereoProc_pointPickingCallback.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */




#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

//using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;
using namespace finroc::stereo_traversability_experiments::stereo_processing;

void mStereoProcessing::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
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
    case 'n':
      display_normals = !display_normals;
      break;
    }
  }
}//keyboard callback


void mStereoProcessing::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* args)
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

  cout << str <<  endl;
  text_id ++;
} //point picking callback
