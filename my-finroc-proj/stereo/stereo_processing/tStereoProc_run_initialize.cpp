/*
 * tStereoProc_run_initialize.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void mStereoProcessing::run_initialize()
{
  /* displaying the points selected by gui - mouse and */
  Cloud::Ptr clicked_points_3d(new Cloud);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);

  /*removing and clean up*/
  cb_args.clicked_points_3d->clear();
  viewer->removeAllPointClouds();  //ok

  text_id = 0;
  for (int i = 0; i < 100; i++)
  {
    viewer->removeText3D(text_id_str[i]);
  }//for

}





