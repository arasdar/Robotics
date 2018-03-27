/*
 * tStereoProc_run_initialize.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/openTraverse/pointCloudProcessing/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::openTraverse::pointCloudProcessing;

void tStereoProcessing::run_initialize()
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

  cout << "left_images[img_index]: " << left_images[images_idx] << std::endl;
  cout << "images_idx: " << images_idx << endl;
  cout << "img_pairs_num: " << img_pairs_num << endl;
  cout << "press c for continue or space for forward or b for backward to the next image ................. " << endl;
  cout << " ===================================================================================================== " << endl;
  cout << " ===================================================================================================== " << endl;
}





