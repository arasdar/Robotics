/*
 * tStereoProc_run_initialize.cpp
 *
 *  Created on: Aug 27, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/stereo_gray/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::stereo_gray::offline;

void tStereoProcessing::run_initialize()
{
  /*! displaying disparity map*/
  /*! visualizing in opencv*/
  /*
        Mat vmap_opencv(left_img_rect.rows, left_img_rect.cols, CV_8UC3);
        stereo.getVisualMap(vmap_opencv);
        imshow("vmap_opencv", vmap_opencv);
        waitKey(1);
  */

  /* displaying the points selected by gui - mouse and */
  Cloud::Ptr clicked_points_3d(new Cloud);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
  cb_args.viewerPtr_2 = pcl::visualization::PCLVisualizer::Ptr(viewer_disparity);
  cb_args.viewerPtr_5 = pcl::visualization::PCLVisualizer::Ptr(viewer_disparity_processed);
  cb_args.viewerPtr_3 = pcl::visualization::PCLVisualizer::Ptr(viewer_proc_segm);
  cb_args.viewerPtr_4 = pcl::visualization::PCLVisualizer::Ptr(viewer_proc_trav);


  /*removing and clean up*/
  cb_args.clicked_points_3d->clear();
  viewer->removeAllPointClouds();  //ok
  viewer_disparity->removeAllPointClouds();
  viewer_disparity_processed->removeAllPointClouds();
  viewer_proc_segm ->removeAllPointClouds();
  viewer_proc_trav->removeAllPointClouds();
  text_id = 0;
  for (int i = 0; i < 100; i++)
  {
    viewer->removeText3D(text_id_str[i]);
    viewer_disparity->removeText3D(text_id_str[i]);
    viewer_disparity_processed->removeText3D(text_id_str[i]);
    viewer_proc_segm->removeText3D(text_id_str[i]);
    viewer_proc_trav->removeText3D(text_id_str[i]);
  }//for

  cout << "left_images[img_index]: " << left_images[images_idx] << std::endl;
  cout << "right_images[img_index]: " << right_images[images_idx] << std::endl;
  cout << "images_idx: " << images_idx << endl;
  cout << "img_pairs_num: " << img_pairs_num << endl;
  cout << "press space to continue to the next image ................. " << endl;
}





