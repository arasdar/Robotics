/*
 * mStereoProc_run_all.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;

void
mStereoGrayOffline::drawVisualization()
{

  /*! Draw visualizations*/
  if (cloud_mutex.try_lock())
  {
    if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
    {
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_ground_image);
      viewer->addPointCloud(prev_ground_image, rgb, "cloud"); //works too
    }//if

    if (!viewer_disp_proc->updatePointCloud(prev_disp_image, "cloud disparity processed"))
    {
      viewer_disp_proc->addPointCloud(prev_disp_image, "cloud disparity processed");
    }

    if (display_normals)
    {
      viewer->removePointCloud("normals");
      viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
    }

    if (!display_normals)
    {
      viewer->removePointCloud("normals");
    }

    /*
        if (prev_cloud->points.size() > 1000)
        {
          image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
        }
    */

    image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);

    /*! Show the ground plane normal*/
    pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
    pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                      prev_ground_centroid[1] + prev_ground_normal[1],
                      prev_ground_centroid[2] + prev_ground_normal[2]);
    pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                      prev_ground_centroid[1] + tilt_road_normal[1],
                      prev_ground_centroid[2] + tilt_road_normal[2]);

    viewer->removeShape("ground_norm");
    viewer->addArrow(np2, np1, 0.0, 1.0, 0, false, "ground_norm");
    viewer->removeShape("expected_ground_norm");
    viewer->addArrow(np3, np1, 1.0, 0.0, 0, false, "expected_ground_norm");

    cloud_mutex.unlock();
  } //if cloud_mutex

  viewer->spinOnce(1);
  viewer_disparity->spinOnce(1);
  viewer_disp_proc->spinOnce(1);
  viewer_original->spinOnce(1); //not used but can be used

  image_viewer->spinOnce(1);
  image_viewer_disparity->spinOnce(1);
  image_viewer_disparity_processed->spinOnce(1);
}// run





