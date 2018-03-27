
#include "projects/stereo_traversability_experiments/openTraverse/pointCloudProcessing/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::openTraverse::pointCloudProcessing;

void
tStereoProcessing::run_visualize()
{

  if (!viewer->updatePointCloud(prev_cloud, "cloud"))
  {
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 1.0, 0.333333, v1);
    viewer->setBackgroundColor(1.0, 1.0, 1.0, v1);
    visualization::PointCloudColorHandlerRGBField<PointT> rgb1(prev_cloud);
    viewer->addPointCloud<PointT> (prev_cloud, rgb1, "cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  }
  image_viewer->addRGBImage<PointT>(prev_cloud, "rgb_image", 0.3);


  if (!viewer->updatePointCloud(prev_cloud_filtered, "cloud filtered"))
  {
    int v2(0);
    viewer->createViewPort(0.0, 0.333333, 1.0, 0.666666, v2);
    viewer->setBackgroundColor(1.0, 1.0, 1.0, v2);
    visualization::PointCloudColorHandlerRGBField<PointT> rgb2(prev_cloud_filtered);
    viewer->addPointCloud<PointT> (prev_cloud_filtered, rgb2, "cloud filtered", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud filtered");
  }

  if (!viewer->updatePointCloud(prev_cloud_trav, "C-cloud"))
  {
    int v3(0);
    viewer->createViewPort(0.0, 0.666666, 1.0, 1.0, v3);
    viewer->setBackgroundColor(1.0, 1.0, 1.0, v3);
    visualization::PointCloudColorHandlerRGBField<PointT> rgb3(prev_cloud_trav);
    viewer->addPointCloud<PointT> (prev_cloud_trav, rgb3, "C-cloud", v3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "C-cloud");
  }

  viewer->spinOnce(1);
  image_viewer->spinOnce(1);

}// run





