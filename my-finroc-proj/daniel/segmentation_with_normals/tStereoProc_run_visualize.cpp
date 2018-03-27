
#include "projects/stereo_traversability_experiments/daniel/segmentation_with_normals/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_normals;

void
tStereoProcessing::run_visualize()
{

  viewer->removeText3D("cloud");
  //image_viewer->removeLayer("line");  //const std::string &layer_id

  //visualizing generated point cloud and extracted normals on it optionally
  if (!viewer->updatePointCloud(prev_cloud, "cloud"))
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_cloud);
    viewer->addPointCloud(prev_cloud, rgb, "cloud"); //works too
  }//if

  viewer->removePointCloud("normals");
  if (display_normals)
  {
    viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_cloud, prev_normal_cloud, 10, 0.15f, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
  }
  image_viewer->addRGBImage<PointT>(prev_cloud, "rgb_image", 0.3);

  if (!viewer_disparity->updatePointCloud(prev_cloud_disp, "cloud disparity"))
  {
    viewer_disparity->addPointCloud(prev_cloud_disp, "cloud disparity");
  }
  image_viewer_disparity->addRGBImage<RGB>(prev_img_disp);

  // visualizing segmented regions on the point cloud
  if (!viewer_proc_segm->updatePointCloud(prev_cloud_segm, "cloud_segm")) //previous cloud for segmentation results
  {
    viewer_proc_segm->addPointCloud(prev_cloud_segm, "cloud_segm");
  }
  image_viewer_proc_segm->addRGBImage<PointT>(prev_cloud_segm);

  viewer->spinOnce(1);
  image_viewer->spinOnce(1);

  viewer_disparity->spinOnce(1);
  image_viewer_disparity->spinOnce(1);

  viewer_proc_segm->spinOnce(1);
  image_viewer_proc_segm->spinOnce(1);

  waitKey(1);

}// run





