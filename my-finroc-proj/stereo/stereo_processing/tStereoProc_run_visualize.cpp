
#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void
mStereoProcessing::run_visualize()
{

//  viewer->removeText3D("cloud");
  //image_viewer->removeLayer("line");  //const std::string &layer_id

  //visualizing generated point cloud and extracted normals on it optionally
  if (!viewer->updatePointCloud(prev_cloud_trav_final, "cloud"))
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_cloud_trav_final);
    viewer->addPointCloud(prev_cloud_trav_final, rgb, "cloud"); //works too
  }//if

  viewer->removePointCloud("normals");
  if (display_normals)
  {
    viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_cloud_trav_final, prev_normal_cloud, 10, 0.15f, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
  }
  image_viewer->addRGBImage<PointT>(prev_cloud_trav_final, "rgb_image", 0.3);
  image_viewer_disparity->addRGBImage<RGB>(prev_img_disp);


  viewer->spinOnce(1);
  image_viewer->spinOnce(1);

  image_viewer_disparity->spinOnce(1);

  waitKey(1);
}// run





