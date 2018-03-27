
//#include "projects/icarus/sensor_processing/pointCloudProcessing/tStereoProcessing.h"
#include "projects/stereo_traversability_experiments/aras/pointCloudProcessing/tStereoProcessing.h"

//using namespace finroc::icarus::sensor_processing::pointCloudProcessing;
using namespace finroc::stereo_traversability_experiments::aras::pointCloudProcessing;

void
tStereoProcessing::run_visualize()
{

//  viewer->removeText3D("cloud");
//  //image_viewer->removeLayer("line");  //const std::string &layer_id

  //visualizing generated point cloud and extracted normals on it optionally
  if (!viewer->updatePointCloud(prev_cloud, "cloud"))
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_cloud);
    viewer->addPointCloud(prev_cloud, rgb, "cloud"); //works too
  }//if

//  viewer->removePointCloud("normals");
//  if (display_normals)
//  {
//    viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_cloud, prev_normal_cloud, 10, 0.15f, "normals");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
//  }
  image_viewer->addRGBImage<PointT>(prev_cloud, "rgb_image", 0.3);

////  if (!viewer_disparity->updatePointCloud(prev_cloud_disp, "cloud disparity"))
////  {
////    viewer_disparity->addPointCloud(prev_cloud_disp, "cloud disparity");
////  }
////  image_viewer_disparity->addRGBImage<RGB>(prev_img_disp);
////
////  if (!viewer_disparity_processed->updatePointCloud(prev_cloud_disp, "cloud disp proc"))
////  {
////    viewer_disparity_processed->addPointCloud(prev_cloud_disp, "cloud disp proc");
////  }
////  image_viewer_disparity_processed->addRGBImage<PointT>(prev_cloud_disp, "cloud disp proc");
//
//
  // visualizing segmented regions on the point cloud
  if (!viewer_proc_segm->updatePointCloud(prev_cloud_segm, "cloud_segm")) //previous cloud for segmentation results
  {
    viewer_proc_segm->addPointCloud(prev_cloud_segm, "cloud_segm");
  }
  image_viewer_proc_segm->addRGBImage<PointT>(prev_cloud_segm);

//  if (!viewer_proc_trav->updatePointCloud(prev_cloud_trav_dominGroundPlane, "cloud_trav"))
//  {
//    viewer_proc_trav->addPointCloud(prev_cloud_trav_dominGroundPlane, "cloud_trav");
//  }
//  image_viewer_proc_trav->addRGBImage<PointT>(prev_cloud_trav_dominGroundPlane);
//
//
//  if (!viewer_proc_trav_stepAnalysis->updatePointCloud(prev_cloud_trav_stepAnalysis, "cloud trav step"))
//  {
//    viewer_proc_trav_stepAnalysis->addPointCloud(prev_cloud_trav_stepAnalysis, "cloud trav step");
//  }
//  image_viewer_proc_trav_stepAnalysis->addRGBImage<PointT>(prev_cloud_trav_stepAnalysis, "cloud trav step");
//
//  if (!viewer_proc_trav_slopeAnalysis->updatePointCloud(prev_cloud_trav_slopeAnalysis, "cloud trav slope"))
//  {
//    viewer_proc_trav_slopeAnalysis->addPointCloud(prev_cloud_trav_slopeAnalysis, "cloud trav slope");
//  }
//  image_viewer_proc_trav_slopeAnalysis->addRGBImage<PointT>(prev_cloud_trav_slopeAnalysis, "cloud trav slope");

  viewer->spinOnce(1);
  image_viewer->spinOnce(1);

////  viewer_disparity->spinOnce(1);
////  image_viewer_disparity->spinOnce(1);
////
////  viewer_disparity_processed->spinOnce(1);
////  image_viewer_disparity_processed->spinOnce(1);
////
  viewer_proc_segm->spinOnce(1);
  image_viewer_proc_segm->spinOnce(1);

//  viewer_proc_trav->spinOnce(1);
//  image_viewer_proc_trav->spinOnce(1);
//
//  viewer_proc_trav_stepAnalysis->spinOnce(1);
//  image_viewer_proc_trav_stepAnalysis->spinOnce(1);
//
//  viewer_proc_trav_slopeAnalysis->spinOnce(1);
//  image_viewer_proc_trav_slopeAnalysis->spinOnce(1);

}// run





