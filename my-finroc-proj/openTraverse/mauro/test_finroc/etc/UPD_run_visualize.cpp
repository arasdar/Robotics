
//#include "projects/icarus/sensor_processing/pointCloudProcessing/tStereoProcessing.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

//using namespace finroc::icarus::sensor_processing::pointCloudProcessing;
using namespace finroc::mauro::test;

void
UPD::run_visualize()
{

  //visualizing generated point cloud and extracted normals on it optionally
  if (!viewer_test->updatePointCloud(prev_cloud, "cloud"))
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_cloud);
    viewer_test->addPointCloud(prev_cloud, rgb, "cloud"); //works too
  }//if
//  image_viewer->addRGBImage<PointT>(prev_cloud, "rgb_image", 0.3);

  viewer_test->spinOnce(1);
//  image_viewer->spinOnce(1);

}// run





