


//#include "UPD.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/registration/transforms.h>


namespace finroc
{
namespace mauro
{
namespace test
{

/************************************************************************************************
 ******************************************* VIEWER PROGRAM *************************************
 ************************************************************************************************/


void
UPD::viewerOneOff(visualization::PCLVisualizer& viewer)
{
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 1.0, 0.5, v1);
  viewer.setBackgroundColor(1.0, 1.0, 1.0, v1);
  visualization::PointCloudColorHandlerRGBField<PointT> rgb1(image_cloud);
  viewer.addPointCloud<PointT> (image_cloud, rgb1, "cloud", v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  int v2(0);
  viewer.createViewPort(0.0, 0.5, 1.0, 1.0, v2);
  viewer.setBackgroundColor(1.0, 1.0, 1.0, v2);
  visualization::PointCloudColorHandlerRGBField<PointT> rgb2(colored_cloud);
  viewer.addPointCloud<PointT> (colored_cloud, rgb2, "C-cloud", v2);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "C-cloud");
  cout << "i only run once" << endl;
}

/************************************************************************************************
 ******************************************* main *************************************
 ************************************************************************************************/

void
UPD::viewerPsycho(visualization::PCLVisualizer& viewer)
{
  static unsigned count = 0;
  stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);

  //FIXME: possible race condition here:
  user_data++;
}

}
}
}
