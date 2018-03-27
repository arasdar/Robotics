

#include "projects/icarus/sensor_processing/pointCloudProcessing/tStereoProcessing.h"

using namespace finroc::icarus::sensor_processing::pointCloudProcessing;

void
tStereoProcessing::processCloud_normals()
{

  /*! Set up process cloud  -- normal extraction*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
  ne.setDepthDependentSmoothing(false);

  /*Compute the normals*/
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(prev_cloud); //input
  ne.compute(*normal_cloud); //output
  prev_normal_cloud = normal_cloud;
}
