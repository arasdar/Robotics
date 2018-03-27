

#include "projects/stereo_traversability_experiments/daniel/stereo_color_original/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_original::offline;

void
tStereoProcessing::processCloud_normals(const CloudConstPtr& cloud)
{

  /*! Set up process cloud  -- normal extraction*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
  ne.setDepthDependentSmoothing(false);

  /*Compute the normals*/
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(cloud); //input
  ne.compute(*normal_cloud); //output
  prev_normal_cloud = normal_cloud;
}
