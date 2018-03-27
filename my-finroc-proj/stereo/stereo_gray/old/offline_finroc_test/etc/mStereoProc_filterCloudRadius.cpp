
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/conditional_removal.h>

#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"


using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;

//typedef pcl::PointXYZRGB PointType;
//
//float default_radius = 0.1f;
//bool default_inside = true;
//bool default_keep_organized = true;
//
//
//void
//compute(const Cloud::Ptr &input, Cloud::Ptr &output,
//        float radius, bool inside, bool keep_organized)
//{
//  // Estimate
//  pcl::console::TicToc tt;
//  tt.tic();
//
//  pcl::console::print_highlight(stderr, "Computing ");
//
//  pcl::ConditionOr<PointType>::Ptr cond(new pcl::ConditionOr<PointType> ());
//  cond->addComparison(pcl::TfQuadraticXYZComparison<PointType>::ConstPtr(new pcl::TfQuadraticXYZComparison<PointType> (inside ? pcl::ComparisonOps::LT : pcl::ComparisonOps::GT, Eigen::Matrix3f::Identity(),
//                      Eigen::Vector3f::Zero(), - radius * radius)));
//
//  pcl::ConditionalRemoval<PointType> condrem;
//  condrem.setCondition(cond);
//  condrem.setInputCloud(input);
//  condrem.setKeepOrganized(keep_organized);
//  condrem.filter(*output);
//
//  pcl::console::print_info("[done, ");
//  pcl::console::print_value("%g", tt.toc());
//  pcl::console::print_info(" ms : ");
//  pcl::console::print_value("%d", output->size());
//  pcl::console::print_info(" points]\n");
//}
//
//void mStereoGrayOffline::filterCloudRadius()
//{
//
//  float radius = default_radius;
//  bool inside = default_inside;
//  bool keep_organized = default_keep_organized;
//
//  CloudPtr cloud(new Cloud);
//  cloud = prev_ground_image;
//
//  // Perform the feature estimation
//  CloudPtr output(new Cloud);
//  output = prev_ground_image;
//  compute(cloud, output, radius, inside, keep_organized);
//}



#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

void mStereoGrayOffline::filterCloudRadius()
{
  CloudPtr cloud(new Cloud);
  cloud = prev_ground_image;
  CloudPtr cloud_filtered(new Cloud);

//  {
//    pcl::RadiusOutlierRemoval<PointT> outrem;
//    // build the filter
//    outrem.setInputCloud(cloud);
//    outrem.setRadiusSearch(2);
//    outrem.setMinNeighborsInRadius (200);
//    // apply filter
//    outrem.filter (*cloud_filtered);
//  }

  {
    // build the condition
    pcl::ConditionAnd<PointT>::Ptr range_cond(new
        pcl::ConditionAnd<PointT> ());

    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
                              pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.0)));

    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new
                              pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, 0.8)));

    // build the filter
    pcl::ConditionalRemoval<PointT> condrem(range_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);

    // apply filter
    condrem.filter(*cloud_filtered);
  }

}
