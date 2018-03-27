
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;

typedef PointXYZRGB PointType;

//std::string default_method = "radius";
const std::string default_method = "statistical";

const int default_mean_k = 50; //2;
const double default_std_dev_mul = 1.0; //0.0;
const int default_negative = 0;
const double default_radius = 0.0;
const int default_min_pts = 0;

void
compute(const CloudConstPtr& input, CloudPtr& output,
        std::string method,
        int min_pts, double radius,
        int mean_k, double std_dev_mul, bool negative, bool keep_organized)
{

//  PointCloud<PointXYZ>::Ptr xyz_cloud_pre(new pcl::PointCloud<PointXYZ> ()),
//             xyz_cloud(new pcl::PointCloud<PointXYZ> ());
//  fromPCLPointCloud2(*input, *xyz_cloud_pre);
  CloudPtr xyz_cloud_pre(new Cloud), xyz_cloud(new Cloud);
  *xyz_cloud_pre = *input;

  pcl::PointIndices::Ptr removed_indices(new PointIndices),
      indices(new PointIndices);
  std::vector<int> valid_indices;
  if (keep_organized)
  {
    xyz_cloud = xyz_cloud_pre;
    for (int i = 0; i < int (xyz_cloud->size()); ++i)
      valid_indices.push_back(i);
  }
  else
    removeNaNFromPointCloud<PointType> (*xyz_cloud_pre, *xyz_cloud, valid_indices);

  TicToc tt;
  tt.tic();
  //PointCloud<PointXYZ>::Ptr xyz_cloud_filtered(new PointCloud<PointXYZ> ());
  CloudPtr xyz_cloud_filtered(new Cloud);
  if (method == "statistical")
  {
    StatisticalOutlierRemoval<PointType> filter(true);
    filter.setInputCloud(xyz_cloud);
    filter.setMeanK(mean_k);
    filter.setStddevMulThresh(std_dev_mul);
    filter.setNegative(negative);
    filter.setKeepOrganized(keep_organized);
    PCL_INFO("Computing filtered cloud from %lu points with mean_k %d, std_dev_mul %f, inliers %d ...", xyz_cloud->size(), filter.getMeanK(), filter.getStddevMulThresh(), filter.getNegative());
    filter.filter(*xyz_cloud_filtered);
    // Get the indices that have been explicitly removed
    filter.getRemovedIndices(*removed_indices);
  }
  else if (method == "radius")
  {
    RadiusOutlierRemoval<PointType> filter(true);
    filter.setInputCloud(xyz_cloud);
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(min_pts);
    filter.setNegative(negative);
    filter.setKeepOrganized(keep_organized);
    PCL_INFO("Computing filtered cloud from %lu points with radius %f, min_pts %d ...", xyz_cloud->size(), radius, min_pts);
    filter.filter(*xyz_cloud_filtered);
    // Get the indices that have been explicitly removed
    filter.getRemovedIndices(*removed_indices);
  }
  else
  {
    PCL_ERROR("%s is not a valid filter name! Quitting!\n", method.c_str());
    return;
  }

  print_info("[done, ");
  print_value("%g", tt.toc());
  print_info(" ms : ");
  print_value("%d", xyz_cloud_filtered->width * xyz_cloud_filtered->height);
  print_info(" points, %lu indices removed]\n", removed_indices->indices.size());

//  if (keep_organized)
////  {
////    Cloud output_filtered;
////    //toPCLPointCloud2(*xyz_cloud_filtered, output_filtered);
////    output_filtered = *xyz_cloud_filtered;
////    //concatenateFields(*input, output_filtered, output);
////  }
////  else
//  {
//    // Make sure we are addressing values in the original index vector
//    for (size_t i = 0; i < removed_indices->indices.size(); ++i)
//      indices->indices.push_back(valid_indices[removed_indices->indices[i]]);
//
//    // Extract the indices of the remaining points
////    pcl::ExtractIndices<pcl::PCLPointCloud2> ei;
//    ExtractIndices<Cloud> ei;
//    ei.setInputCloud(input);
//    ei.setIndices(indices);
//    ei.setNegative(true);
//    ei.filter(output);
//  }
}

//void mStereoGrayOffline::filterCloudOutlierRemoval()
//{
//
//  // parsing the parameters
//  std::string method = default_method;
//  int min_pts = default_min_pts;
//  double radius = default_radius;
//  int mean_k = default_mean_k;
//  double std_dev_mul = default_std_dev_mul;
//  int negative = default_negative;
//  bool keep_organized = true; //find_switch (argc, argv, "-keep_organized");
//
//  compute(prev_ground_image, prev_ground_image, method, min_pts, radius, mean_k, std_dev_mul, negative, keep_organized);
//
//}



void mStereoGrayOffline::filterCloudOutlierRemoval()
{
  CloudPtr cloud(new Cloud);
  cloud = prev_ground_image;
  CloudPtr cloud_filtered(new Cloud);


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.5);
  sor.setKeepOrganized(true);

  sor.setNegative(true); //showing the filtered part or not
  sor.filter(*cloud_filtered);
  prev_ground_image = cloud_filtered;

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

}
