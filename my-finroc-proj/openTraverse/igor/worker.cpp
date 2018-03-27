#include "projects/stereo_traversability_experiments/openTraverse/igor/worker.h"

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

using namespace std;

Worker::Worker()
{}

void adHocReadPclCloud(CloudPtr points)
{
//  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("points.pcd", *points) == -1) //* load the file
  if (pcl::io::loadPCDFile<PointT>("/home/aras/mine/openTraverse/igor/data/points.pcd", *points) == -1)
//  if(pcl::io::loadPCDFile<pcl::PointXYZ>("../points.pcd", *points) == -1)
  {
    PCL_ERROR("Couldn't read file points.pcd \n");
    return;
  }
}

void adHocComputeNormalsPCL(CloudPtr points, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(points);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod(tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*normals);

  cout << "size " << normals->points.size() << endl;
  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}


void Worker::processPCLToMap(int maxAngle,
                             double cellWidth,
                             double maxTravHeight,
                             double radiusNormals,
                             double robotHeight,
                             string outputImage)
{
  std::vector<int> traversabilityIndicators;
  TraversabilityAnalyzer traversabilityAnalyzer(maxAngle,
      cellWidth,
      maxTravHeight,
      radiusNormals,
      robotHeight);

  // temporary holders for points and normals
  CloudPtr points(new Cloud);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

//  // We read here the same pcd file 10 times
//  // If we would read it once, the map would be too uncertain.
//  for (int i = 0; i < 1; ++i)
//  {
//    //fill points and normals
//    adHocReadPclCloud(points);
//    adHocComputeNormalsPCL(points, normals);
//
//    // Provided the point cloud and normals
//    // we can analyze the traversability.
//    // This function analyzes current point cloud and
//    // --- returns the traversabilityIndicators vector.
//    // --- updates the global 2d map.
//    //
//    // traversability indicators will be same size as point cloud
//    // each element represents if corresponding point is traversable
//    //  "-1" = not traversable
//    //  "1" = traversable
//    traversabilityAnalyzer.analyzeAndAddPointCloud(traversabilityIndicators, points, normals);
//  }

  //fill points and normals
  adHocReadPclCloud(points);
  adHocComputeNormalsPCL(points, normals);

  // Provided the point cloud and normals
  traversabilityAnalyzer.analyzeAndAddPointCloud(traversabilityIndicators, points, normals);

//  traversabilityAnalyzer.printTraversabilityMap(outputImage);
  traversabilityAnalyzer.printTraversabilityMap();
}


void Worker::processCloudToMap(int maxAngle,
                               double cellWidth,
                               double maxTravHeight,
                               double radiusNormals,
                               double robotHeight,
                               CloudPtr& cloud)
{
  std::vector<int> traversabilityIndicators;
  TraversabilityAnalyzer traversabilityAnalyzer(maxAngle,
      cellWidth,
      maxTravHeight,
      radiusNormals,
      robotHeight);

  // temporary holders for points and normals
  CloudPtr points(new Cloud);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  //fill points and normals
//    adHocReadPclCloud(points);
  points = cloud;
  adHocComputeNormalsPCL(points, normals);

  // Provided the point cloud and normals
  traversabilityAnalyzer.analyzeAndAddPointCloud(traversabilityIndicators, points, normals);
  traversabilityAnalyzer.printTraversabilityMap();

}

}
}
}
}
