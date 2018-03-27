#ifndef TRAVERSABILITY_ANALYZER_H
#define TRAVERSABILITY_ANALYZER_H

#include "projects/stereo_traversability_experiments/openTraverse/igor/grid3d.h"
#include <pcl/io/pcd_io.h>

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> Cloud;
//typedef Cloud::ConstPtr CloudConstPt;
//typedef Cloud::Ptr CloudPtr;

class TraversabilityAnalyzer
{
private:
  int _maxAngle;
  double _cellWidth;
  double _maxTravHeight;
  double _robotHeight;
  double _radiusNormals;

  Grid2D _globalMap;

  HomogeneousNormal3fVector* _imageNormals;
  HomogeneousPoint3fVector* _imagePoints;

//  bool setNormals(HomogeneousNormal3fVector* imageNormals);
//  bool setPoints(HomogeneousPoint3fVector* imagePoints);


public:
  TraversabilityAnalyzer(int _maxAngle,
                         double cellWidth,
                         double maxTravHeight,
                         double radiusNormals,
                         double robotHeight);
//  bool instantiate(HomogeneousPoint3fVector* imagePoints,
//                    HomogeneousNormal3fVector* imageNormals);
//  void analyzeAndAddPointCloud(std::vector<int>& travIndicators);
  void analyzeAndAddPointCloud(std::vector<int>& travIndicators,
                               const CloudPtr imagePoints,
                               const pcl::PointCloud<pcl::Normal>::Ptr imageNormals);
  void printTraversabilityMap(const string& path);
  void printTraversabilityMap();
//  void clearCurrentPointCloud();
};

}
}
}
}

#endif
