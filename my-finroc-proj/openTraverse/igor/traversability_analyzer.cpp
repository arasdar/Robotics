#include "projects/stereo_traversability_experiments/openTraverse/igor/traversability_analyzer.h"


namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

TraversabilityAnalyzer::TraversabilityAnalyzer(int _maxAngle,
    double cellWidth,
    double maxTravHeight,
    double radiusNormals,
    double robotHeight)
{
  this->_maxAngle = _maxAngle;
  this->_cellWidth = cellWidth;
  this->_maxTravHeight = maxTravHeight;
  this->_robotHeight = robotHeight;
  this->_radiusNormals = radiusNormals;
  this->_globalMap = Grid2D(cellWidth);
  this->_imageNormals = NULL;
  this->_imagePoints = NULL;
}

//// analyzes the traversability of current point cloud based on normals and step height
//// this is the version designed for g2o types, version for pcl is below
//// ---> fills the traversability indicator vector - an int for each point in the cloud
//// adds current point cloud information to the local 2d map
//void TraversabilityAnalyzer::analyzeAndAddPointCloud(std::vector<int>& travIndicators)
//{
//    travIndicators.resize(_imagePoints->size());
//    Grid3D localSparse3DMap=Grid3D(_cellWidth, _maxTravHeight, _radiusNormals, _maxAngle, _robotHeight);
//    Grid2D localMap2D = Grid2D(_cellWidth);
//    localSparse3DMap.addPointCloudG2O(*_imagePoints, *_imageNormals);
//    localSparse3DMap.checkTraversability(localMap2D);
//
//    //this call should write the traversability information to the vector
//    localMap2D.getTraversabilityVector(travIndicators);
//    _globalMap.add(localMap2D);
//}

// does the same as the version for g2o, but uses pcl input clouds
void TraversabilityAnalyzer::analyzeAndAddPointCloud(std::vector<int>& travIndicators,
    CloudPtr imagePoints,
    const pcl::PointCloud<pcl::Normal>::Ptr imageNormals)
{
  travIndicators.resize(imagePoints->points.size());
  Grid3D localSparse3DMap = Grid3D(_cellWidth, _maxTravHeight, _radiusNormals, _maxAngle, _robotHeight);
  Grid2D localMap2D = Grid2D(_cellWidth);
  localSparse3DMap.addPointCloudPCL(imagePoints, imageNormals);
  localSparse3DMap.checkTraversability(localMap2D);

  //this call should write the traversability information to the vector
//  localMap2D.getTraversabilityVector(travIndicators);
  localMap2D.getTraversabilityCloud(travIndicators, imagePoints);
  _globalMap.add(localMap2D);

}

void TraversabilityAnalyzer::printTraversabilityMap(const string& path)
{
  cv::Mat map_image;
  _globalMap.showMap(map_image);
  cv::cvtColor(map_image, map_image, CV_RGB2BGR);
  cv::imwrite(path, map_image);
}

void TraversabilityAnalyzer::printTraversabilityMap()
{
  cv::Mat map_image;
  _globalMap.showMap(map_image);
  cv::cvtColor(map_image, map_image, CV_RGB2BGR);
  cv::namedWindow("traversability map", 0);
  cv::imshow("traversability map", map_image);
  cv::waitKey(1);
}


//bool TraversabilityAnalyzer::instantiate(HomogeneousPoint3fVector* imagePoints,
//                                          HomogeneousNormal3fVector* imageNormals)
//{
//  return setNormals(imageNormals) && setPoints(imagePoints);
//}
//
//
//bool TraversabilityAnalyzer::setNormals(HomogeneousNormal3fVector* imageNormals)
//{
//  if (!imageNormals)
//  {
//    Logger::instance()->logError("TraversabilityAnalyzer", Logger::NO_NORMALS);
//    return false;
//  }
//  _imageNormals = imageNormals;
//  return true;
//}
//
//bool TraversabilityAnalyzer::setPoints(HomogeneousPoint3fVector* imagePoints)
//{
//  if (!imagePoints)
//  {
//    Logger::instance()->logError("TraversabilityAnalyzer", Logger::NO_POINTS);
//    return false;
//  }
//  _imagePoints=imagePoints;
//  return true;
//}
//
//void TraversabilityAnalyzer::clearCurrentPointCloud()
//{
//  _imagePoints = NULL;
//  _imageNormals = NULL;
//}


}
}
}
}
