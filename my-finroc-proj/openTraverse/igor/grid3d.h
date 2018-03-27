#ifndef GRID3D_H
#define GRID3D_H

#include "projects/stereo_traversability_experiments/openTraverse/igor/grid2d.h"
#include "projects/stereo_traversability_experiments/openTraverse/igor/homogeneousvector4f.h"
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

class Grid3D
{
private:
  HashMap3D grid3d_;
  int xMin_;
  int xMax_;
  int yMin_;
  int yMax_;

  /* map parameters*/
  double cellWidth_;
  double maxTravHeight_;
  double normalsRadius_;
  double maxAngle_;
  double robotHeight_;

  // void findRange(int& xMin, int& xMax, int& yMin, int& yMax);
  int* getCellCoords(const float point_x, const float point_y, const float point_z);
  bool hasCell(const int* coor);
  void addNewCell(const int* coor, const bool isTraversable, double depth, int index);
  void updateCell(const int* coor, const bool isTraversable, double depth, int index);
  void checkSteps(Grid2D& grid2d);
  void checkBlobs(const int numOfBlobs, const cv::Mat& blobs, int xOffset, int yOffset, Grid2D& grid2d, int neighborhoodSize);
  void getMinMaxKey(int x, int y, int& minKey, int& maxKey);
  bool checkNormals(const map<int, Voxel>& zBins);
  void updateXYBounds(int x, int y);
  void addPoint(float x, float y, float z, const bool isTraversable, float depth, int index);
  bool isStepTraversable(const vector<map<int, Voxel>>& vectBins);
  void getIndexesInPointCloud(const map<int, Voxel>& voxels, vector<int>& indeces);
  bool isNormalTraversable(const Eigen::Vector4f& normal);
  bool isNormalTraversable(pcl::Normal normal);
  bool robotFitsUnder(int keyStart, int keyEnd);
  bool stepIsLow(int keyStart, int keyEnd);
public:
  Grid3D(double cellWidth,
         double maxTravHeight,
         double normalsRadius,
         double maxAngle,
         double robotHeight);
//    void addPointCloudG2O(const HomogeneousPoint3fVector& imagePoints,
//                        const HomogeneousNormal3fVector& imageNormals);
  void addPointCloudPCL(const CloudPtr imagePoints,
                        const pcl::PointCloud<pcl::Normal>::Ptr imageNormals);
  void checkTraversability(Grid2D& grid2d);
  void project(cv::Mat& img, int& xOffset, int& yOffset);
};

}
}
}
}

#endif
