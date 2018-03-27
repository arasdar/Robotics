#ifndef GRID2D_H
#define GRID2D_H

#include "projects/stereo_traversability_experiments/openTraverse/igor/utils.h"

#include <pcl/features/normal_3d.h>

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
//typedef Cloud::ConstPtr CloudConstPt;
typedef Cloud::Ptr CloudPtr;

class Grid2D
{
private:
  //basic structure - keys of the maps represent the positions of cells in the grid2d
  HashMap2D grid2d_;
  int xMin_;
  int xMax_;
  int yMin_;
  int yMax_;

  /* parameters */
  double cellWidth_;

  //find the minimum and maximum possible positions in the grid2d
  // void findRange(int& xMin, int& xMax, int& yMin, int& yMax);
  bool hasCell(const int x, const int y);
  void addNewCell(const int x, const int y, const Voxel& voxel);
  void updateCell(const int x, const int y, const Voxel& voxel);
  void updateXYBounds(int x, int y);
  int check2DTraversability(Voxel voxel);

public:
  Grid2D();
  Grid2D(double cellWidth);
  // Write the image to disc
  void showMap(cv::Mat& map_image);
  void add(const Grid2D& grid2d);
  void setVoxel(const int x, const int y, int color, const std::vector<int>& indexesInPointCloud);
  void setVoxelColor(const int x, const int y, int color);
  HashMap2D getMap() const;
  void getTraversabilityVector(vector<int>& indeces);
  void getTraversabilityCloud(vector<int>& indices, CloudPtr& cloud);
};

}
}
}
}

#endif
