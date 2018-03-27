#include "projects/stereo_traversability_experiments/openTraverse/igor/grid2d.h"


namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

Grid2D::Grid2D()
{
  xMin_ = BIG_NUM;
  xMax_ = -BIG_NUM;
  yMin_ = BIG_NUM;
  yMax_ = -BIG_NUM;
}

Grid2D::Grid2D(double cellWidth)
{
  xMin_ = BIG_NUM;
  xMax_ = -BIG_NUM;
  yMin_ = BIG_NUM;
  yMax_ = -BIG_NUM;
  cellWidth_ = cellWidth;
}


void Grid2D::updateXYBounds(int x, int y)
{
  if (x < xMin_)
  {
    xMin_ = x;
  }
  if (x > xMax_)
  {
    xMax_ = x;
  }

  if (y < yMin_)
  {
    yMin_ = y;
  }
  if (y > yMax_)
  {
    yMax_ = y;
  }
}

//This will print an image of 1px=1cm
void Grid2D::showMap(cv::Mat& map_image)
{
  // int xMin=-1;
  // int xMax=-1;
  // int yMin=-1;
  // int yMax=-1;
  // findRange(xMin, xMax, yMin, yMax);
  int xSize = (int)ceil((xMax_ - xMin_) * 100 * cellWidth_); //x size in px
  int ySize = (int)ceil((yMax_ - yMin_) * 100 * cellWidth_); //y size in px
  int cellWidthImage = (int)floor(100 * cellWidth_); //width of the cell in px

  //these are needed as the position of the cell can be <0, while the coordinates on the image can not
  int compensateX = xMin_ * cellWidthImage;
  int compensateY = yMin_ * cellWidthImage;

  //maximal y in the image to rotate the image along the y axes
  //(because opencv has y axes pointing downwards and pcl not)
  int yMaxImage = yMax_ * cellWidthImage + compensateY;

  map_image = cv::Mat::zeros(ySize, xSize, CV_8UC3);

  HashMap2D::iterator x_iter;
  Map1D::iterator y_iter;
  int keyX, keyY;
  for (x_iter = grid2d_.begin(); x_iter != grid2d_.end(); x_iter++)
  {
    keyX = x_iter->first;
    for (y_iter = x_iter->second.begin(); y_iter != x_iter->second.end(); y_iter++)
    {
      keyY = y_iter->first;
      int traversable = this->check2DTraversability(y_iter->second);
      if (traversable == 1)
        cv::rectangle(map_image,
                      cv::Point((keyX)*cellWidthImage - compensateX, yMaxImage - (keyY + 1)*cellWidthImage - compensateY),
                      cv::Point((keyX + 1)*cellWidthImage - compensateX, yMaxImage - (keyY)*cellWidthImage - compensateY),
                      cvScalar(0, 255, 0),
                      -1, 8);
      else if (traversable == -1)
        cv::rectangle(map_image,
                      cv::Point((keyX)*cellWidthImage - compensateX, yMaxImage - (keyY + 1)*cellWidthImage - compensateY),
                      cv::Point((keyX + 1)*cellWidthImage - compensateX, yMaxImage - (keyY)*cellWidthImage - compensateY),
                      cvScalar(255, 0, 0),
                      -1, 8);
      else if (traversable == 0)
        cv::rectangle(map_image,
                      cv::Point((keyX)*cellWidthImage - compensateX, yMaxImage - (keyY + 1)*cellWidthImage - compensateY),
                      cv::Point((keyX + 1)*cellWidthImage - compensateX, yMaxImage - (keyY)*cellWidthImage - compensateY),
                      cvScalar(255, 255, 0),
                      -1, 8);
    }
  }
}

void Grid2D::add(const Grid2D& gridAdd)
{
  Map1D::iterator y_iter;
  HashMap2D tempGridAdd = gridAdd.getMap();
  HashMap2D::iterator x_iter;
  int keyX, keyY;
  for (x_iter = tempGridAdd.begin(); x_iter != tempGridAdd.end(); x_iter++)
  {
    keyX = x_iter->first;
    for (y_iter = x_iter->second.begin(); y_iter != x_iter->second.end(); y_iter++)
    {
      keyY = y_iter->first;
      if (this->hasCell(keyX, keyY))
      {
        this->updateCell(keyX, keyY, y_iter->second);
      }
      else
      {
        this->addNewCell(keyX, keyY, y_iter->second);
      }
    }
  }
}

HashMap2D Grid2D::getMap() const
{
  return this->grid2d_;
}

bool Grid2D::hasCell(const int x, const int y)
{
  HashMap2D::iterator x_iter;
  Map1D::iterator y_iter;

  x_iter = grid2d_.find(x);
  if (x_iter == grid2d_.end())
  {
    return false;
  }
  else
  {
    y_iter = x_iter->second.find(y);
    if (y_iter == x_iter->second.end())
    {
      return false;
    }
  }
  return true;
}

void Grid2D::addNewCell(const int x, const int y, const Voxel& voxel)
{
  grid2d_[x][y] = voxel;
  updateXYBounds(x, y);
}

void Grid2D::updateCell(const int x, const int y, const Voxel& voxel)
{
  Voxel traverce = grid2d_[x][y];
  Voxel color = voxel;
  traverce.red += color.red;
  traverce.green += color.green;
  grid2d_[x][y] = traverce;
}

void Grid2D::setVoxel(const int x, const int y, int color, const std::vector<int>& indexesInPointCloud)
{
  Voxel tempVoxel;
  tempVoxel.red = 0;
  tempVoxel.green = 0;
  tempVoxel.depth = 0;
  tempVoxel.indexesInPointCloud = indexesInPointCloud;
  switch (color)
  {
  case RED:
    tempVoxel.red = 1;
    tempVoxel.green = 0;
    grid2d_[x][y] = tempVoxel;
    break;
  case GREEN:
    tempVoxel.red = 0;
    tempVoxel.green = 1;
    grid2d_[x][y] = tempVoxel;
    break;
  }
}

void Grid2D::setVoxelColor(const int x, const int y, int color)
{
  Voxel tempVoxel = grid2d_[x][y];
  tempVoxel.red = 0;
  tempVoxel.green = 0;
  tempVoxel.depth = 0;
  switch (color)
  {
  case RED:
    tempVoxel.red = 1;
    tempVoxel.green = 0;
    grid2d_[x][y] = tempVoxel;
    break;
  case GREEN:
    tempVoxel.red = 0;
    tempVoxel.green = 1;
    grid2d_[x][y] = tempVoxel;
    break;
  }
}

int Grid2D::check2DTraversability(Voxel voxel)
{
  if (voxel.red + voxel.green < 1)
  {
    return 0;
  }
  float factor = 0.9;
  if (voxel.red / (float)voxel.green > factor)
  {
    return -1;
  }
  else if (voxel.green / (float)voxel.red > factor)
  {
    return 1;
  }
  else
    return 0;
}


void Grid2D::getTraversabilityVector(vector<int>& indeces)
{
  HashMap2D::iterator x_iter;
  Map1D::iterator y_iter;
  Voxel tempVoxel;
  std::vector<int>::iterator indecesIter;

  for (x_iter = grid2d_.begin(); x_iter != grid2d_.end(); ++x_iter)
  {
    for (y_iter = x_iter->second.begin(); y_iter != x_iter->second.end(); ++y_iter)
    {
      tempVoxel = y_iter->second;
      for (indecesIter = tempVoxel.indexesInPointCloud.begin(); indecesIter != tempVoxel.indexesInPointCloud.end(); ++indecesIter)
      {
        if (tempVoxel.red > 0)
        {
          //set points non-trav
          indeces[*indecesIter] = -1;
        }
        else if (tempVoxel.green > 0)
        {
          indeces[*indecesIter] = 1;
        }
      }
    }
  }
}


void Grid2D::getTraversabilityCloud(vector<int>& indeces, CloudPtr &cloud)
{
  HashMap2D::iterator x_iter;
  Map1D::iterator y_iter;
  Voxel tempVoxel;
  std::vector<int>::iterator indecesIter;

  for (x_iter = grid2d_.begin(); x_iter != grid2d_.end(); ++x_iter)
  {
    for (y_iter = x_iter->second.begin(); y_iter != x_iter->second.end(); ++y_iter)
    {
      tempVoxel = y_iter->second;
      for (indecesIter = tempVoxel.indexesInPointCloud.begin(); indecesIter != tempVoxel.indexesInPointCloud.end(); ++indecesIter)
      {
        if (tempVoxel.red > 0)
        {
          //set points non-trav
          indeces[*indecesIter] = -1;
          cloud->points[*indecesIter].r = 255;
//          cloud->points[*indecesIter].r = tempVoxel.red;

        }
        else if (tempVoxel.green > 0)
        {
          indeces[*indecesIter] = 1;
          cloud->points[*indecesIter].g = 255;
//          cloud->points[*indecesIter].g = tempVoxel.green;

        }
      }
    }
  }
}


}
}
}
}
