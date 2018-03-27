#include "projects/stereo_traversability_experiments/openTraverse/igor/grid3d.h"
//#include <cassert>

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{
Grid3D::Grid3D(double cellWidth,
               double maxTravHeight,
               double normalsRadius,
               double maxAngle,
               double robotHeight)
{
  xMin_ = BIG_NUM;
  xMax_ = -BIG_NUM;
  yMin_ = BIG_NUM;
  yMax_ = -BIG_NUM;
  cellWidth_ = cellWidth;
  maxTravHeight_ = maxTravHeight;
  normalsRadius_ = normalsRadius;
  maxAngle_ = maxAngle;
  robotHeight_ = robotHeight;
}

int* Grid3D::getCellCoords(const float x, const float y, const float z)
{
  int* coor = new int[3];
  coor[0] = 0;
  coor[1] = 0;
  coor[2] = 0;

  coor[0] = (int)ceil(x / cellWidth_);
  coor[1] = (int)ceil(y / cellWidth_);
  coor[2] = (int)ceil(z / cellWidth_);

  return coor;
}

bool Grid3D::hasCell(const int* coor)
{
  int x = coor[0];
  int y = coor[1];
  int z = coor[2];
  HashMap3D::iterator iteratorX;
  HashMap2D::iterator iteratorY;
  Map1D::iterator iteratorZ;

  iteratorX = grid3d_.find(x);
  if (iteratorX == grid3d_.end())
  {
    //Cell was not found
    return false;
  }
  else
  {
    //if found => check the y value
    iteratorY = iteratorX->second.find(y);
    if (iteratorY == iteratorX->second.end())
    {
      //Cell was not found
      return false;
    }
    else
    {
      iteratorZ = iteratorY->second.find(z);
      if (iteratorZ == iteratorY->second.end())
      {
        //Cell was not found
        return false;
      }
    }
  }
  // cell found!
  return true;
}

void Grid3D::updateXYBounds(int x, int y)
{

//  cout << "cell updates bounds " << x << " " << y << endl;
  if (x < xMin_)
  {
    xMin_ = x;
  }
  if (x > xMax_) //&& x < 500 * cellWidth_
  {
    xMax_ = x;
  }

  if (y < yMin_)  // depth
  {
    yMin_ = y;
  }

  if (y > yMax_)  //depth
  {
    yMax_ = y;
  }
//  cout << "new bounds \n" << xMax_ << " " << yMax_ << " " << xMin_ << " " << yMin_ << endl;

}

void Grid3D::addNewCell(const int* coor, const bool isTraversable, double depth, int index)
{
  int x = coor[0];
  int y = coor[1];
  int z = coor[2];
  grid3d_[x][y][z] = Utils::createVoxel(isTraversable, depth, index);
  updateXYBounds(x, y);
}

void Grid3D::updateCell(const int* coor, const bool isTraversable, double depth, int index)
{
  int x = coor[0];
  int y = coor[1];
  int z = coor[2];
  Voxel traverce = grid3d_[x][y][z];
  Voxel color = Utils::createVoxel(isTraversable, depth);
  traverce.red += color.red;
  traverce.green += color.green;
  // update the depth. This means that the voxel was observed from a closer distance
  // and therefore is more reliable
  if (traverce.depth < color.depth)
  {
    traverce.depth = color.depth;
  }
  traverce.indexesInPointCloud.push_back(index); //a new point adds a new index to current voxel
  grid3d_[x][y][z] = traverce;
}

void Grid3D::checkTraversability(Grid2D& grid2d)
{
  cv::Mat projection;
  cv::Mat blobs;
  int xOffset = 0;
  int yOffset = 0;
  this->checkSteps(grid2d);
  this->project(projection, xOffset, yOffset);

  int neighborhoodSize = ceil(normalsRadius_ / cellWidth_);
  int numOfBlobs = Utils::blobDetection(projection, blobs, neighborhoodSize);
  this->checkBlobs(numOfBlobs, blobs, xOffset, yOffset, grid2d, neighborhoodSize);
}

void Grid3D::checkSteps(Grid2D& grid2d)
{
  HashMap3D::iterator iteratorX;
  HashMap2D::iterator iteratorY;
  int keyX, keyY;
  vector<map<int, Voxel>> binsVect;
  vector<int> indexesInPointCloud;
  for (iteratorX = grid3d_.begin(); iteratorX != grid3d_.end(); iteratorX++)
  {
    keyX = iteratorX->first;
    for (iteratorY = iteratorX->second.begin(); iteratorY != iteratorX->second.end(); iteratorY++)
    {
      indexesInPointCloud.clear();
      keyY = iteratorY->first;
      binsVect.clear();
      binsVect.push_back(iteratorY->second);
      //here indexes indexesInPointCloud receives all the points'
      //indexes from a vertical bin of voxels
      this->getIndexesInPointCloud(binsVect[0], indexesInPointCloud);
      if (isStepTraversable(binsVect))
      {
        grid2d.setVoxel(keyX, keyY, GREEN, indexesInPointCloud);
      }
      else
      {
        grid2d.setVoxel(keyX, keyY, RED, indexesInPointCloud);
      }
    }
  }
}

void Grid3D::getIndexesInPointCloud(const map<int, Voxel>& voxels, vector<int>& indeces)
{
  map<int, Voxel>::const_iterator iter;
  for (iter = voxels.begin(); iter != voxels.end(); ++iter)
  {
    Utils::concatenateVectors(indeces, iter->second.indexesInPointCloud);
  }
}

void Grid3D::checkBlobs(const int numOfBlobs, const cv::Mat& blobs,
                        int xOffset, int yOffset, Grid2D& grid2d, int neighborhoodSize)
{
  for (int i = 0; i < numOfBlobs; ++i)
  {
    int maxHeight = -BIG_NUM;
    int minHeight = BIG_NUM;
    for (int x = 0; x < blobs.cols; ++x)
    {
      for (int y = 0; y < blobs.rows; ++y)
      {
        if (blobs.at<int>(y, x) == (i + 1))
        {
          this->getMinMaxKey(x + xOffset, y + yOffset, minHeight, maxHeight);
        }
      }
    }
    if ((maxHeight - minHeight)*cellWidth_ > maxTravHeight_)
    {
      for (int x = 0; x < blobs.cols; ++x)
      {
        for (int y = 0; y < blobs.rows; ++y)
        {
          if (blobs.at<int>(y, x) == (i + 1))
          {
            for (int xx = x - neighborhoodSize / 2; xx < x + neighborhoodSize / 2; ++xx)
            {
              for (int yy = y - neighborhoodSize / 2; yy < y + neighborhoodSize / 2; ++yy)
              {
                grid2d.setVoxelColor(xx + xOffset, yy + yOffset, RED);
              }
            }
          }
        }
      }
    }
  }
}

void Grid3D::getMinMaxKey(int x, int y, int& minKey, int& maxKey)
{
  Map1D tempCell = grid3d_[x][y];
  Map1D::iterator iteratorZ;
  for (iteratorZ = tempCell.begin(); iteratorZ != tempCell.end(); ++iteratorZ)
  {
    int zKey = iteratorZ->first;
    if (minKey > zKey)
    {
      minKey = zKey;
    }
    if (maxKey < zKey)
    {
      maxKey = zKey;
    }
  }
}

//This will project our structure to the b/w image
//with white spots at non traversable parts
void Grid3D::project(cv::Mat& img, int& xOffset, int& yOffset)
{
  int xSize = xMax_ - xMin_; //x size in px
  int ySize = yMax_ - yMin_; //y size in px

//  cout << "xSize = " << xSize << " ySize = " << ySize << endl;

  //these are needed as the position of the cell can be <0,
  //while the coordinates on the image can not
  xOffset = xMin_;
  yOffset = yMin_;

  img = cv::Mat::zeros(ySize, xSize, cv::DataType<int>::type);

  HashMap3D::iterator iteratorX;
  HashMap2D::iterator iteratorY;
  int keyX, keyY;
  for (iteratorX = grid3d_.begin(); iteratorX != grid3d_.end(); iteratorX++)
  {
    keyX = iteratorX->first;
    for (iteratorY = iteratorX->second.begin(); iteratorY != iteratorX->second.end(); iteratorY++)
    {
      keyY = iteratorY->first;
      bool traversable = this->checkNormals(iteratorY->second);
      if (traversable)
      {
        if (keyY - yOffset >= 0 && keyX - xOffset >= 0 && keyY - yOffset < img.rows && keyX - xOffset < img.cols)
          img.at<int>(keyY - yOffset, keyX - xOffset) = 0;
      }
      else
      {
        if (keyY - yOffset >= 0 && keyX - xOffset >= 0 && keyY - yOffset < img.rows && keyX - xOffset < img.cols)
          img.at<int>(keyY - yOffset, keyX - xOffset) = 255;
      }
    }
  }

}

bool Grid3D::checkNormals(const map<int, Voxel>& zBins)
{
  Map1D::const_iterator iteratorZ;
  Voxel tempColor;
  tempColor.red = 0;
  tempColor.green = 0;
  tempColor.depth = 0;
  int nonTraversableCounter = 0;
  int valid = 0;
  int nonValid = 0;
  float red = 0;
  float green = 0;
  float nonTraversableRatio = 0.6; //TODO: get rid of this magic constant
  for (iteratorZ = zBins.begin(); iteratorZ != zBins.end(); ++iteratorZ)
  {
    if (iteratorZ->first < robotHeight_)
    {
      valid++;
      tempColor = iteratorZ->second;
      green = tempColor.green;
      red = tempColor.red;
      if (red / (green + red) > nonTraversableRatio)
      {
        nonTraversableCounter++;
      }
    }
    else
    {
      nonValid++;
    }
  }
  return (nonTraversableCounter < 0.5 * (valid + nonValid));
  // return ((nonTraversableCounter < (float)nonTraversableRatio*zBins.size()));
}

void Grid3D::addPoint(float x, float y, float z, const bool isTraversable, float depth, int index)
{
  int* coor = this->getCellCoords(x, y, z);

//  cout << "coor x: "<< *coor << endl;

  if (this->hasCell(coor))
  {
    this->updateCell(coor, isTraversable, depth, index);
  }
  else
  {
    this->addNewCell(coor, isTraversable, depth, index);
  }
  delete[] coor;
}

//void Grid3D::addPointCloudG2O(const HomogeneousPoint3fVector& imagePoints,
//                            const HomogeneousNormal3fVector& imageNormals)
//{
//    bool boolNormalTraversable;
//    int size = -1;
//    if (imagePoints.size()==imageNormals.size())
//    {
//        size = imagePoints.size();
//    }
//    for(int i=0 ; i < size; i++)
//    {
//        if (imageNormals[i](0)!=0 || imageNormals[i](1)!=0 || imageNormals[i](2)!=0)
//        {
//            boolNormalTraversable = isNormalTraversable(imageNormals[i]);
//            this->addPoint(
//                imagePoints[i](0),
//                imagePoints[i](1),
//                imagePoints[i](2),
//                boolNormalTraversable,
//                0, //depth currently set to 0
//                i); //index in the point cloud
//        }
//    }
//}

void Grid3D::addPointCloudPCL(const CloudPtr imagePoints,
                              const pcl::PointCloud<pcl::Normal>::Ptr imageNormals)
{
  bool boolNormalTraversable;
  int size = -1;
  assert(imagePoints->points.size() == imageNormals->points.size());
  size = imagePoints->points.size();
  for (int i = 0 ; i < size; i++)
  {
    boolNormalTraversable = isNormalTraversable(imageNormals->points[i]);

    if (imagePoints->points[i].x > -5 / cellWidth_ &&
        imagePoints->points[i].x < 5 / cellWidth_)
    {
      if (imagePoints->points[i].z > 0 &&
          imagePoints->points[i].z < 10 / cellWidth_) // depth
      {
        if (imagePoints->points[i].y > -1 / cellWidth_ &&
            imagePoints->points[i].y < 1 / cellWidth_) //height
        {
          this->addPoint(
            imagePoints->points[i].x, //width = image coordinate system
            imagePoints->points[i].z, //depth
            imagePoints->points[i].y, //height = image coordinate system
            boolNormalTraversable,
            0, //depth currently set to 0
            i); //index in the point cloud

        }
      }
    }
  }
}

//detect if traversable and return color red or green
bool Grid3D::isNormalTraversable(const Eigen::Vector4f& normal)
{
  //Max.angle between the normal and the gravitation vector
  double max_phi = maxAngle_;
  //to radians
  max_phi = max_phi * M_PI * 0.00555;
  //find the angle from the data
  double cosAlpha = 0;
  cosAlpha = normal(2) / normal.norm();
  if (cosAlpha < 0)
  {
    return false;
  }
  double phi = acos(cosAlpha);

  if (phi > max_phi)
  {
    return false;
  }
  else
  {
    return true;
  }
}

//detect if traversable and return color red or green
bool Grid3D::isNormalTraversable(pcl::Normal normal)
{
  //Max.angle between the normal and the gravitation vector
  double max_phi = maxAngle_;
  //to radians
  max_phi = max_phi * M_PI * 0.00555;
  //find the angle from the data
  double cosAlpha = 0;
  cosAlpha = normal.normal[2] / Utils::norm(normal.normal);
  if (cosAlpha < 0)
  {
    return false;
  }
  double phi = acos(cosAlpha);

  if (phi > max_phi)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool Grid3D::isStepTraversable(const vector<map<int, Voxel>>& vectBins)
{
  map<int, Voxel>::const_iterator zIter;
  int gapKeyStart, gapKeyEnd;
  int stairKeyStart, stairKeyEnd;
  for (uint i = 0; i < vectBins.size(); ++i)
  {
    gapKeyStart = vectBins[i].begin()->first;
    stairKeyStart = vectBins[i].begin()->first;
    for (zIter = vectBins[i].begin(); zIter != vectBins[i].end(); ++zIter)
    {
      gapKeyEnd = zIter->first;
      stairKeyEnd = zIter->first;
      if (!stepIsLow(stairKeyStart, stairKeyEnd))
      {
        if (robotFitsUnder(gapKeyStart, gapKeyEnd))
        {
          return true;
        }
        else
        {
          return false;
        }
      }
      gapKeyStart = gapKeyEnd;
    }
  }
  return true;
}

bool Grid3D::robotFitsUnder(int keyStart, int keyEnd)
{
  return (keyEnd - keyStart) * cellWidth_ > robotHeight_;
}

bool Grid3D::stepIsLow(int keyStart, int keyEnd)
{
  return (keyEnd - keyStart) * cellWidth_ < maxTravHeight_;
}

}
}
}
}
