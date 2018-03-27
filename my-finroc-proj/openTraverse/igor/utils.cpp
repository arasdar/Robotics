#include "projects/stereo_traversability_experiments/openTraverse/igor/utils.h"

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

using namespace cv;

Voxel Utils::createVoxel(const bool& isTraversable, double depth, int index)
{
  /* assuming rgb*/
  Voxel voxel;
  if (isTraversable)
  {
    voxel.red = 0;
    voxel.green = 1;
    voxel.depth = depth;
  }
  else
  {
    voxel.red = 1;
    voxel.green = 0;
    voxel.depth = depth;
  }
  if (index > -1)
  {
    voxel.indexesInPointCloud.push_back(index);
  }
  return voxel;
}

void Utils::LabelNeighbourhood(int x, int y, int l, const Mat& data, Mat& labels, int neighborhoodSize)
{
  if (y < 0 || x < 0 || y >= labels.rows || x >= labels.cols)
  {
    return;
  }

  if (labels.at<int>(y, x) == 0 && data.at<int>(y, x) > 0  && Utils::validPoint(data, x, y, neighborhoodSize)) // if it is red and not labeled
  {
    labels.at<int>(y, x) = l;

    Utils::LabelNeighbourhood(x - 1, y, l, data, labels, neighborhoodSize);
    Utils::LabelNeighbourhood(x, y + 1, l, data, labels, neighborhoodSize);
    Utils::LabelNeighbourhood(x + 1, y, l, data, labels, neighborhoodSize);
    Utils::LabelNeighbourhood(x, y - 1, l, data, labels, neighborhoodSize);

    Utils::LabelNeighbourhood(x - 1, y - 1, l, data, labels, neighborhoodSize);
    Utils::LabelNeighbourhood(x + 1, y + 1, l, data, labels, neighborhoodSize);
    Utils::LabelNeighbourhood(x - 1, y + 1, l, data, labels, neighborhoodSize);
    Utils::LabelNeighbourhood(x + 1, y - 1, l, data, labels, neighborhoodSize);
  }
  return;
}

bool Utils::validPoint(const Mat& input, int x, int y, int neighborhoodSize)
{
  if (x < neighborhoodSize
      || y < neighborhoodSize
      || x >= input.cols - neighborhoodSize
      || y >= input.rows - neighborhoodSize)
  {
    return false;
  }
  if (input.at<int>(y + neighborhoodSize, x) == 0
      || input.at<int>(y - neighborhoodSize, x) == 0
      || input.at<int>(y, x + neighborhoodSize) == 0
      || input.at<int>(y, x - neighborhoodSize) == 0)
  {
    return false;
  }
  return true;
}

int Utils::blobDetection(const Mat& input, Mat& labels, int neighborhoodSize)
{
  int l = 0;
  labels = cv::Mat::zeros(input.size(), DataType<int>::type);
  for (int x = 0; x < input.cols; x++)
  {
    for (int y = 0; y < input.rows; y++)
    {
      if (input.at<int>(y, x) > 0 && labels.at<int>(y, x) == 0 && Utils::validPoint(input, x, y, neighborhoodSize))
      {
        Utils::LabelNeighbourhood(x, y, ++l, input, labels, neighborhoodSize);
      }
    }
  }
  return l;

}

void Utils::concatenateVectors(std::vector<int>& AB, const vector<int>& B)
{
  AB.reserve(AB.size() + B.size());
  AB.insert(AB.end(), B.begin(), B.end());
}


double Utils::norm(float* vect)
{
  int sum = 0;
  for (int i = 0; i < 3; ++i)
  {
    sum += vect[i] * vect[i];
  }
  return sqrt(sum);
}

}
}
}
}
