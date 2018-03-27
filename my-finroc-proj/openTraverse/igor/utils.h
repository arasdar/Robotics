#ifndef _TRAVERSE_UTILS_H_
#define _TRAVERSE_UTILS_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <tr1/unordered_map>

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

using namespace std;

#define RED 1
#define GREEN 2
#define BIG_NUM 100000000

struct Voxel
{
  int green;
  int red;
  float depth;
  vector<int> indexesInPointCloud;
};

typedef tr1::unordered_map<int, tr1::unordered_map<int, map<int, Voxel>>> HashMap3D;
typedef tr1::unordered_map<int, map<int, Voxel>> HashMap2D;
typedef map<int, Voxel> Map1D;

class Utils
{
public:
  static Voxel createVoxel(const bool& isTraversable, double depth, int index = -1);
  static void LabelNeighbourhood(int x, int y, int l, const cv::Mat& data, cv::Mat& labels, int neighborhoodSize);
  static int blobDetection(const cv::Mat& input, cv::Mat& labels, int neighborhoodSize);
  static bool validPoint(const cv::Mat& input, int x, int y, int neighborhoodSize);
  static void concatenateVectors(std::vector<int>& AB, const vector<int>& B);
  static double norm(float* vect);
};

}
}
}
}

#endif
