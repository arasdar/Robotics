/*
 * sample_FileStorage.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: aras
 */





//#include "opencv2/opencv.hpp"
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;

#include "projects/stereo_traversability_experiments/def/tMLR.h"

//int main(int, char** argv)
int finroc::stereo_traversability_experiments::def::tMLR::file_storage_write(int argc, const char** argv)
{
  FileStorage fs("test.yml", FileStorage::WRITE);

  fs << "frameCount" << 5;
  time_t rawtime;
  time(&rawtime);
  fs << "calibrationDate" << asctime(localtime(&rawtime));
  Mat cameraMatrix = (Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
  Mat distCoeffs = (Mat_<double>(5, 1) << 0.1, 0.01, -0.001, 0, 0);
  fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
  fs << "features" << "[";
  for (int i = 0; i < 3; i++)
  {
    int x = rand() % 640;
    int y = rand() % 480;
    uchar lbp = rand() % 256;

    fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
    for (int j = 0; j < 8; j++)
      fs << ((lbp >> j) & 1);
    fs << "]" << "}";
  }
  fs << "]";
  fs.release();
  return 0;
}



/*
%YAML:1.0
frameCount: 5
calibrationDate: "Fri Jun 17 14:09:29 2011\n"
cameraMatrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1000., 0., 320., 0., 1000., 240., 0., 0., 1. ]
distCoeffs: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ 1.0000000000000001e-01, 1.0000000000000000e-02,
       -1.0000000000000000e-03, 0., 0. ]
features:
   - { x:167, y:49, lbp:[ 1, 0, 0, 1, 1, 0, 1, 1 ] }
   - { x:298, y:130, lbp:[ 0, 0, 0, 1, 0, 0, 1, 1 ] }
   - { x:344, y:158, lbp:[ 1, 1, 0, 0, 0, 0, 1, 0 ] }*/


//int main(int ac, const char** ag)
int finroc::stereo_traversability_experiments::def::tMLR::file_storage_read(int argc, const char** argv)
{

  FileStorage fs2("test.yml", FileStorage::READ);

  // first method: use (type) operator on FileNode.
  int frameCount = (int)fs2["frameCount"];

  std::string date;
  // second method: use FileNode::operator >>
  fs2["calibrationDate"] >> date;

  Mat cameraMatrix2, distCoeffs2;
  fs2["cameraMatrix"] >> cameraMatrix2;
  fs2["distCoeffs"] >> distCoeffs2;

  cout << "frameCount: " << frameCount << endl
       << "calibration date: " << date << endl
       << "camera matrix: " << cameraMatrix2 << endl
       << "distortion coeffs: " << distCoeffs2 << endl;

  FileNode features = fs2["features"];
  FileNodeIterator it = features.begin(), it_end = features.end();
  int idx = 0;
  std::vector<uchar> lbpval;

  // iterate through a sequence using FileNodeIterator
  for (; it != it_end; ++it, idx++)
  {
    cout << "feature #" << idx << ": ";
    cout << "x=" << (int)(*it)["x"] << ", y=" << (int)(*it)["y"] << ", lbp: (";
    // you can also easily read numerical arrays using FileNode >> std::vector operator.
    (*it)["lbp"] >> lbpval;
    for (int i = 0; i < (int)lbpval.size(); i++)
      cout << " " << (int)lbpval[i];
    cout << ")" << endl;
  }
  fs2.release();

  return 0;
}

