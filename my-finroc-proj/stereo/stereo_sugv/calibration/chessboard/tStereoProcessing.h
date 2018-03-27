//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/sensor_processing/stereo_sugv/tStereoProcessing.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-13
 *
 * \brief   Contains tStereoProcessing
 *
 * \b tStereoProcessing
 *
 * This is the class for chessboard detection
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__stereo_sugv__tStereoProcessing_h__
#define __projects__icarus__sensor_processing__stereo_sugv__tStereoProcessing_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
// system files
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <FlyCapture2.h>

#include <iostream>
#include <vector>
#include <stdio.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace stereo_sugv
{
namespace calibration
{

// using namespace
using namespace std;
using namespace cv; //opencv
using namespace FlyCapture2;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum tPattern { CHESSBOARD };

struct chessboardDataOutput
{
  Mat view, view_chessboard;
  bool found = false;
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the class for chessboard detection
 */
class tStereoProcessing
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tStereoProcessing();

  ~tStereoProcessing();   //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  /*Here is the right place for your public methods. Replace this line by your declarations!*/
  IplImage* ConvertImageToOpenCV(Image* pImage);
  chessboardDataOutput chessboardDetection(const IplImage* opencv_img_input);

  void run();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  BusManager bus_manager;
  PGRGuid pointgrey_guid;
  GigECamera camera;
  Image flycap_img;
  GigECamera camera_2;
  Image flycap_img_2;
  Image colorImage;
  bool bInitialized = false;

  // changing the resolution of images
  GigEImageSettings imageSettings;


  //saving frames
  int images_idx;
  int n_l = 1, n_r = 1;
  char filename[200];

  /* ------- chessboard detection --------------*/
  Size boardSize, imageSize;
  Mat cameraMatrix, distCoeffs;
  int delay = 1;
  clock_t prevTimestamp = 0;
  int mode = DETECTION;
  vector<vector<Point2f>> imagePoints;
  tPattern pattern = CHESSBOARD;
  bool blink = false;
  vector<Point2f> pointbuf;
  bool found = false;
  chessboardDataOutput data_1, data_2;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}

#endif
