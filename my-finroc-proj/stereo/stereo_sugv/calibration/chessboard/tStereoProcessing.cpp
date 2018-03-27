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
/*!\file    projects/icarus/sensor_processing/stereo_sugv/tStereoProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-13
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_sugv/calibration/chessboard/tStereoProcessing.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tStereoProcessing constructors
//----------------------------------------------------------------------
tStereoProcessing::tStereoProcessing()
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  //changing the resolution
  imageSettings.offsetX = 0;
  imageSettings.offsetY = 0;
  imageSettings.height = 900; //480;  //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
  imageSettings.width = 1200; //640; //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
  imageSettings.pixelFormat = PIXEL_FORMAT_MONO8;

  //RunSingleCamera(pointgrey_guid);
  bus_manager.GetCameraFromIndex(0, &pointgrey_guid); //left eth1
  camera.Connect(&pointgrey_guid);
  camera.SetGigEImageSettings(&imageSettings);
  camera.StartCapture();

  bus_manager.GetCameraFromIndex(1, &pointgrey_guid); //right eth2
  camera_2.Connect(&pointgrey_guid);
  camera_2.SetGigEImageSettings(&imageSettings);
  camera_2.StartCapture();

  boardSize.width = 9;
  boardSize.height = 6; //every cell i tested is 5*5 cm -- the same as the size of the my cells in grid map :D
}

//----------------------------------------------------------------------
// tStereoProcessing destructor
//----------------------------------------------------------------------
tStereoProcessing::~tStereoProcessing()
{
  // Stop capturing images
  camera.StopCapture();
  camera_2.StopCapture();

  // Disconnect the camera
  camera.Disconnect();
  camera_2.Disconnect();
  // Stop capturing images
  camera.StopCapture();
  camera_2.StopCapture();

  // Disconnect the camera
  camera.Disconnect();
  camera_2.Disconnect();
}

////----------------------------------------------------------------------
//// tStereoProcessing SomeExampleMethod
////----------------------------------------------------------------------
//void tStereoProcessing::SomeExampleMethod()
//{
//  /*This is an example for a method. Replace it by your own methods!*/
//}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}
