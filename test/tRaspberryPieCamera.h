//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    projects/test/tRaspberryPieCamera.h
 *
 * \author  Aras Dar
 *
 * \date    2018-04-06
 *
 * \brief   Contains tRaspberryPieCamera
 *
 * \b tRaspberryPieCamera
 *
 * This is the class to serialize the Raspberry Pie Camera class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__test__tRaspberryPieCamera_h__
#define __projects__test__tRaspberryPieCamera_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/serialization/serialization.h"
#include <raspicam/raspicam.h>
#include <fstream>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace test
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the class to serialize the Raspberry Pie Camera class.
 */
class tRaspberryPieCamera
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tRaspberryPieCamera();

  ~tRaspberryPieCamera();   //You need this destructor if you allocated memory on the heap that must be free'd.
  //Delete otherwise!

  //Here is the right place for your public methods. Replace this line by your declarations!
  void SomeExampleMethod();

  //Camera object
  raspicam::RaspiCam Camera;

  //allocate memory
  unsigned char *data;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables.
  //Replace this line by your declarations!

};

// introduction of these two operator in .h file
//implemetation will be in .cpp if the class is not template or in .hpp if the class is a template type;
rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const raspicam::RaspiCam& Camera);
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, raspicam::RaspiCam& Camera);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
