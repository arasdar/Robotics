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
/*!\file    projects/test/tRaspberryPieCamera.cpp
 *
 * \author  Aras Dar
 *
 * \date    2018-04-06
 *
 */
//----------------------------------------------------------------------
#include "projects/test/tRaspberryPieCamera.h"

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
namespace test
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
// tRaspberryPieCamera constructors
//----------------------------------------------------------------------
tRaspberryPieCamera::tRaspberryPieCamera():
	data(new unsigned char[Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)])
  //If you have some member variables,
  //please initialize them here.
  //Especially built-in types (like pointers!).
  //Delete this line otherwise!
{
	//Open camera
	std::cout<<"Opening Camera..."<<std::endl;
	if ( !Camera.open())
	{
		std::cerr<<"Error opening camera"<<std::endl;
	//		return -1;
	}
	//wait a while until camera stabilizes
	std::cout<<"Sleeping for 3 secs"<<std::endl;
	//	usleep(3*1000000);
}

//----------------------------------------------------------------------
// tRaspberryPieCamera destructor
//----------------------------------------------------------------------
tRaspberryPieCamera::~tRaspberryPieCamera()
{
	//free resrources
	delete data;
}

//----------------------------------------------------------------------
// tRaspberryPieCamera SomeExampleMethod
//----------------------------------------------------------------------
void tRaspberryPieCamera::SomeExampleMethod()
{
  //This is an example for a method.
	//Replace it by your own methods!
	//capture
	Camera.grab();

	//extract the image in rgb format
	Camera.retrieve ( data, raspicam::RASPICAM_FORMAT_RGB );//get camera image

	//save
	std::ofstream outFile ( "raspicam_image.ppm", std::ios::binary );
	outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
	outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
	std::cout <<"Image saved at raspicam_image.ppm"<< std::endl;
}

rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tRaspberryPieCamera& cam)
{
	// Implementation of this operator/function
	stream << *cam.data;
	return stream;
}
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tRaspberryPieCamera& cam)
{
	// Implementation of this operator or function
	stream >> *cam.data;
	return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
