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
#include <fstream>
//#include <unistd.h>

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
tRaspberryPieCamera::tRaspberryPieCamera()
//		data(new unsigned char[Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)]) // image size= image type x image shape (NHWC) for RGB
  //If you have some member variables,
  //please initialize them here.
  //Especially built-in types (like pointers!).
  //Delete this line otherwise!
{
	// Some thoughts & NOTES
	// Here it is important to make sure this new chunk of mem OR new dynamicly refrences buffer is not NULL;
	// assert or nullpnter can hep to make sure this mem block or mem chunk is assigned successfully.
	// testing/ unittesting or assert or if clause for nullpnter meaning a pointer with no or null address
	// try and catch is helpfull in this case? IDK

	// It looks like this initializes thr Camera obj/class/instantiated object
	// We need to make sure that the Camera class/ buffer/ mem is assigned successfully
	// The camera can open and start receving the images/ feed/ stream
	//Open camera
	std::cout<<"Opening Camera..."<<std::endl;
	if ( !Camera.open())
	{
		std::cerr<<"Error opening camera"<<std::endl;
	//		return -1;
	}

	//	// wait a while until camera stabilizes
	//	std::cout<<"Sleeping for 3 secs"<<std::endl;
	//	usleep(3*1000000); // 3x1e6 or 10^6 or million; unit?

	// Initializing Camera Data Buffer
	// new: Dynamic addressing or mem assignment or mem allocation NOT static mem allocation
	// unsigned char: data type or class
	// [ImageTypeSize]: array or data blocks number
	data = new unsigned char[Camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB)];
}

//----------------------------------------------------------------------
// tRaspberryPieCamera destructor
//----------------------------------------------------------------------
tRaspberryPieCamera::~tRaspberryPieCamera()
{
	//free resources
//	delete Camera; // Only one block
	delete[] data; // very many blocks data type x data shape; data shape NxCxHXW or NCHW or NHWC, C is RGB in this case or 3 channels
}

//----------------------------------------------------------------------
// tRaspberryPieCamera SomeExampleMethod
//----------------------------------------------------------------------
void tRaspberryPieCamera::GetData()
{
	//This is an example for a method.
	//Replace it by your own methods!
	//capture an image in Camera class or buffer
	Camera.grab();
	//extract the image in rgb format
	Camera.retrieve (data, raspicam::RASPICAM_FORMAT_RGB );//get camera image

//	//save
//	std::ofstream outFile ( "raspicam_image.ppm", std::ios::binary );
//	outFile<<"P6\n"<< Camera.getWidth() <<" "<< Camera.getHeight() <<" 255\n";
//	outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB) );
//	std::cout <<"Image saved at raspicam_image.ppm"<< std::endl;

}

void tRaspberryPieCamera::SaveData()
{
	//save
	std::ofstream outFile ( "raspicam_image.ppm", std::ios::binary );
	outFile<<"P6\n"<< Camera.getWidth() <<" "<< Camera.getHeight() <<" 255\n";
	outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB) );
	std::cout <<"Image saved at raspicam_image.ppm"<< std::endl;
}

rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tRaspberryPieCamera& Camera)
{
	// Implementation of this operator/function
//	Camera.GetData();
	stream << *(Camera.data);
	return stream;
}
rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tRaspberryPieCamera& Camera)
{
//	// Implementation of this operator or function
//	Camera.GetData();
	stream >> *(Camera.data);
	return stream;
}

//rrlib::serialization::tOutputStream& operator << (rrlib::serialization::tOutputStream& stream, const tAnnotatedPose2D& pose)
//{
//  stream << pose.pose << pose.name; //<< pose.last_visit;
//  return stream;
//}
//
//rrlib::serialization::tInputStream& operator >> (rrlib::serialization::tInputStream& stream, tAnnotatedPose2D& pose)
//{
//  stream >> pose.pose >> pose.name; // >> pose.last_visit;
//  return stream;
//}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}// test
}// finroc
