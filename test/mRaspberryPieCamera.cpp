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
/*!\file    projects/test/mRaspberryPieCamera.cpp
 *
 * \author  Aras Dar
 *
 * \date    2018-04-05
 *
 */
//----------------------------------------------------------------------
#include "projects/test/mRaspberryPieCamera.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <fstream>

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
runtime_construction::tStandardCreateModuleAction<mRaspberryPieCamera> cCREATE_ACTION_FOR_M_RASPBERRYPIECAMERA("RaspberryPieCamera");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mRaspberryPieCamera constructor
//----------------------------------------------------------------------
mRaspberryPieCamera::mRaspberryPieCamera(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false)//,
    // change to 'true' to make module's ports shared
	//(so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here.
  //Especially built-in types (like pointers!).
  //Delete this line otherwise!
{
	raspicam::RaspiCam Camera; //Camera object

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

	//capture
	Camera.grab();

	//allocate memory
	unsigned char *data=new unsigned char[  Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB )];

	//extract the image in rgb format
	Camera.retrieve ( data, raspicam::RASPICAM_FORMAT_RGB );//get camera image

	//save
	std::ofstream outFile ( "raspicam_image.ppm", std::ios::binary );
	outFile<<"P6\n"<<Camera.getWidth() <<" "<<Camera.getHeight() <<" 255\n";
	outFile.write ( ( char* ) data, Camera.getImageTypeSize ( raspicam::RASPICAM_FORMAT_RGB ) );
	std::cout <<"Image saved at raspicam_image.ppm"<< std::endl;

	//free resrources
	delete data;
}

//----------------------------------------------------------------------
// mRaspberryPieCamera destructor
//----------------------------------------------------------------------
mRaspberryPieCamera::~mRaspberryPieCamera()
{}

//----------------------------------------------------------------------
// mRaspberryPieCamera OnStaticParameterChange
//----------------------------------------------------------------------
void mRaspberryPieCamera::OnStaticParameterChange()
{
  if (this->static_parameter_1.HasChanged())
  {
    //As this static parameter has changed, do something with its value!
  }
}

//----------------------------------------------------------------------
// mRaspberryPieCamera OnParameterChange
//----------------------------------------------------------------------
void mRaspberryPieCamera::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed.
	//However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mRaspberryPieCamera Update
//----------------------------------------------------------------------
void mRaspberryPieCamera::Update()
{
  if (this->InputChanged())
  {
    //At least one of your input ports has changed.
	  //Do something useful with its data.
    //However, using the .HasChanged() method on each port you can check in more detail.
  }
  
  //Do something each cycle independent from changing ports.
  
  //this->out_signal_1.Publish(some meaningful value);
  //can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
