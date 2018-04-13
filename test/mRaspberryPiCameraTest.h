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
/*!\file    projects/test/mRaspberryPiCameraTest.h
 *
 * \author  Aras Dar
 *
 * \date    2018-04-08
 *
 * \brief Contains mRaspberryPiCameraTest
 *
 * \b mRaspberryPiCameraTest
 *
 * This is the module for testing the serialized and run-type data type idetified/informed.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__test__mRaspberryPiCameraTest_h__
#define __projects__test__mRaspberryPiCameraTest_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/test/tRaspberryPieCamera.h"

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
 * This is the module for testing the serialized and run-type data type idetified/informed.
 */
class mRaspberryPiCameraTest : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<double> static_parameter_1;   //Example for a static parameter. Replace or delete it!

  tParameter<double> par_parameter_1;   //Example for a runtime parameter named "Parameter 1". Replace or delete it!

  tInput<double> in_signal_1;   //Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
  tInput<double> in_signal_2;

  tOutput<double> out_signal_1;   //Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
  tOutput<double> out_signal_2;

//	tInput<tRaspberryPieCamera> in_camera;
//	tOutput<tRaspberryPieCamera> out_camera;

  tInput<tAnnotatedPose2D> in_pose;
  tOutput<tAnnotatedPose2D> out_pose;


//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mRaspberryPiCameraTest(core::tFrameworkElement *parent, const std::string &name = "RaspberryPiCameraTest");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables.
  //Replace this line by your declarations!
//  tRaspberryPieCamera camera;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mRaspberryPiCameraTest();

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
