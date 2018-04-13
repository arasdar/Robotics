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
/*!\file    projects/test/mRaspberryPiCamera.h
 *
 * \author  Aras Dar
 *
 * \date    2018-04-09
 *
 * \brief Contains mRaspberryPiCamera
 *
 * \b mRaspberryPiCamera
 *
 * This is the sensor controller/motor module for testing the Raspberry Pi camera class.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__test__mRaspberryPiCamera_h__
#define __projects__test__mRaspberryPiCamera_h__

#include "plugins/structure/tSenseControlModule.h"

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
 * This is the sensor controller/motor module for testing the Raspberry Pi camera class.
 */
class mRaspberryPiCamera : public structure::tSenseControlModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  tStaticParameter<double> static_parameter_1;   //Example for a static parameter. Replace or delete it!
  
  tParameter<double> par_parameter_1;   //Example for a runtime parameter named "Parameter 1. Replace or delete it!

  tSensorInput<double> si_signal_1;   //Example for sensor input ports named "Signal 1" and "Signal 2". Replace or delete them!
  tSensorInput<double> si_signal_2;

  tSensorOutput<double> so_signal_1;   //Examples for a sensor output port named "Signal 1". Replace or delete them!
//  tSensorOutput<tRaspberryPieCamera> so_camera;   //Examples for a sensor output port named "Signal 1". Replace or delete them!

  tControllerInput<double> ci_signal_1;   //Example for controller input ports named "Signal 1" and "Signal 2". Replace or delete them!
  tControllerInput<double> ci_signal_2;

  tControllerOutput<double> co_signal_1;   //Examples for a controller output port named "Signal 1". Replace or delete them!

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mRaspberryPiCamera(core::tFrameworkElement *parent, const std::string &name = "RaspberryPiCamera");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  //Here is the right place for your variables. Replace this line by your declarations!
  tRaspberryPieCamera camera;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mRaspberryPiCamera();

  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Sense() override;

  virtual void Control() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}



#endif
