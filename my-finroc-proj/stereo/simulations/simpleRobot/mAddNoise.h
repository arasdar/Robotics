//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    projects/simple_robot_simulation/mAddNoise.h
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 * \brief Contains mAddNoise
 *
 * \b mAddNoise
 *
 * this is a plain Module for simulating sensor noise.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__simple_robot_simulation__mAddNoise_h__
#define __projects__simple_robot_simulation__mAddNoise_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <random>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation_simple_robot
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this is a plain Module for simulating sensor noise.
 */
class mAddNoise : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  /*
//   * example ports
//   */
//  tStaticParameter<double> static_parameter_1;   Example for a static parameter. Replace or delete it!
//
//  tParameter<double> parameter_1;   Example for a runtime parameter. Replace or delete it!
//
//  tInput<double> input_signal_1;   Example for input ports. Replace or delete them!
//  tInput<double> input_signal_2;
//
//  tOutput<double> output_signal_1;   Examples for output ports. Replace or delete them!
//  tOutput<double> output_signal_2;

  /*! Input value */
  tInput<double> input;

  /*! Output value (= input value with added noise) */
  tOutput<double> output;

  /*! Standard deviation for added noise */
  tParameter<double> standard_deviation;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mAddNoise(core::tFrameworkElement *parent, const std::string &name = "AddNoise");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*
   * Here is the right place for your variables. Replace this line by your declarations!
   */
  std::normal_distribution<double> normal_distribution;
  std::mt19937 eng;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mAddNoise();

  virtual void OnStaticParameterChange();   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
