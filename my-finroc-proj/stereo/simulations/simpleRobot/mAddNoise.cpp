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
/*!\file    projects/simple_robot_simulation/mAddNoise.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2013-09-03
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_simple_robot/mAddNoise.h"

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
using namespace finroc::data_ports;

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
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mAddNoise> cCREATE_ACTION_FOR_M_ADDNOISE("AddNoise");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mAddNoise constructor
//----------------------------------------------------------------------
mAddNoise::mAddNoise(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false, false),
  // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  input(tUnit::m),
  output(tUnit::m),
  standard_deviation(0.05, tUnit::m),
  normal_distribution(0, 0.05),
  eng(1234)
{}

//----------------------------------------------------------------------
// mAddNoise destructor
//----------------------------------------------------------------------
mAddNoise::~mAddNoise()
{}

//----------------------------------------------------------------------
// mAddNoise OnStaticParameterChange
//----------------------------------------------------------------------
void mAddNoise::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    //As this static parameter has changed, do something with its value!
//  }
}

//----------------------------------------------------------------------
// mAddNoise OnParameterChange
//----------------------------------------------------------------------
void mAddNoise::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.

  normal_distribution = std::normal_distribution<double>(0, standard_deviation.Get());
}

//----------------------------------------------------------------------
// mAddNoise Update
//----------------------------------------------------------------------
void mAddNoise::Update()
{
//  if (this->InputChanged())
//  {
//    /*At least one of your input ports has changed. Do something useful with its data.
//    However, using the .HasChanged() method on each port you can check in more detail.
//     * */
//
//  }

  //Do something each cycle independent from changing ports.
  output.Publish(input.Get() + normal_distribution(eng));

  //this->output_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
