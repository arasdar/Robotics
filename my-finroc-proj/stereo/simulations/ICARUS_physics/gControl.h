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
/*!\file    projects/stereo_traversability_experiments/simulation_physics/test/gControl.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-11-18
 *
 * \brief Contains gControl
 *
 * \b gControl
 *
 * This is the control group for generating the controling command to the action group for comtrolling the robot dynamic.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__simulation_physics__test__gControl_h__
#define __projects__stereo_traversability_experiments__simulation_physics__test__gControl_h__

#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
namespace simulation_physics
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the control group for generating the controling command to the action group for comtrolling the robot dynamic.
 */
class gControl : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gControl(core::tFrameworkElement *parent, const std::string &name = "Control",
           const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*! Destructor
   *
   * The destructor of groups is declared private to avoid accidental deletion. Deleting
   * groups is already handled by the framework.
   */
  ~gControl();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
