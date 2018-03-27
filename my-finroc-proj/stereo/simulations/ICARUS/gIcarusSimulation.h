//
// You received this file as part of Finroc
// A framework for integrated robot control
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
/*!\file    gIcarusSimulation.h
 *
 * \author  Aras Dargazany
 * \date    2012-05-16
 *
 */
//----------------------------------------------------------------------
#ifndef _icarus__gIcarusSimulation_h_
#define _icarus__gIcarusSimulation_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/algorithm/string.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/structure/tModule.h"
#include "plugins/structure/tSenseControlGroup.h"
//#include "plugins/runtime_construction/tFinstructableGroup.h"
#include "plugins/scheduling/tThreadContainerElement.h"
#include "libraries/laser_scanner/simulation/mLaserScannerCoin.h"
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tVector.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/physics_simulation_newton/tEngineNewton.h"
#include "rrlib/simvis3d/tFrameGrabberCoin.h"
#include "rrlib/mapping/definitions.h"
#include "rrlib/canvas/tCanvas2D.h"


//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <vector>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------


class gIcarusSimulation : public structure::tSenseControlGroup
{
  static finroc::runtime_construction::tStandardCreateModuleAction<gIcarusSimulation> cCREATE_ACTION;

  //----------------------------------------------------------------------
  // Ports (These are the only variables that may be declared public)
  //----------------------------------------------------------------------
public:

  /*
   * kinematics for differential drive
   */
  tControllerInput<double> ci_angular_velocity, ci_velocity, ci_enable_motors; //double added by me
  tSensorOutput<rrlib::math::tPose3D> so_pose;
  tSensorOutput<double> so_pose_x, so_pose_y, so_pose_yaw; //double added by me

  //----------------------------------------------------------------------
  // Public methods and typedefs (no fields/variables)
  //----------------------------------------------------------------------
public:

  gIcarusSimulation(core::tFrameworkElement *parent, const std::string &name = "IcarusSimulation",
                    const std::string &structure_config_file = __FILE__".xml");


  //----------------------------------------------------------------------
  // Protected methods (no fields/variables)
  //----------------------------------------------------------------------
protected:


  //----------------------------------------------------------------------
  // Private fields and methods
  //----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
