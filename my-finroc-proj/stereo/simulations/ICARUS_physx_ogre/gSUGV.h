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
/*!\file    projects/icarus/simulation_physx_ogre/gSUGV.h
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-09-30
 *
 * \brief Contains gSUGV
 *
 * \b gSUGV
 *
 * This group encapsulates the SUGV.This group encapsulates the SUGV and its sensors.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__simulation_physx_ogre__gSUGV_h__
#define __projects__stereo_traversability_experiments__simulation_physx_ogre__gSUGV_h__

#include "plugins/structure/tGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/simulation_physx_ogre/elements/sensors/mDepthCamera.h"
#include "libraries/simulation_physx_ogre/elements/sensors/mSickLMS511.h"
#include "libraries/vehicle_kinematics/mDifferentialDriveKinematics.h"

#include "rrlib/coviroa/tImage.h"
#include "rrlib/distance_data/tDistanceData.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/mIcarusSUGV.h"
//#include "projects/icarus/simulation_physx_ogre/mRumblePad.h"
//#include "projects/icarus/simulation_physx_ogre/mGUIJoystick.h"
//#include "projects/icarus/simulation_physx_ogre/mTrackVelocityController.h"
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/mTrackVelocityController.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation_physx_ogre
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This group encapsulates the SUGV.This group encapsulates the SUGV and its sensors.
 */
class gSUGV : public structure::tGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:
  tInput<float> in_velocity;
  tInput<float> in_angular_velocity;
  tInput<double> in_arm_0_angular_vel;
  tInput<double> in_arm_1_angular_vel;
  tInput<double> in_arm_2_angular_vel;
  tInput<double> in_arm_3_angular_vel;


  tOutput<float> out_velocity;
  tOutput<float> out_angular_velocity;
  tOutput<rrlib::math::tPose3D> out_current_pose;

  tOutput<std::vector<rrlib::coviroa::tImage>> out_stereo_images;
  tOutput<rrlib::distance_data::tDistanceData> out_front_lrf;
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gSUGV(core::tFrameworkElement *parent, const std::string &name = "SUGV",
        const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
  mIcarusSUGV *sugv;
//  mGUIJoystick *gui_joystick;
  finroc::simulation_physx_ogre::elements::sensors::mSickLMS511* front_scanner;
  finroc::vehicle_kinematics::mDifferentialDriveKinematics* kinematic;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~gSUGV();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
