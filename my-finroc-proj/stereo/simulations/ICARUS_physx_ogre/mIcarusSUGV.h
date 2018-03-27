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
/*!\file    projects/simulation/elements/mIcarusLUGV.h
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-08-22
 *
 * \brief Contains mIcarusSUGV
 *
 * \b mIcarusSUGV
 *
 * This module performs the simulation of the ICARUS-SUGV.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__simulation__elements__mIcarusSUGV_h__
#define __projects__simulation__elements__mIcarusSUGV_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/simulation_physx_ogre/elements/tSimulationPhysxOgreElement.h"

#include "rrlib/math/tPose3D.h"
#include "rrlib/physics_simulation_physx/elements/tPhysXVehicle.h"
#include "rrlib/ogre/elements/tOgreElement.h"

#include <OGRE/OgreSceneManager.h>
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
 * This module performs the simulation of the ICARUS-SUGV.
 */
class mIcarusSUGV : public structure::tModule, public rrlib::physics_simulation_physx::elements::tPhysXVehicle,
  public rrlib::ogre::elements::tOgreElement, public finroc::simulation_physx_ogre::elements::tSimulationPhysxOgreElement
{
private:

  class VelAndHoldJoint
  {

  };

  const physx::PxVehiclePadSmoothingData smoothing_data =
  {
    {
      6.0f,   //rise rate eANALOG_INPUT_ACCEL
      6.0f,   //rise rate eANALOG_INPUT_BRAKE
      12.0f,  //rise rate eANALOG_INPUT_HANDBRAKE
      2.5f,   //rise rate eANALOG_INPUT_STEER_LEFT
      2.5f,   //rise rate eANALOG_INPUT_STEER_RIGHT
    },
    {
      10.0f,  //fall rate eANALOG_INPUT_ACCEL
      10.0f,  //fall rate eANALOG_INPUT_BRAKE
      12.0f,  //fall rate eANALOG_INPUT_HANDBRAKE
      5.0f,   //fall rate eANALOG_INPUT_STEER_LEFT
      5.0f    //fall rate eANALOG_INPUT_STEER_RIGHT
    }
  };

  static const std::string resource_group_name;
  static const std::string dir;

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:
  tParameter<rrlib::math::tPose3D> par_pose;

  tInput<bool> in_reset;
  tInput<float> in_accel;
  tInput<float> in_brake_left;
  tInput<float> in_brake_right;
  tInput<float> in_thrust_left;
  tInput<float> in_thrust_right;
  tInput<double> in_arm_0_angular_vel;
  tInput<double> in_arm_1_angular_vel;
  tInput<double> in_arm_2_angular_vel;
  tInput<double> in_arm_3_angular_vel;

  tOutput<rrlib::math::tVec3d> out_linear_velocity;
  tOutput<float> out_forwards_speed;
  tOutput<rrlib::math::tVec3d> out_angular_velocity;
  tOutput<bool> out_in_air;
  tOutput<rrlib::math::tPose3D> out_chassis_pose;
  tOutput<float> out_left_track_speed;
  tOutput<float> out_right_track_speed;
  tOutput<float> out_left_track_friction;
  tOutput<float> out_right_track_friction;
  tOutput<double> out_engine;
  tOutput<int> out_gear;
  tOutput<float> out_left_thrust;
  tOutput<float> out_right_thrust;
  tOutput<float> out_accel;
  tOutput<float> out_break_l;
  tOutput<float> out_break_r;

  tOutput<float> out_joint_0_angle;
  tOutput<float> out_joint_1_angle;
  tOutput<float> out_joint_2_angle;
  tOutput<float> out_joint_3_angle;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mIcarusSUGV(core::tFrameworkElement *parent, const std::string &name = "IcarusSUGV");
  virtual ~mIcarusSUGV();

  virtual void UpdateOgreElement();
  virtual void PreTick(double time_step);
  virtual void PostTick(double time_step);
  virtual void PostTick(const physx::PxVehicleWheelQueryResult &wq);

  virtual void InitOgreElement();
  virtual void ReleaseOgreElement();
  bool IsInitialized() const;

  virtual physx::PxVehicleWheels* GetVehicle()
  {
    return vehicle;
  }

  virtual const std::string GetName() const
  {
    return tModule::GetName();
  }

  virtual void DestroyElement();
  virtual void SetPose(const rrlib::math::tPose3D &pose);
  virtual const rrlib::math::tPose3D GetPose();

  Ogre::SceneNode *GetChassisNode();
//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:
private:

  physx::PxVehicleDriveTank* vehicle;
  physx::PxVehicleDriveTankRawInputData raw_input;

  physx::PxRigidDynamic *arm_0;
  physx::PxRigidDynamic *arm_1;
  physx::PxRigidDynamic *arm_2;
  physx::PxRigidDynamic *arm_3;
  physx::PxRigidDynamic *arm_4;

  physx::PxScene *px_scene;

  physx::PxRevoluteJoint *arm_joint_0;
  double arm_joint_0_angle;
  double arm_joint_0_v;
  double arm_joint_0_set;
  double arm_joint_0_set_i;
  bool arm_0_hold;
  physx::PxRevoluteJoint *arm_joint_1;
  double arm_joint_1_angle;
  double arm_joint_1_v;
  double arm_joint_1_set;
  double arm_joint_1_set_i;
  bool arm_1_hold;
  physx::PxRevoluteJoint *arm_joint_2;
  double arm_joint_2_angle;
  double arm_joint_2_v;
  double arm_joint_2_set;
  double arm_joint_2_set_i;
  bool arm_2_hold;
  physx::PxRevoluteJoint *arm_joint_3;
  double arm_joint_3_angle;
  double arm_joint_3_v;
  double arm_joint_3_set;
  double arm_joint_3_set_i;
  bool arm_3_hold;

  Ogre::SceneNode *chassis_node;
//  Ogre::SceneNode *wheel_fl_node;
//  Ogre::SceneNode *wheel_fr_node;
//  Ogre::SceneNode *wheel_ml_node;
//  Ogre::SceneNode *wheel_mr_node;
//  Ogre::SceneNode *wheel_rl_node;
//  Ogre::SceneNode *wheel_rr_node;

  Ogre::SceneNode *wheel_0l_node;
  Ogre::SceneNode *wheel_0r_node;
  Ogre::SceneNode *wheel_1l_node;
  Ogre::SceneNode *wheel_1r_node;
  Ogre::SceneNode *wheel_2l_node;
  Ogre::SceneNode *wheel_2r_node;
  Ogre::SceneNode *wheel_3l_node;
  Ogre::SceneNode *wheel_3r_node;
  Ogre::SceneNode *wheel_4l_node;
  Ogre::SceneNode *wheel_4r_node;
  Ogre::SceneNode *wheel_5l_node;
  Ogre::SceneNode *wheel_5r_node;
  Ogre::SceneNode *wheel_6l_node;
  Ogre::SceneNode *wheel_6r_node;
  Ogre::SceneNode *wheel_7l_node;
  Ogre::SceneNode *wheel_7r_node;
  Ogre::SceneNode *wheel_8l_node;
  Ogre::SceneNode *wheel_8r_node;

  Ogre::SceneNode *sn_arm_0;
  Ogre::SceneNode *sn_arm_1;
  Ogre::SceneNode *sn_arm_2;
  Ogre::SceneNode *sn_arm_3;

  // wheels
  Ogre::Vector3 l0_v, r0_v, l1_v, r1_v, l2_v, r2_v;
  Ogre::Vector3 l3_v, r3_v, l4_v, r4_v, l5_v, r5_v;
  Ogre::Vector3 l6_v, r6_v, l7_v, r7_v, l8_v, r8_v;
  Ogre::Quaternion l0_q, r0_q, l1_q, r1_q, l2_q, r2_q;
  Ogre::Quaternion l3_q, r3_q, l4_q, r4_q, l5_q, r5_q;
  Ogre::Quaternion l6_q, r6_q, l7_q, r7_q, l8_q, r8_q;
  // chassis and arm
  Ogre::Vector3 c_v, a0_v, a1_v, a2_v, a3_v;
  Ogre::Quaternion c_q, a0_q, a1_q, a2_q, a3_q;

  Ogre::Camera *vehicle_cam;
  rrlib::ogre::elements::tPoseWrapper *cam_pose_wrapper;

  virtual void OnParameterChange();

  virtual void Update();
  virtual void Reset();
  void InitPhysics(const std::string& name);
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
