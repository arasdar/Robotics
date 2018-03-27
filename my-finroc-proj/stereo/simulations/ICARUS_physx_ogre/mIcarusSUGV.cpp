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
/*!\file    projects/simulation/elements/mIcarusSUGV.cpp
 *
 * \author  Thorsten Ropertz
 *
 * \date    2013-08-22
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/simulation_physx_ogre/mIcarusSUGV.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>

#include "libraries/simulation_physx_ogre/tSimulation.h"
#include "libraries/simulation_physx_ogre/tCommon.h"

#include "rrlib/physics_simulation_physx/tMeshFactory.h"
#include "rrlib/ogre/elements/tPoseWrapper.h"

#include "rrlib/math/tPose3D.h"
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
namespace stereo_traversability_experiments
{
namespace simulation_physx_ogre
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mIcarusSUGV> cCREATE_ACTION_FOR_M_ICARUSSUGV("IcarusSUGV");
const std::string mIcarusSUGV::resource_group_name = "Icarus_SUGV_Resources";
const std::string mIcarusSUGV::dir = std::string(std::getenv("FINROC_PROJECT_HOME")) + ("/simulation_physx_ogre/etc/models/ICARUS_SUGV/");

const rrlib::math::tPose3D arm_0_pose(0, 0, 0.837, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0));
const rrlib::math::tPose3D arm_1_pose(-0.674, 0, 0.9, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0));
const rrlib::math::tPose3D arm_2_pose(-0.74, 0, 1.163, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0));
const rrlib::math::tPose3D arm_3_pose(0.616, 0, 1.39, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0));
const rrlib::math::tPose3D arm_4_pose(0.619, 0, 1.51, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0));

const rrlib::math::tPose3D arm_joint_0_pose(0, 0, 0.685, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(M_PI_2), rrlib::math::tAngleRad(0));
const rrlib::math::tPose3D arm_joint_1_pose(0, 0, 0.873, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(-M_PI_2));
const rrlib::math::tPose3D arm_joint_2_pose(-1.397, 0, 1, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(M_PI_2));
const rrlib::math::tPose3D arm_joint_3_pose(0.697, 0, 1.207, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(-0.6646831539752474), rrlib::math::tAngleRad(-M_PI_2));

const unsigned int number_of_wheels = 18;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mIcarusSUGV constructor
//----------------------------------------------------------------------
mIcarusSUGV::mIcarusSUGV(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name),
  rrlib::physics_simulation_physx::elements::tPhysXVehicle(finroc::simulation_physx_ogre::engine::Instance()),
  rrlib::ogre::elements::tOgreElement(finroc::simulation_physx_ogre::engine::Instance(), name),
  raw_input(physx::PxVehicleDriveTankControlModel::eSPECIAL),
  arm_joint_0_angle(0),
  arm_joint_0_v(0),
  arm_joint_0_set(0),
  arm_joint_0_set_i(0),
  arm_0_hold(true),
  arm_joint_1(nullptr),
  arm_joint_1_angle(0),
  arm_joint_1_v(0),
  arm_joint_1_set(0),
  arm_joint_1_set_i(0),
  arm_1_hold(true),
  arm_joint_2(nullptr),
  arm_joint_2_angle(0),
  arm_joint_2_v(0),
  arm_joint_2_set(0),
  arm_joint_2_set_i(0),
  arm_2_hold(true),
  arm_joint_3(nullptr),
  arm_joint_3_angle(0),
  arm_joint_3_v(0),
  arm_joint_3_set(0),
  arm_joint_3_set_i(0),
  arm_3_hold(true),
  wheel_0l_node(nullptr),
  wheel_0r_node(nullptr),
  wheel_1l_node(nullptr),
  wheel_1r_node(nullptr),
  wheel_2l_node(nullptr),
  wheel_2r_node(nullptr),
  wheel_3l_node(nullptr),
  wheel_3r_node(nullptr),
  wheel_4l_node(nullptr),
  wheel_4r_node(nullptr),
  wheel_5l_node(nullptr),
  wheel_5r_node(nullptr)
{
  // init ports
  {
    in_reset.Init();
    in_accel.Init();
    in_brake_left.Init();
    in_brake_right.Init();
    in_thrust_left.Init();
    in_thrust_right.Init();
    in_arm_0_angular_vel.Init();
    in_arm_1_angular_vel.Init();
    in_arm_2_angular_vel.Init();
    in_arm_3_angular_vel.Init();
    out_linear_velocity.Init();
    out_forwards_speed.Init();
    out_angular_velocity.Init();
    out_in_air.Init();
    out_chassis_pose.Init();
    out_left_track_speed.Init();
    out_right_track_speed.Init();
    out_left_track_friction.Init();
    out_right_track_friction.Init();
    out_joint_0_angle.Init();
    out_joint_1_angle.Init();
    out_joint_2_angle.Init();
    out_joint_3_angle.Init();
  }
  this->par_pose.Set(rrlib::math::tPose3D(0, 0, 0, rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0), rrlib::math::tAngleRad(0)));

  // Initialize visualization
  {
    tOgreElement::engine.InitOgreElement(this);
  }

  // Initialize physics
  InitPhysics(name);
}

void mIcarusSUGV::DestroyElement()
{
  delete(this);
}

void mIcarusSUGV::SetPose(const rrlib::math::tPose3D& pose)
{
  par_pose.Set(pose);
  this->Reset();
}

const rrlib::math::tPose3D mIcarusSUGV::GetPose()
{
  physx::PxSceneReadLock lock(*px_scene);
  return finroc::simulation_physx_ogre::cvt(actor->getGlobalPose());
}

void mIcarusSUGV::InitPhysics(const std::string& name)
{
  const double mass = 50;
  const physx::PxVec3 com(0.0, 0.0, -0.08);
  const float chassis_length = 1.083;
  const float chassis_width = 0.45;
  const float chassis_height = 0.31;

  // old wheel configuration with 6 pairs of wheels
//  const physx::PxVec3 w_center_offsets[number_of_wheels] =
//  {
//    physx::PxVec3(-0.479,  0.185, -0.08),
//    physx::PxVec3(-0.479, -0.185, -0.08),
//    physx::PxVec3(-0.28,   0.185, -0.08),
//    physx::PxVec3(-0.28,  -0.185, -0.08),
//    physx::PxVec3(-0.081,  0.185, -0.08),
//    physx::PxVec3(-0.081, -0.185, -0.08),
//    physx::PxVec3(0.118,  0.185, -0.08),
//    physx::PxVec3(0.118, -0.185, -0.08),
//    physx::PxVec3(0.318,  0.185, -0.08),
//    physx::PxVec3(0.318, -0.185, -0.08),
//    physx::PxVec3(0.478,  0.185,  0.017),
//    physx::PxVec3(0.478, -0.185,  0.017),
//  };

  const physx::PxVec3 w_center_offsets[number_of_wheels] =
  {
    physx::PxVec3(-0.479,  0.185, -0.08),
    physx::PxVec3(-0.479, -0.185, -0.08),
    physx::PxVec3(-0.346,  0.185, -0.08),
    physx::PxVec3(-0.346, -0.185, -0.08),
    physx::PxVec3(-0.213,  0.185, -0.08),
    physx::PxVec3(-0.213, -0.185, -0.08),
    physx::PxVec3(-0.08,  0.185, -0.08),
    physx::PxVec3(-0.08, -0.185, -0.08),
    physx::PxVec3(0.053,  0.185, -0.08),
    physx::PxVec3(0.053, -0.185, -0.08),
    physx::PxVec3(0.186,  0.185, -0.08),
    physx::PxVec3(0.186, -0.185, -0.08),
    physx::PxVec3(0.318,  0.185, -0.08),
    physx::PxVec3(0.318, -0.185, -0.08),
    physx::PxVec3(0.40,  0.185, -0.034),
    physx::PxVec3(0.40, -0.185, -0.034),
    physx::PxVec3(0.478,  0.185,  0.017),
    physx::PxVec3(0.478, -0.185,  0.017),
  };

  // approx moi
  const physx::PxVec3 moi(
    (chassis_height * chassis_height
     + chassis_length * chassis_length) * mass / 12.0f,
    (chassis_width * chassis_width + chassis_length * chassis_length)
    * mass / 12.0f,
    (chassis_width * chassis_width + chassis_height * chassis_height)
    * mass / 12.0f);
  finroc::simulation_physx_ogre::tSimulation& sim =
    finroc::simulation_physx_ogre::engine::Instance();
  physx::PxPhysics& px = *sim.GetPhysx();
  px_scene = sim.GetPhyScene();
  actor = px.createRigidDynamic(physx::PxTransform(physx::PxIdentity));
  actor->setName(name.c_str());
  actor->userData =
    //dynamic_cast<rrlib::physics_simulation_physx::elements::tPhysXVehicle*>(this);
    dynamic_cast<rrlib::physics_simulation_physx::elements::tPhysxElementBase*>(this);
  RRLIB_LOG_PRINT(DEBUG, "Set userData=", actor->userData, " this=",
                  this);
  arm_0 = px.createRigidDynamic(
            finroc::simulation_physx_ogre::cvt(arm_0_pose));
  arm_1 = px.createRigidDynamic(
            finroc::simulation_physx_ogre::cvt(arm_1_pose));
  arm_2 = px.createRigidDynamic(
            finroc::simulation_physx_ogre::cvt(arm_2_pose));
  arm_3 = px.createRigidDynamic(
            finroc::simulation_physx_ogre::cvt(arm_3_pose));
  wheel_coll_filter_data.word0 =
    rrlib::physics_simulation_physx::tCollisionFlag::WHEEL;
  wheel_coll_filter_data.word1 =
    rrlib::physics_simulation_physx::tCollisionFlag::WHEEL_AGAINST;
  chassis_coll_filter_data.word0 =
    rrlib::physics_simulation_physx::tCollisionFlag::CHASSIS;
  chassis_coll_filter_data.word1 =
    rrlib::physics_simulation_physx::tCollisionFlag::CHASSIS_AGAINST;
  rrlib::physics_simulation_physx::tSceneQueryData::SetupNonDrivableShapeQueryFilterData(
    &veh_qry_filter_data);

  physx::PxVehicleWheelsSimData* wheels_data =
    physx::PxVehicleWheelsSimData::allocate(number_of_wheels);
  physx::PxVehicleChassisData chassis_data;
  physx::PxVehicleDriveSimData4W drive_data;
  physx::PxVehicleWheelData wheel;
  physx::PxVehicleTireData tire;
  physx::PxVehicleSuspensionData susp;
  physx::PxVehicleEngineData engine;
  physx::PxVehicleGearsData gears;
  physx::PxShape* shapes[number_of_wheels + 1];
  {
    // setup chassis
    {
      std::string mat_id;
      shapes[number_of_wheels] = actor->createShape(
                                   physx::PxConvexMeshGeometry(
                                     sim.GetMeshFactory().LoadConvexMesh(
                                       dir + "Chassis.obj", mat_id)),
                                   *sim.GetMaterial("steel"));

      physx::PxShape* chain_left = actor->createShape(
                                     physx::PxConvexMeshGeometry(
                                       sim.GetMeshFactory().LoadConvexMesh(
                                         dir + "chain_left.obj", mat_id)),
                                     *px.createMaterial(0.01, 0.01, 0.01));
      chain_left->setLocalPose(physx::PxTransform(physx::PxIdentity));
      chain_left->setQueryFilterData(veh_qry_filter_data);
      chain_left->setSimulationFilterData(chassis_coll_filter_data);
      actor->attachShape(*chain_left);

      physx::PxShape* chain_right = actor->createShape(
                                      physx::PxConvexMeshGeometry(
                                        sim.GetMeshFactory().LoadConvexMesh(
                                          dir + "chain_right.obj", mat_id)),
                                      *px.createMaterial(0.01, 0.01, 0.01));
      chain_right->setLocalPose(physx::PxTransform(physx::PxIdentity));
      chain_right->setQueryFilterData(veh_qry_filter_data);
      chain_right->setSimulationFilterData(chassis_coll_filter_data);
      actor->attachShape(*chain_right);

      shapes[number_of_wheels]->setLocalPose(physx::PxTransform(physx::PxIdentity));
      shapes[number_of_wheels]->setQueryFilterData(veh_qry_filter_data);
      shapes[number_of_wheels]->setSimulationFilterData(chassis_coll_filter_data);
      physx::PxRigidBodyExt::setMassAndUpdateInertia(*actor, mass, &com);
      chassis_data.mMass = mass;
      chassis_data.mCMOffset = com;
      chassis_data.mMOI = actor->getMassSpaceInertiaTensor(); //moi;
      wheels_data->setChassisMass(mass);
    }
    {
      wheel.mMass = 8.0;
      wheel.mMaxHandBrakeTorque = 1500.0;
      wheel.mMaxBrakeTorque = 1500.0;
      wheel.mDampingRate = 2.0;
      wheel.mRadius = 0.055;
      wheel.mWidth = 0.09;
      // If the wheel is approximately cylindrical then a simple formula can be used to compute MOI:
      // MOI = 0.5 * Mass * Radius * Radius
      wheel.mMOI = 0.5 * wheel.mMass * wheel.mRadius * wheel.mRadius;
      tire.mType =
        finroc::simulation_physx_ogre::engine::Instance().GetTireType(
          "lugv_chain");
      physx::PxConvexMesh* w_mesh = sim.GetMeshFactory().CreateCylinderConvexMesh(wheel.mWidth, wheel.mRadius, 16);
      RRLIB_LOG_PRINT(DEBUG, "Creating wheel_shapes");
      for (unsigned int i = 0; i < number_of_wheels; ++i)
      {
        shapes[i] = px.createShape(
                      physx::PxConvexMeshGeometry(w_mesh),
                      *sim.GetMaterial("lugv_chain"),
                      true);

        shapes[i]->setQueryFilterData(veh_qry_filter_data);
        shapes[i]->setSimulationFilterData(wheel_coll_filter_data);

        wheels_data->setWheelData(i, wheel);
        wheels_data->setTireData(i, tire);
      }
    }
    // setup suspensions
    {
      //Compute the sprung masses of each suspension spring using a helper function.
      physx::PxF32 susp_sprung_masses[number_of_wheels];
      physx::PxVehicleComputeSprungMasses(number_of_wheels, w_center_offsets, com,
                                          mass, 2, susp_sprung_masses);
      for (unsigned int i = 0; i < number_of_wheels; ++i)
        std::cout << "Susp sprung mass: " << susp_sprung_masses[i]
                  << "\n";
      susp.mMaxCompression = 0.005;
      susp.mMaxDroop = 0.005;
      susp.mSpringStrength = 27000;
      susp.mSpringDamperRate = 800;
      susp.mCamberAtMaxCompression = 0.0;
      susp.mCamberAtMaxDroop = 0.0;
      susp.mCamberAtRest = 0.0;
      for (unsigned int i = 0; i < number_of_wheels; ++i)
      {
        susp.mSprungMass = susp_sprung_masses[i];
        wheels_data->setSuspensionData(i, susp);
      }
      physx::PxVec3 susp_travel_dir(0, 0, -1.0);
      physx::PxVec3 w_com[number_of_wheels];
      physx::PxVec3 susp_force_app[number_of_wheels];
      physx::PxVec3 tire_force_app[number_of_wheels];
      for (unsigned int i = 0; i < number_of_wheels; i++)
      {
        w_com[i] = w_center_offsets[i] - com;
        susp_force_app[i] = physx::PxVec3(w_com[i].x, w_com[i].y,
                                          -0.3f);
        tire_force_app[i] = physx::PxVec3(w_com[i].x, w_com[i].y,
                                          -0.3f);
      }

      for (unsigned int i = 0; i < number_of_wheels - 6; i++)
      {
        wheels_data->setSuspTravelDirection(i, susp_travel_dir);
        wheels_data->setWheelCentreOffset(i, w_com[i]);
        wheels_data->setSuspForceAppPointOffset(i,
                                                susp_force_app[i]);
        wheels_data->setTireForceAppPointOffset(i,
                                                tire_force_app[i]);
      }
      // another suspension travel direction for the two wheels in the front
      for (unsigned int i = number_of_wheels - 6; i < number_of_wheels; i++)
      {
        wheels_data->setSuspTravelDirection(i, physx::PxVec3(0.51842, 0.0, -0.855126));
        //wheels_data->setSuspTravelDirection(i, physx::PxVec3(0.7071067, 0.0, -0.7071067));
        wheels_data->setWheelCentreOffset(i, w_com[i]);
        wheels_data->setSuspForceAppPointOffset(i,
                                                susp_force_app[i]);
        wheels_data->setTireForceAppPointOffset(i,
                                                tire_force_app[i]);
      }
    }
    // setup engine
    {
      engine.mPeakTorque = 79.75;
      engine.mMaxOmega = 61.79831789;
      engine.mTorqueCurve.clear();
      // hydraulic engine
      engine.mTorqueCurve.addPair(0.0, 1.0);
      engine.mTorqueCurve.addPair(1.0, 1.0);
      drive_data.setEngineData(engine);
    }
    // setup gears
    {
      gears.mSwitchTime = 0.001f;
      gears.mFinalRatio = 1.0;
      gears.mNbRatios = 3;
      gears.setGearRatio(physx::PxVehicleGearsData::eREVERSE, -3.16);
      gears.setGearRatio(physx::PxVehicleGearsData::eNEUTRAL, 0.0f);
      gears.setGearRatio(physx::PxVehicleGearsData::eFIRST, 3.16);

      drive_data.setGearsData(gears);
      physx::PxVehicleClutchData clutch;
      clutch.mStrength = 10.0f;
      clutch.mAccuracyMode = physx::PxVehicleClutchAccuracyMode::eESTIMATE;
      clutch.mEstimateIterations = 1;
      drive_data.setClutchData(clutch);
    }
    // create arm
    {
      arm_0->userData = nullptr;
      //arm_0->setSolverIterationCounts(8, 8);
      //arm_0->setSleepThreshold(0);
      std::string mat_id;
      physx::PxConvexMesh* mesh = sim.GetMeshFactory().LoadConvexMesh(
                                    dir + "Arm0.obj", mat_id);
      physx::PxShape* shape =
        arm_0->createShape(physx::PxConvexMeshGeometry(mesh),
                           *finroc::simulation_physx_ogre::engine::Instance().GetMaterial(
                             "steel"));
      physx::PxFilterData col;
      col.word0 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_0;
      col.word1 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_0_AGAINST;
      shape->setQueryFilterData(veh_qry_filter_data);
      shape->setSimulationFilterData(col);
      // synchronize shape and actor location
      shape->setLocalPose(
        finroc::simulation_physx_ogre::cvt(
          rrlib::math::tPose3D::Zero() - arm_0_pose));
      physx::PxRigidBodyExt::setMassAndUpdateInertia(*arm_0, 40);
      arm_joint_0 = physx::PxRevoluteJointCreate(px, actor,
                    finroc::simulation_physx_ogre::cvt(arm_joint_0_pose),
                    arm_0,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_0_pose - arm_0_pose));
      arm_joint_0->userData = this;
      arm_joint_0->setConstraintFlag(
        physx::PxConstraintFlag::eVISUALIZATION, true);
      arm_joint_0->setProjectionLinearTolerance(0.1);
      arm_joint_0->setConstraintFlag(
        physx::PxConstraintFlag::ePROJECTION, true);
      arm_joint_0->setLimit(
        physx::PxJointAngularLimitPair(-3.1, 3.1));
      arm_joint_0->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
      arm_joint_0->setDriveForceLimit(800);
      arm_joint_0->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_ENABLED, true);
      arm_joint_0->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_FREESPIN, false);
    }
    {
      //arm_1->setSolverIterationCounts(10, 10);
      arm_1->userData = nullptr;
      //arm_1->setSleepThreshold(0);
      std::string mat_id;
      physx::PxConvexMesh* mesh = sim.GetMeshFactory().LoadConvexMesh(
                                    dir + "Arm1.obj", mat_id);
      physx::PxShape* shape =
        arm_1->createShape(physx::PxConvexMeshGeometry(mesh),
                           *finroc::simulation_physx_ogre::engine::Instance().GetMaterial(
                             "steel"));
      physx::PxFilterData col;
      col.word0 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_0;
      col.word1 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_0_AGAINST;
      shape->setQueryFilterData(veh_qry_filter_data);
      shape->setSimulationFilterData(col);

      shape->setLocalPose(
        finroc::simulation_physx_ogre::cvt(
          rrlib::math::tPose3D::Zero() - arm_1_pose));
      physx::PxRigidBodyExt::setMassAndUpdateInertia(*arm_1, 40);
      arm_joint_1 = physx::PxRevoluteJointCreate(px, arm_0,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_1_pose - arm_0_pose), arm_1,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_1_pose - arm_1_pose));
      arm_joint_1->userData = this;
      arm_joint_1->setConstraintFlag(
        physx::PxConstraintFlag::eVISUALIZATION, true);
      arm_joint_1->setProjectionLinearTolerance(0.1);
      arm_joint_1->setConstraintFlag(
        physx::PxConstraintFlag::ePROJECTION, true);
      arm_joint_1->setLimit(physx::PxJointAngularLimitPair(0, 3.1));
      arm_joint_1->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
      arm_joint_1->setDriveForceLimit(800);
      arm_joint_1->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_ENABLED, true);
      arm_joint_1->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_FREESPIN, false);
    }
    {
      arm_2->userData = nullptr;
      //arm_2->setSolverIterationCounts(8, 8);
      //arm_2->setSleepThreshold(0);
      std::string mat_id;
      physx::PxConvexMesh* mesh = sim.GetMeshFactory().LoadConvexMesh(
                                    dir + "Arm2.obj", mat_id);
      physx::PxShape* shape =
        arm_2->createShape(physx::PxConvexMeshGeometry(mesh),
                           *finroc::simulation_physx_ogre::engine::Instance().GetMaterial(
                             "steel"));

      physx::PxFilterData col;
      col.word0 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_1;
      col.word1 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_1_AGAINST;
      shape->setQueryFilterData(veh_qry_filter_data);
      shape->setSimulationFilterData(col);

      shape->setLocalPose(
        finroc::simulation_physx_ogre::cvt(
          rrlib::math::tPose3D::Zero() - arm_2_pose));
      physx::PxRigidBodyExt::setMassAndUpdateInertia(*arm_2, 10);
      arm_joint_2 = physx::PxRevoluteJointCreate(px, arm_1,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_2_pose - arm_1_pose), arm_2,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_2_pose - arm_2_pose));
      arm_joint_2->userData = this;
      arm_joint_2->setConstraintFlag(
        physx::PxConstraintFlag::eVISUALIZATION, true);
      arm_joint_2->setProjectionLinearTolerance(0.1);
      arm_joint_2->setConstraintFlag(
        physx::PxConstraintFlag::ePROJECTION, true);
      arm_joint_2->setLimit(physx::PxJointAngularLimitPair(0, 3));
      arm_joint_2->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
      arm_joint_2->setDriveForceLimit(800);
      arm_joint_2->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_ENABLED, true);
      arm_joint_2->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_FREESPIN, false);
    }
    {
      arm_3->userData = nullptr;
      //arm_3->setSolverIterationCounts(4, 2);
      //arm_3->setSleepThreshold(0);
      std::string mat_id;
      physx::PxConvexMesh* mesh = sim.GetMeshFactory().LoadConvexMesh(
                                    dir + "Arm3.obj", mat_id);
      physx::PxShape* shape =
        arm_3->createShape(physx::PxConvexMeshGeometry(mesh),
                           *finroc::simulation_physx_ogre::engine::Instance().GetMaterial(
                             "steel"));

      physx::PxFilterData col;
      col.word0 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_2;
      col.word1 = rrlib::physics_simulation_physx::tCollisionFlag::ROBOT_2_AGAINST;
      shape->setQueryFilterData(veh_qry_filter_data);
      shape->setSimulationFilterData(col);

      shape->setLocalPose(
        finroc::simulation_physx_ogre::cvt(
          rrlib::math::tPose3D::Zero() - arm_3_pose));
      physx::PxRigidBodyExt::setMassAndUpdateInertia(*arm_3, 5);
      arm_joint_3 = physx::PxRevoluteJointCreate(px, arm_2,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_3_pose - arm_2_pose), arm_3,
                    finroc::simulation_physx_ogre::cvt(
                      arm_joint_3_pose - arm_3_pose));
      arm_joint_3->userData = this;
      arm_joint_3->setConstraintFlag(
        physx::PxConstraintFlag::eVISUALIZATION, true);
      arm_joint_3->setProjectionLinearTolerance(0.1);
      arm_joint_3->setConstraintFlag(
        physx::PxConstraintFlag::ePROJECTION, true);
      arm_joint_3->setLimit(physx::PxJointAngularLimitPair(0, 3));
      arm_joint_3->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
      arm_joint_3->setDriveForceLimit(800);
      arm_joint_3->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_ENABLED, true);
      arm_joint_3->setRevoluteJointFlag(
        physx::PxRevoluteJointFlag::eDRIVE_FREESPIN, false);
    }
  }
  // attach wheel shapes to actor
  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::eFRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[0]);
  shapes[0]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::eFRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[1]);
  shapes[1]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[2]);
  shapes[2]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[3]);
  shapes[3]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[4]);
  shapes[4]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[5]);
  shapes[5]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[6]);
  shapes[6]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[7]);
  shapes[7]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[8]);
  shapes[8]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[9]);
  shapes[9]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[10]);
  shapes[10]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[11]);
  shapes[11]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[12]);
  shapes[12]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[13]);
  shapes[13]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[14]);
  shapes[14]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[15]);
  shapes[15]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_LEFT,
    actor->getNbShapes());
  actor->attachShape(*shapes[16]);
  shapes[16]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  wheels_data->setWheelShapeMapping(
    physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_RIGHT,
    actor->getNbShapes());
  actor->attachShape(*shapes[17]);
  shapes[17]->setLocalPose(physx::PxTransform(physx::PxIdentity));

  actor->setMass(chassis_data.mMass);
  actor->setMassSpaceInertiaTensor(chassis_data.mMOI);
  actor->setCMassLocalPose(
    physx::PxTransform(chassis_data.mCMOffset,
                       physx::PxQuat(physx::PxIdentity)));
  //actor->setSleepThreshold(0);

  //actor->setSolverIterationCounts(4,2);

  //wheels_data->setSubStepCount(3,3, 2);
  //wheels_data->setMinLongSlipDenominator(2);

  //wheels_data->setSubStepCount(5.0f, 3, 1);
  vehicle = physx::PxVehicleDriveTank::allocate(number_of_wheels);
  vehicle->setup(&px, actor, *wheels_data, drive_data, number_of_wheels);
  //wheels_data->free();
  sim.RegisterVehicle(vehicle);
  //Don't forget to add the actor to the scene.
  {
    physx::PxSceneWriteLock scopedLock(*px_scene);
    //px_scene->addActor(*actor);
    /*px_scene->addActor(*arm_0);
    px_scene->addActor(*arm_1);
    px_scene->addActor(*arm_2);
    px_scene->addActor(*arm_3);*/
  }
  vehicle->mDriveDynData.setUseAutoGears(true);
  vehicle->setDriveModel(physx::PxVehicleDriveTankControlModel::eSPECIAL);
  WaitForInitialization();
  sim.RegisterPreTick(this);
  sim.RegisterPostTick(this);
  Reset();
  RRLIB_LOG_PRINT(DEBUG, "SUGV::Created");
  phy_initialized = true;
}

//----------------------------------------------------------------------
// mIcarusSUGV destructor
//----------------------------------------------------------------------
mIcarusSUGV::~mIcarusSUGV()
{
  vis_initialized = false;
  phy_initialized = false;
  request_mutex.lock();
  RRLIB_LOG_PRINT(DEBUG, "Releasing Element");
  tPhyElement::engine.UnregisterVehicle(vehicle);
  std::unique_lock<std::mutex> lk(request_finished_mutex);
  tOgreElement::engine.ReleaseOgreElement(this);
  request_finished_cv.wait(lk, [this]()->bool {return pending_requests < 1;});
  request_mutex.unlock();
}

//----------------------------------------------------------------------
// mIcarusSUGV OnParameterChange
//----------------------------------------------------------------------
void mIcarusSUGV::OnParameterChange()
{
  if (!rrlib::ogre::elements::tOgreElement::vis_initialized.load())
  {
    std::unique_lock<std::mutex> lk(request_mutex);
    request_finished_cv.wait(lk);
  }
  if (par_pose.HasChanged())
  {
    this->Reset();
    par_pose.ResetChanged();
  }
}

//----------------------------------------------------------------------
// mIcarusSUGV Update
//----------------------------------------------------------------------
void mIcarusSUGV::Update()
{
  if (in_reset.Get())
  {
    this->Reset();
  }

  if (phy_initialized.load())
  {
    px_scene->lockRead();
    physx::PxVec3 vel = actor->getLinearVelocity();
    out_linear_velocity.Publish(rrlib::math::tVec3d(vel.x, vel.y, vel.z));
    out_forwards_speed.Publish(vehicle->computeForwardSpeed());

    vel = actor->getAngularVelocity();
    out_angular_velocity.Publish(rrlib::math::tVec3d(vel.x, vel.y, vel.z));
    out_chassis_pose.Publish(finroc::simulation_physx_ogre::cvt(actor->getGlobalPose()));

    arm_joint_0_angle = arm_joint_0->getAngle();
    arm_joint_1_angle = arm_joint_1->getAngle();
    arm_joint_2_angle = arm_joint_2->getAngle();
    arm_joint_3_angle = arm_joint_3->getAngle();

    out_left_track_speed.Publish(vehicle->mWheelsDynData.getWheelRotationSpeed(
                                   physx::PxVehicleDriveTankWheelOrder::eFRONT_LEFT));
    out_right_track_speed.Publish(vehicle->mWheelsDynData.getWheelRotationSpeed(
                                    physx::PxVehicleDriveTankWheelOrder::eFRONT_RIGHT));
    px_scene->unlockRead();

    out_joint_0_angle.Publish(arm_joint_0_angle);
    out_joint_1_angle.Publish(arm_joint_1_angle);
    out_joint_2_angle.Publish(arm_joint_2_angle);
    out_joint_3_angle.Publish(arm_joint_3_angle);
  }
}

void mIcarusSUGV::PostTick(double time_step)
{
  if (!IsInitialized())
    return;

  px_scene->lockRead();
  finroc::simulation_physx_ogre::cvt(actor->getGlobalPose(), c_v, c_q);

  physx::PxShape* shape_buffer[number_of_wheels];
  actor->getShapes(shape_buffer, number_of_wheels, 3);
  //actor->getShapes(shape_buffer, number_of_wheels, 1);

  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::eFRONT_LEFT]->getLocalPose(), l0_v, l0_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::eFRONT_RIGHT]->getLocalPose(), r0_v, r0_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_LEFT]->getLocalPose(), l1_v, l1_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_RIGHT]->getLocalPose(), r1_v, r1_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_LEFT]->getLocalPose(), l2_v, l2_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_RIGHT]->getLocalPose(), r2_v, r2_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_LEFT]->getLocalPose(), l3_v, l3_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_RIGHT]->getLocalPose(), r3_v, r3_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_LEFT]->getLocalPose(), l4_v, l4_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_RIGHT]->getLocalPose(), r4_v, r4_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_LEFT]->getLocalPose(), l5_v, l5_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_RIGHT]->getLocalPose(), r5_v, r5_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_LEFT]->getLocalPose(), l6_v, l6_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_RIGHT]->getLocalPose(), r6_v, r6_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_LEFT]->getLocalPose(), l7_v, l7_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_RIGHT]->getLocalPose(), r7_v, r7_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_LEFT]->getLocalPose(), l8_v, l8_q);
  finroc::simulation_physx_ogre::cvt(shape_buffer[physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_RIGHT]->getLocalPose(), r8_v, r8_q);

  finroc::simulation_physx_ogre::cvt(arm_0->getGlobalPose(), a0_v, a0_q);
  finroc::simulation_physx_ogre::cvt(arm_1->getGlobalPose(), a1_v, a1_q);
  finroc::simulation_physx_ogre::cvt(arm_2->getGlobalPose(), a2_v, a2_q);
  finroc::simulation_physx_ogre::cvt(arm_3->getGlobalPose(), a3_v, a3_q);
  px_scene->unlockRead();

  finroc::simulation_physx_ogre::engine::Instance().UpdateOgreElement(this);

}

void mIcarusSUGV::PostTick(const physx::PxVehicleWheelQueryResult& result)
{
  if (!IsInitialized())
    return;
  bool ia[number_of_wheels] =
  {
    result.wheelQueryResults[0].isInAir,
    result.wheelQueryResults[1].isInAir,
    result.wheelQueryResults[2].isInAir,
    result.wheelQueryResults[3].isInAir,
    result.wheelQueryResults[4].isInAir,
    result.wheelQueryResults[5].isInAir,
    result.wheelQueryResults[6].isInAir,
    result.wheelQueryResults[7].isInAir,
    result.wheelQueryResults[8].isInAir,
    result.wheelQueryResults[9].isInAir,
    result.wheelQueryResults[10].isInAir,
    result.wheelQueryResults[11].isInAir,
    result.wheelQueryResults[12].isInAir,
    result.wheelQueryResults[13].isInAir,
    result.wheelQueryResults[14].isInAir,
    result.wheelQueryResults[15].isInAir,
    result.wheelQueryResults[16].isInAir,
    result.wheelQueryResults[17].isInAir
  };
  out_in_air.Publish(ia[0] && ia[1] && ia[2] && ia[3] && ia[4] && ia[5] && ia[6] && ia[7]);

  if (!ia[physx::PxVehicleDriveTankWheelOrder::eFRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::eFRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_LEFT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_LEFT])
  {
    out_left_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_LEFT].tireFriction);
  }
  else
  {
    out_left_track_friction.Publish(0.0);
  }

  if (!ia[physx::PxVehicleDriveTankWheelOrder::eFRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::eFRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e1ST_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e2ND_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e3RD_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e4TH_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e5TH_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e6TH_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e7TH_FROM_FRONT_RIGHT].tireFriction);
  }
  else if (!ia[physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_RIGHT])
  {
    out_right_track_friction.Publish(result.wheelQueryResults[physx::PxVehicleDriveTankWheelOrder::e8TH_FROM_FRONT_RIGHT].tireFriction);
  }
  else
  {
    out_right_track_friction.Publish(0.0);
  }
}

void mIcarusSUGV::PreTick(double time_step)
{
  #pragma omp sections
  {
    #pragma omp section
    {
      if (in_accel.HasChanged())
      {
        raw_input.setAnalogAccel(in_accel.Get());
        in_accel.ResetChanged();
      }
      if (in_thrust_left.HasChanged())
      {
        raw_input.setAnalogLeftThrust(in_thrust_left.Get());
        in_thrust_left.ResetChanged();
      }
      if (in_thrust_right.HasChanged())
      {
        raw_input.setAnalogRightThrust(in_thrust_right.Get());
        in_thrust_right.ResetChanged();
      }
      if (in_brake_left.HasChanged())
      {
        raw_input.setAnalogLeftBrake(in_brake_left.Get());
        in_brake_left.ResetChanged();
      }
      if (in_brake_right.HasChanged())
      {
        raw_input.setAnalogRightBrake(in_brake_right.Get());
        in_brake_right.ResetChanged();
      }
    }
//-------------------------------------------------------------------------------------
    #pragma omp section
    {
      if (in_arm_0_angular_vel.HasChanged())
      {
        if (in_arm_0_angular_vel.Get() > -0.01 && in_arm_0_angular_vel.Get() < 0.01)
        {
          arm_joint_0_set = arm_joint_0_angle;
          arm_joint_0_set_i = 0;
          arm_0_hold = true;
        }
        else
        {
          arm_0_hold = false;
        }
        arm_joint_0_v = in_arm_0_angular_vel.Get();
        in_arm_0_angular_vel.ResetChanged();
      }
      else  {
        if (arm_0_hold)
        {
          double e = arm_joint_0_set - arm_joint_0_angle;
          arm_joint_0_set_i += e;
          arm_joint_0_v = (2 * e + 0.01 * arm_joint_0_set_i);
        }
      }
      //-------------------------------------------------------------------------------------
      if (in_arm_1_angular_vel.HasChanged())
      {
        if (in_arm_1_angular_vel.Get() > -0.01 && in_arm_1_angular_vel.Get() < 0.01)
        {
          arm_joint_1_set = arm_joint_1_angle;
          arm_joint_1_set_i = 0;
          arm_1_hold = true;
        }
        else
        {
          arm_1_hold = false;
        }
        arm_joint_1_v = (in_arm_1_angular_vel.Get());
        in_arm_1_angular_vel.ResetChanged();
      }
      else  {
        if (arm_1_hold)
        {
          double e = arm_joint_1_set - arm_joint_1_angle;
          arm_joint_1_set_i += e;
          arm_joint_1_v = (2 * e + 0.01 * arm_joint_1_set_i);
        }
      }
    }
//-------------------------------------------------------------------------------------
    #pragma omp section
    {
      if (in_arm_2_angular_vel.HasChanged())
      {
        if (in_arm_2_angular_vel.Get() > -0.01 && in_arm_2_angular_vel.Get() < 0.01)
        {
          arm_joint_2_set = arm_joint_2_angle;
          arm_joint_2_set_i = 0;
          arm_2_hold = true;
        }
        else
        {
          arm_2_hold = false;
        }
        arm_joint_2_v = (in_arm_2_angular_vel.Get());
        in_arm_2_angular_vel.ResetChanged();
      }
      else  {
        if (arm_2_hold)
        {
          double e = arm_joint_2_set - arm_joint_2_angle;
          arm_joint_2_set_i += e;
          arm_joint_2_v = (2 * e + 0.01 * arm_joint_2_set_i);
        }
      }
      //-------------------------------------------------------------------------------------
      if (in_arm_3_angular_vel.HasChanged())
      {
        if (in_arm_3_angular_vel.Get() > -0.01 && in_arm_3_angular_vel.Get() < 0.01)
        {
          arm_joint_3_set = arm_joint_3_angle;
          arm_joint_3_set_i = 0;
          arm_3_hold = true;
        }
        else
        {
          arm_3_hold = false;
        }
        arm_joint_3_v = (in_arm_3_angular_vel.Get());
        in_arm_3_angular_vel.ResetChanged();
      }
      else  {
        if (arm_3_hold)
        {
          double e = arm_joint_3_set - arm_joint_3_angle;
          arm_joint_3_set_i += e;
          arm_joint_3_v = (2 * e + 0.01 * arm_joint_3_set_i);
        }
      }
    }
  }

  px_scene->lockWrite();
  arm_joint_0->setDriveVelocity(arm_joint_0_v);
  //RRLIB_LOG_PRINT(DEBUG, "arm_joint_1 set drive velocity: ", arm_joint_1_v);
  arm_joint_1->setDriveVelocity(arm_joint_1_v);
  arm_joint_2->setDriveVelocity(arm_joint_2_v);
  arm_joint_3->setDriveVelocity(arm_joint_3_v);
  physx::PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs(smoothing_data, raw_input, time_step, *vehicle);
  px_scene->unlockWrite();

}

void mIcarusSUGV::InitOgreElement()
{
  Ogre::SceneManager &sm = *tOgreElement::engine.GetOgreSceneManager();
  Ogre::ResourceGroupManager &rgm = Ogre::ResourceGroupManager::getSingleton();

  rgm.createResourceGroup(resource_group_name, false);
  rgm.addResourceLocation(dir + "bitmap", "FileSystem", resource_group_name, false);
  rgm.addResourceLocation(dir + "program", "FileSystem", resource_group_name, false);
  rgm.addResourceLocation(dir + "material", "FileSystem", resource_group_name, false);
  rgm.addResourceLocation(dir + "mesh", "FileSystem", resource_group_name, false);
  rgm.addResourceLocation(dir, "FileSystem", resource_group_name, false);
  rgm.initialiseResourceGroup(resource_group_name);

  chassis_node = sm.getRootSceneNode()->createChildSceneNode(this->GetName());
//  wheel_fl_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_fl");
//  wheel_fr_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_fr");
//  wheel_ml_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_ml");
//  wheel_mr_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_mr");
//  wheel_rl_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_rl");
//  wheel_rr_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_rr");

  wheel_0l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_0l");
  wheel_0r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_0r");
  wheel_1l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_1l");
  wheel_1r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_1r");
  wheel_2l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_2l");
  wheel_2r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_2r");
  wheel_3l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_3l");
  wheel_3r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_3r");
  wheel_4l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_4l");
  wheel_4r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_4r");
//  wheel_5l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_5l");
//  wheel_5r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_5r");
//  wheel_6l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_6l");
//  wheel_6r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_6r");
//  wheel_7l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_7l");
//  wheel_7r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_7r");
//  wheel_8l_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_8l");
//  wheel_8r_node = chassis_node->createChildSceneNode(this->GetName() + "_wheel_8r");

  Ogre::Entity *entity;
  RRLIB_LOG_PRINT(DEBUG, "Creating chassis mesh with name ", tModule::GetName());
  entity = sm.createEntity(tModule::GetName(), "Chassis.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("steel-gray", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  //chassis_node->yaw(Ogre::Radian(M_PI));
  chassis_node->attachObject(entity);

#ifdef DEBUG_RENDER
  {
    Ogre::SceneNode *sn;
    entity = sm.createEntity(this->GetName() + "_chassis_X", "X.mesh", "DEBUG");
    entity->setVisible(true);
    sn = chassis_node->createChildSceneNode(this->GetName() + "_chassis_X");
    sn->attachObject(entity);
    sn->setOrientation(-0.707107, 0, -0.707107, 0);
    entity = sm.createEntity(this->GetName() + "_chassis_Y", "Y.mesh", "DEBUG");
    entity->setVisible(true);
    sn = chassis_node->createChildSceneNode(this->GetName() + "_chassis_Y");
    sn->attachObject(entity);
    sn->setOrientation(-0.707107, -0.707107, 0, 0);
    entity = sm.createEntity(this->GetName() + "_chassis_Z", "Z.mesh", "DEBUG");
    entity->setVisible(true);
    sn = chassis_node->createChildSceneNode(this->GetName() + "_chassis_Z");
    sn->attachObject(entity);
    sn->setOrientation(0, 0, 0, 1);
  }
#endif


  entity = sm.createEntity(this->GetName() + "_wheel_0l", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_0l_node->attachObject(entity);
  entity = sm.createEntity(this->GetName() + "_wheel_0r", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_0r_node->attachObject(entity);

  entity = sm.createEntity(this->GetName() + "_wheel_1l", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_1l_node->attachObject(entity);
  entity = sm.createEntity(this->GetName() + "_wheel_1r", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_1r_node->attachObject(entity);

  entity = sm.createEntity(this->GetName() + "_wheel_2l", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_2l_node->attachObject(entity);
  entity = sm.createEntity(this->GetName() + "_wheel_2r", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_2r_node->attachObject(entity);

  entity = sm.createEntity(this->GetName() + "_wheel_3l", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_3l_node->attachObject(entity);
  entity = sm.createEntity(this->GetName() + "_wheel_3r", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_3r_node->attachObject(entity);

  entity = sm.createEntity(this->GetName() + "_wheel_4l", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_4l_node->attachObject(entity);
  entity = sm.createEntity(this->GetName() + "_wheel_4r", "Tire26x11.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  wheel_4r_node->attachObject(entity);

//  entity = sm.createEntity(this->GetName() + "_wheel_5l", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_5l_node->attachObject(entity);
//  entity = sm.createEntity(this->GetName() + "_wheel_5r", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_5r_node->attachObject(entity);
//
//  entity = sm.createEntity(this->GetName() + "_wheel_6l", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_6l_node->attachObject(entity);
//  entity = sm.createEntity(this->GetName() + "_wheel_6r", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_6r_node->attachObject(entity);
//
//  entity = sm.createEntity(this->GetName() + "_wheel_7l", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_7l_node->attachObject(entity);
//  entity = sm.createEntity(this->GetName() + "_wheel_7r", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_7r_node->attachObject(entity);
//
//  entity = sm.createEntity(this->GetName() + "_wheel_8l", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_8l_node->attachObject(entity);
//  entity = sm.createEntity(this->GetName() + "_wheel_8r", "Tire26x11.mesh", resource_group_name);
//  entity->getSubEntity(0)->setMaterialName("GatorXUVTire", resource_group_name);
//  entity->getSubEntity(1)->setMaterialName("GatorXUVWheel", resource_group_name);
//  entity->setCastShadows(true);
//  entity->setVisible(true);
//  wheel_8r_node->attachObject(entity);

  Ogre::Vector3 v;
  Ogre::Quaternion q;

  sn_arm_0 = sm.getRootSceneNode()->createChildSceneNode(this->GetName() + "_arm_0");
#ifdef DEBUG_RENDER
  {
    Ogre::SceneNode *sn;
    entity = sm.createEntity(this->GetName() + "_arm_0_X", "X.mesh", "DEBUG");
    entity->setVisible(true);
    sn = sn_arm_0->createChildSceneNode(this->GetName() + "_arm_0_X");
    sn->attachObject(entity);
    sn->setOrientation(-0.707107, 0, -0.707107, 0);
    entity = sm.createEntity(this->GetName() + "_arm_0_Y", "Y.mesh", "DEBUG");
    entity->setVisible(true);
    sn = sn_arm_0->createChildSceneNode(this->GetName() + "_arm_0_Y");
    sn->attachObject(entity);
    sn->setOrientation(-0.707107, -0.707107, 0, 0);
    entity = sm.createEntity(this->GetName() + "_arm_0_Z", "Z.mesh", "DEBUG");
    entity->setVisible(true);
    sn = sn_arm_0->createChildSceneNode(this->GetName() + "_arm_0_Z");
    sn->attachObject(entity);
    sn->setOrientation(0, 0, 0, 1);
  }
#endif
  entity = sm.createEntity(this->GetName() + "_arm_0", "Arm0.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("steel-gray", resource_group_name);
  entity->setCastShadows(false);
  entity->setVisible(true);
  sn_arm_0->attachObject(entity);


  sn_arm_1 = sm.getRootSceneNode()->createChildSceneNode(this->GetName() + "_arm_1");
#ifdef DEBUG_RENDER
  {
    Ogre::SceneNode *sn;
    entity = sm.createEntity(this->GetName() + "_arm_1_X", "X.mesh", "DEBUG");
    entity->setVisible(true);
    sn = sn_arm_1->createChildSceneNode(this->GetName() + "_arm_1_X");
    sn->attachObject(entity);
    sn->setOrientation(-0.707107, 0, -0.707107, 0);
    entity = sm.createEntity(this->GetName() + "_arm_1_Y", "Y.mesh", "DEBUG");
    entity->setVisible(true);
    sn = sn_arm_1->createChildSceneNode(this->GetName() + "_arm_1_Y");
    sn->attachObject(entity);
    sn->setOrientation(-0.707107, -0.707107, 0, 0);
    entity = sm.createEntity(this->GetName() + "_arm_1_Z", "Z.mesh", "DEBUG");
    entity->setVisible(true);
    sn = sn_arm_1->createChildSceneNode(this->GetName() + "_arm_1_Z");
    sn->attachObject(entity);
    sn->setOrientation(0, 0, 0, 1);
  }
#endif
  entity = sm.createEntity(this->GetName() + "_arm_1", "Arm1.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("steel-gray", resource_group_name);
  entity->setCastShadows(false);
  entity->setVisible(true);
  sn_arm_1->attachObject(entity);


  entity = sm.createEntity(this->GetName() + "_arm_2", "Arm2.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("steel-gray", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  sn_arm_2 = sm.getRootSceneNode()->createChildSceneNode(this->GetName() + "_arm_2");
  sn_arm_2->attachObject(entity);

  entity = sm.createEntity(this->GetName() + "_arm_3", "Arm3.mesh", resource_group_name);
  entity->getSubEntity(0)->setMaterialName("steel-gray", resource_group_name);
  entity->setCastShadows(true);
  entity->setVisible(true);
  sn_arm_3 = sm.getRootSceneNode()->createChildSceneNode(this->GetName() + "_arm_3");
  sn_arm_3->attachObject(entity);

  vehicle_cam = sm.createCamera(name + "_cam");
  vehicle_cam->setNearClipDistance(0.1);
  vehicle_cam->setFarClipDistance(200);
  vehicle_cam->setPosition(0, 0, 0);
  vehicle_cam->setOrientation(Ogre::Quaternion(0.5, 0.5, -0.5, -0.5));
  auto cam_node = chassis_node->createChildSceneNode(name + "_cam_node");
  cam_node->attachObject(vehicle_cam);
  cam_node->setPosition(5, -5, 3);
  cam_node->setDirection(chassis_node->getPosition(), Ogre::Node::TransformSpace::TS_LOCAL,
                         Ogre::Vector3::UNIT_X);
  //cam_node->setAutoTracking(true, chassis_node);
  cam_pose_wrapper = new rrlib::ogre::elements::tPoseWrapper(tOgreElement::engine, cam_node);
  vehicle_cam->Ogre::Renderable::setUserAny(Ogre::Any(cam_pose_wrapper));
  finroc::simulation_physx_ogre::engine::Instance().RegisterCustomCamera(vehicle_cam);

  RRLIB_LOG_PRINT(DEBUG, "OGRE element successfully initialized");
  rrlib::ogre::elements::tOgreElement::vis_initialized = true;
}

void mIcarusSUGV::UpdateOgreElement()
{
  if (IsInitialized())
  {
    chassis_node->setPosition(c_v);
    chassis_node->setOrientation(c_q);

    wheel_0l_node->setPosition(l0_v);
    wheel_0l_node->setOrientation(l0_q);

    wheel_0r_node->setPosition(r0_v);
    wheel_0r_node->setOrientation(r0_q);

    wheel_1l_node->setPosition(l2_v);
    wheel_1l_node->setOrientation(l2_q);

    wheel_1r_node->setPosition(r2_v);
    wheel_1r_node->setOrientation(r2_q);

    wheel_2l_node->setPosition(l4_v);
    wheel_2l_node->setOrientation(l4_q);

    wheel_2r_node->setPosition(r4_v);
    wheel_2r_node->setOrientation(r4_q);

    wheel_3l_node->setPosition(l6_v);
    wheel_3l_node->setOrientation(l6_q);

    wheel_3r_node->setPosition(r6_v);
    wheel_3r_node->setOrientation(r6_q);

    wheel_4l_node->setPosition(l8_v);
    wheel_4l_node->setOrientation(l8_q);

    wheel_4r_node->setPosition(r8_v);
    wheel_4r_node->setOrientation(r8_q);

//    wheel_5l_node->setPosition(l5_v);
//    wheel_5l_node->setOrientation(l5_q);
//
//    wheel_5r_node->setPosition(r5_v);
//    wheel_5r_node->setOrientation(r5_q);
//
//    wheel_6l_node->setPosition(l6_v);
//    wheel_6l_node->setOrientation(l6_q);
//
//    wheel_6r_node->setPosition(r6_v);
//    wheel_6r_node->setOrientation(r6_q);
//
//    wheel_7l_node->setPosition(l7_v);
//    wheel_7l_node->setOrientation(l7_q);
//
//    wheel_7r_node->setPosition(r7_v);
//    wheel_7r_node->setOrientation(r7_q);
//
//    wheel_8l_node->setPosition(l8_v);
//    wheel_8l_node->setOrientation(l8_q);
//
//    wheel_8r_node->setPosition(r8_v);
//    wheel_8r_node->setOrientation(r8_q);

    /*sn_arm_0->setPosition(a0_v);
    sn_arm_0->setOrientation(a0_q);
    sn_arm_1->setPosition(a1_v);
    sn_arm_1->setOrientation(a1_q);
    sn_arm_2->setPosition(a2_v);
    sn_arm_2->setOrientation(a2_q);
    sn_arm_3->setPosition(a3_v);
    sn_arm_3->setOrientation(a3_q);*/
  }
}

void mIcarusSUGV::ReleaseOgreElement()
{
  RRLIB_LOG_PRINT(DEBUG, "Releasing Ogre Element");
  Ogre::SceneManager &sm = *tOgreElement::engine.GetOgreSceneManager();
  Ogre::Entity *e = nullptr;

  e = reinterpret_cast<Ogre::Entity *>(wheel_0l_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_0l_node);
  sm.destroyEntity(e);
  e = reinterpret_cast<Ogre::Entity *>(wheel_0r_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_0r_node);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(wheel_1l_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_1l_node);
  sm.destroyEntity(e);
  e = reinterpret_cast<Ogre::Entity *>(wheel_1r_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_1r_node);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(wheel_2l_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_2l_node);
  sm.destroyEntity(e);
  e = reinterpret_cast<Ogre::Entity *>(wheel_2r_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_2r_node);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(wheel_3l_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_3l_node);
  sm.destroyEntity(e);
  e = reinterpret_cast<Ogre::Entity *>(wheel_3r_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_3r_node);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(wheel_4l_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_4l_node);
  sm.destroyEntity(e);
  e = reinterpret_cast<Ogre::Entity *>(wheel_4r_node->getAttachedObject(0));
  sm.destroySceneNode(wheel_4r_node);
  sm.destroyEntity(e);

//  e = reinterpret_cast<Ogre::Entity *>(wheel_5l_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_5l_node);
//  sm.destroyEntity(e);
//  e = reinterpret_cast<Ogre::Entity *>(wheel_5r_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_5r_node);
//  sm.destroyEntity(e);
//
//  e = reinterpret_cast<Ogre::Entity *>(wheel_6l_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_6l_node);
//  sm.destroyEntity(e);
//  e = reinterpret_cast<Ogre::Entity *>(wheel_6r_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_6r_node);
//  sm.destroyEntity(e);
//
//  e = reinterpret_cast<Ogre::Entity *>(wheel_7l_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_7l_node);
//  sm.destroyEntity(e);
//  e = reinterpret_cast<Ogre::Entity *>(wheel_7r_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_7r_node);
//  sm.destroyEntity(e);
//
//  e = reinterpret_cast<Ogre::Entity *>(wheel_8l_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_8l_node);
//  sm.destroyEntity(e);
//  e = reinterpret_cast<Ogre::Entity *>(wheel_8r_node->getAttachedObject(0));
//  sm.destroySceneNode(wheel_8r_node);
//  sm.destroyEntity(e);

  /*e = reinterpret_cast<Ogre::Entity *>(sn_arm_3->getAttachedObject(0));
  sm.destroySceneNode(sn_arm_3);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(sn_arm_2->getAttachedObject(0));
  sm.destroySceneNode(sn_arm_2);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(sn_arm_1->getAttachedObject(0));
  sm.destroySceneNode(sn_arm_1);
  sm.destroyEntity(e);

  e = reinterpret_cast<Ogre::Entity *>(sn_arm_0->getAttachedObject(0));
  sm.destroySceneNode(sn_arm_0);
  sm.destroyEntity(e);*/

  e = reinterpret_cast<Ogre::Entity *>(chassis_node->getAttachedObject(0));
  sm.destroySceneNode(chassis_node);
  sm.destroyEntity(e);

  if (cam_pose_wrapper != nullptr)
  {
    finroc::simulation_physx_ogre::engine::Instance().UnregisterCustomCamera(vehicle_cam);
    vehicle_cam->detachFromParent();
    sm.destroyCamera(vehicle_cam);
    vehicle_cam = nullptr;
    auto n = cam_pose_wrapper->GetSceneNode();
    delete(cam_pose_wrapper);
    cam_pose_wrapper = nullptr;
    sm.destroySceneNode(n);
  }

  finroc::simulation_physx_ogre::engine::Instance().DestroySceneNode(chassis_node);
}

Ogre::SceneNode* mIcarusSUGV::GetChassisNode()
{
  return chassis_node;
}

void mIcarusSUGV::Reset()
{
  RRLIB_LOG_PRINT(DEBUG, "Resetting vehicle");
  vehicle->mDriveDynData.forceGearChange(physx::PxVehicleGearsData::eNEUTRAL);
  physx::PxTransform t(finroc::simulation_physx_ogre::cvt(*par_pose.GetPointer()));
  physx::PxSceneWriteLock scopedLock(*px_scene);
  vehicle->setToRestState();
  px_scene->removeActor(*actor);
  /*px_scene->removeActor(*arm_0);
  px_scene->removeActor(*arm_1);
  px_scene->removeActor(*arm_2);
  px_scene->removeActor(*arm_3);*/
  actor->setGlobalPose(t);
  arm_0->setGlobalPose(t.transform(finroc::simulation_physx_ogre::cvt(arm_0_pose)));
  arm_1->setGlobalPose(t.transform(finroc::simulation_physx_ogre::cvt(arm_1_pose)));
  arm_2->setGlobalPose(t.transform(finroc::simulation_physx_ogre::cvt(arm_2_pose)));
  arm_3->setGlobalPose(t.transform(finroc::simulation_physx_ogre::cvt(arm_3_pose)));
  px_scene->addActor(*actor);
  /*px_scene->addActor(*arm_0);
  px_scene->addActor(*arm_1);
  px_scene->addActor(*arm_2);
  px_scene->addActor(*arm_3);*/
  raw_input.setAnalogLeftBrake(1);
  raw_input.setAnalogRightBrake(1);
}

bool mIcarusSUGV::IsInitialized() const
{
  return rrlib::ogre::elements::tOgreElement::vis_initialized.load() && phy_initialized.load();
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
