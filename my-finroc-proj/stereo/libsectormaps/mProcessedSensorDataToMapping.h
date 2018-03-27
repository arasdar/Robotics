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
/*!\file    projects/icarus/simulation/mProcessedSensorDataToMapping.h
 *
 * \author  Aras Dargazany
 *
 * \date    2013-07-23
 *
 * \brief Contains mProcessedSensorDataToMapping
 *
 * \b mProcessedSensorDataToMapping
 *
 * In this module we try to map the already processed sensor data such as rear and fron laser scanner, actuated 3d laser scanner and stero vision.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__simulation__mProcessedSensorDataToMapping_h__
#define __projects__icarus__simulation__mProcessedSensorDataToMapping_h__

#include "plugins/structure/tSenseControlModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/mapping/definitions.h"
#include "rrlib/canvas/tCanvas2D.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "projects/icarus/mapping/tSectorMap.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace simulation_physx_ogre
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef mapping::tSectorMap<rrlib::mapping::state_space::tPolar, 2> tSectorMapPolar2D;
typedef mapping::tSectorMap<rrlib::mapping::state_space::tCartesian, 2> tSectorMapCartesian2D;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * In this module we try to map the already processed sensor data such as rear and fron laser scanner, actuated 3d laser scanner and stero vision.
 */
class mProcessedSensorDataToMapping : public structure::tSenseControlModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

  /* -----------> seclet map <---------------*/
  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data_front_scanner;

  /*! reseting the grid map for environment*/
  tControllerInput<double> ci_reset_gridmap_shift;
  tControllerInput<bool> ci_sectormap_cart, ci_sectormap_cart_2, ci_sectormap_polar_2;

  /*! sector map stuff*/
  tSensorOutput<rrlib::mapping::tMapGridCartesian2DShiftable<double>> so_gridmap_shift;
  tSensorInput<rrlib::canvas::tCanvas2D> si_canvas_gridmap_shift;
  tSensorOutput<rrlib::canvas::tCanvas2D> so_canvas_gridmap_shift;
  tSensorOutput<int> so_gridmap_number_obst_total;

  tSensorInput<rrlib::math::tPose3D> si_robot_pose;


//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mProcessedSensorDataToMapping(core::tFrameworkElement *parent, const std::string &name = "ProcessedSensorDataToMapping");

  void GridmapShiftable();
  void DrawRobot(rrlib::canvas::tCanvas2D& canvas);
  void DrawSectorPolarTestSecond(rrlib::canvas::tCanvas2D& canvas, tSectorMapPolar2D& sectormap_polar);
  void dist_polar_second(tSectorMapPolar2D& sectormap_polar, rrlib::math::tVec2d position_obst);
  void dist_cart(tSectorMapCartesian2D& sectormap_cartesian, rrlib::math::tVec2d position_obst);
  void dist_cart_second(tSectorMapCartesian2D& sectormap_cartesian, rrlib::math::tVec2d position_obst);
  void DrawSectorCartesianTest(rrlib::canvas::tCanvas2D& canvas, tSectorMapCartesian2D& sectormap_cartesian);
  void DrawSectorPolarTest(rrlib::canvas::tCanvas2D& canvas, tSectorMapPolar2D& sectormap_polar);
  void dist(tSectorMapPolar2D& sectormap_polar, rrlib::math::tVec2d position_obst);
  void DrawSectorCartesianTestSecond(rrlib::canvas::tCanvas2D& canvas, tSectorMapCartesian2D& sectormap_cartesian);


  /*! Destructor  --- for using it in a new thread should be available for the class
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mProcessedSensorDataToMapping();


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*-------- Here is the right place for your variables. Replace this line by your declarations! ---------------*/
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds gridmap_shift_bounds;
  rrlib::mapping::tMapGridCartesian2DShiftable<double> gridmap_shift;

  std::vector<rrlib::math::tVec2d> data_obst_all;
  rrlib::math::tPose3D robot_pose;

  virtual void OnStaticParameterChange();   ////Might be needed to process static parameters. Delete otherwise!
  virtual void OnParameterChange();   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!
  virtual void Sense();
  virtual void Control();

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
