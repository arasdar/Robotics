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
#include <opencv/cv.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
//#include "projects/icarus/simulation/mStViPr.hpp"
#include "rrlib/mapping/definitions.h"
#include "rrlib/canvas/tCanvas2D.h"
#include "rrlib/distance_data/tDistanceData.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace simulation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

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

  /* -----------> sector map <---------------*/
  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data_front_scanner;
  tSensorOutput<rrlib::mapping::tMapGridPolar2D<double>> so_map_polar_front_scanner;
  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data_rear_scanner;
  tSensorOutput<rrlib::mapping::tMapGridPolar2D<double>> so_map_polar_rear_scanner;

  /*----------> grid map planar laser scanner front and rear <-----------------*/
  tSensorOutput<rrlib::mapping::tMapGridCartesian2D<double>> so_gridmap_planar_scanner;
  tSensorOutput<rrlib::mapping::tMapGridCartesian2D<double>> so_gridmap_planar_scanner_env;
  tSensorInput<rrlib::canvas::tCanvas2D> si_canvas_planar_scanner_env;
  tSensorOutput<rrlib::canvas::tCanvas2D> so_canvas_planar_scanner_env;

  /*
   * grid map for actuated laser scanner
   */
  tSensorInput<rrlib::distance_data::tDistanceData> si_distance_data_actuated_scanner; //front scanner
  tSensorOutput<rrlib::mapping::tMapGridCartesian2D<double>> so_gridmap_actuated_scanner;
  tSensorInput<double> si_angle_actuated;
  tSensorOutput<rrlib::mapping::tMapGridCartesian2D<double>> so_gridmap_actuated_scanner_env;
  tSensorInput<rrlib::canvas::tCanvas2D> si_canvas_actuated_env;
  tSensorOutput<rrlib::canvas::tCanvas2D> so_canvas_actuated_env;

  /*! reseting the grid map for environment*/
  tControllerInput<double> ci_reset_gridmap_actuated_scanner_env;
  tControllerInput<double> ci_reset_gridmap_planar_scanner_env;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mProcessedSensorDataToMapping(core::tFrameworkElement *parent, const std::string &name = "ProcessedSensorDataToMapping");

  void secletmap_planar_scanner_front();
  void secletmap_planar_scanner_rear();

  void gridmap_planar_scanner();
  void gridmap_actuated_scanner();
  void gridmap_stereo();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*-------- Here is the right place for your variables. Replace this line by your declarations! ---------------*/
  rrlib::mapping::tMapGridCartesian2D<double> map_actuated_scanner_env;
  rrlib::mapping::tMapGridCartesian2D<double> map_planar_scanner_env;
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds bounds_planar_scanner_env;

  rrlib::math::tVec3f pose_front_scanner;
  rrlib::math::tVec3f pose_rear_scanner;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mProcessedSensorDataToMapping();

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
