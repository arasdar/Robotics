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
/*!\file    projects/icarus/sensor_processing/libstereo_test/finroc/mStereoToSectorMap.h
 *
 * \author  Aras Dargazany
 *
 * \date    2014-05-07
 *
 * \brief Contains mStereoToSectorMap
 *
 * \b mStereoToSectorMap
 *
 * This module is to generated sector maps and seclet maps from stereo pipeline.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__sensor_processing__libstereo_test__finroc__mStereoToSectorMap_h__
#define __projects__icarus__sensor_processing__libstereo_test__finroc__mStereoToSectorMap_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/canvas/tCanvas2D.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/mapping/tSectorMap.h"

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace stereo_gray
{
namespace offline
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef mapping::tSectorMap<rrlib::mapping::state_space::tPolar, 2> tSectorMapPolar2D;


//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module is to generated sector maps and seclet maps from stereo pipeline.
 */
class mStereoToSectorMap : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  tStaticParameter<double> static_parameter_1;   //Example for a static parameter. Replace or delete it!
//
//  tParameter<double> par_parameter_1;   //Example for a runtime parameter named "Parameter 1". Replace or delete it!
//
//  tInput<double> in_signal_1;   //Example for input ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tInput<double> in_signal_2;
//
//  tOutput<double> out_signal_1;   //Examples for output ports named "Signal 1" and "Signal 2". Replace or delete them!
//  tOutput<double> out_signal_2;

  /*! sector map stuff*/
//  tOutput<rrlib::mapping::tMapGridCartesian2DShiftable<double>> so_gridmap_shift;
  tInput<rrlib::canvas::tCanvas2D> input_canvas_gridmap;
  tOutput<rrlib::canvas::tCanvas2D> output_canvas_gridmap;

  tInput<rrlib::canvas::tCanvas2D> input_canvas_secletmap;
  tOutput<rrlib::canvas::tCanvas2D> output_canvas_secletmap;

  tInput<rrlib::mapping::tMapGridPolar2D<double>> input_secletmap;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mStereoToSectorMap(core::tFrameworkElement *parent, const std::string &name = "StereoToSectorMap");

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  tSectorMapPolar2D* sectormap_polar;

  /*! Destructor
   *
   * The destructor of modules is declared private to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mStereoToSectorMap();

//  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!
//
//  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;

  void gridmapStereo();
  void secletmapStereo();
  void drawSectorPolar(rrlib::canvas::tCanvas2D& canvas, tSectorMapPolar2D& sectormap_polar);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}


#endif
