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
/*!\file    projects/icarus/mapping/tSectorMapBase.h
 *
 * \author  Thomas Pfister, Bernd Helge Schaefer
 *
 * \date    2014-01-27
 *
 * \brief   Contains tSectorMapBase
 *
 * \b tSectorMapBase
 *
 * The sector map declaration for the ICARUS project.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__mapping__tSectorMapBase_h__
#define __projects__icarus__mapping__tSectorMapBase_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/mapping/definitions.h"
#include "rrlib/si_units/si_units.h"

#include "rrlib/logging/messages.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace mapping
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
enum class tSectorStatus
{
  eSECTOR_STATUS_INVALID, /*!< Indicates that the sector in question carries no valid information.*/
  eSECTOR_STATUS_VALID_IRRELEVANT, /*!< Indicates that the sector in question carries valid information which is not relevant with respect to the aspect reflected in the sector map.
               If the sector map was filled by mSectorMapBuilder, the distance of the most distant irrelevant property is stored.
               The Idea is to reflect an estimate of the current viewing distance of the sensor in question.*/
  eSECTOR_STATUS_VALID_RELEVANT,
};

enum class tSectorMapStatus
{
  eSECTOR_MAP_STATUS_IGNORE, /*< indicates that the content of the sector map in question shall be ignored as it is perhaps temporarily deactivated or reflects an aspect that has been defined to ignore all properties. */
  eSECTOR_MAP_STATUS_INVALID, /*< indicates that the content of the sector map in question is not valid at the moment */
  eSECTOR_MAP_STATUS_VALID, /*< indicates that the content of the sector map in question is valid at the moment */
};

enum class tSectorMapPosition
{
  eFRONT_CARTESIAN,
  eREAR_CARTESIAN,
  eFRONT_LEFT_POLAR_BIG,
  eFRONT_RIGHT_POLAR_BIG,
  eREAR_LEFT_POLAR_BIG,
  eREAR_RIGHT_POLAR_BIG,
  eFRONT_LEFT_CARTESIAN,
  eFRONT_RIGHT_CARTESIAN,
  eFRONT_LEFT_POLAR,
  eFRONT_RIGHT_POLAR,
  eREAR_LEFT_CARTESIAN,
  eREAR_RIGHT_CARTESIAN,
  eREAR_LEFT_POLAR,
  eREAR_RIGHT_POLAR,
  eLEFT_REAR_CARTESIAN,
  eLEFT_FRONT_CARTESIAN,
  eRIGHT_REAR_CARTESIAN,
  eRIGHT_FRONT_CARTESIAN
};

typedef struct tSectorData_
{
  rrlib::math::tVec2d obstacle_position;

  //rrlib::si_units::tLength distance_to_obstacle; /*!< Distance to the nearest obstacle in the sector. */
  double distance_to_obstacle = 10; /*!< Distance to the nearest obstacle in the sector. */

  // data from mca2 sector maps
  rrlib::time::tTimestamp timestamp_last_update; /*!< Timestamp of last update. */
  double quality; /*!< Quality of the sector. Values in the range of [0,1]. */
  /*const*/ size_t id; /*!< ID of the sector */
  tSectorStatus status; /*!< Status of the sector. */
} tSectorData;

typedef struct tSectorMapData_
{
  tSectorMapPosition position; /*! */
//  std::string description; /*! */
  rrlib::math::tVec3d origin; /*!< Origin of the sector map. */
  tSectorMapStatus status; /*!< Status of the sector map. */
} tSectorMapData;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * The sector map declaration for the ICARUS project.
 */
template <template<size_t> class TStateSpace, size_t TDimension>
class tSectorMapBase : public rrlib::mapping::tMap<TStateSpace<TDimension>, rrlib::mapping::storage::tGridStorage<tSectorData, TStateSpace<TDimension>>, tSectorData>, public tSectorMapData
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename rrlib::mapping::tMap<TStateSpace<TDimension>, rrlib::mapping::storage::tGridStorage<tSectorData, TStateSpace<TDimension>>, tSectorData> tMapType;
  typedef typename tMapType::tStateSpace tStateSpace;

  tSectorMapBase()
  {}

  virtual ~tSectorMapBase()
  {}

  void SetBounds(const typename tStateSpace::tBounds &bounds)
  {
    tMapType::SetBounds(bounds);
    this->SetResolution(this->GetResolution());
  }

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

// does not work(because out of namespace):
//template< size_t TDimension>
//tSectorMap<rrlib::mapping::state_space::tPolar, TDimension>::SetResolution(const typename rrlib::mapping::state_space::tPolar<TDimension>::tCoordinate &resolution_)
//{
//  typedef typename rrlib::mapping::state_space::tPolar<TDimension> tStateSpace;
//  typedef typename rrlib::mapping::tMap< tStateSpace, rrlib::mapping::storage::tGridStorage<tSectorData, tStateSpace >, tSectorData> tMapType;
//  const typename tStateSpace::tBounds & bounds = this->GetBounds();
//  typename tStateSpace::tCoordinate resolution( resolution_);
//  resolution.Length() = bounds.upper_bounds[tStateSpace::dimension-1].Length() -
//      bounds.lower_bounds[tStateSpace::dimension-1].Length();
//  tMapType::SetResolution( resolution);
//}

//#include "projects/icarus/mapping/tSectorMap.hpp"

#endif
