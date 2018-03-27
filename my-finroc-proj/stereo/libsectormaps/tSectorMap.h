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
/*!\file    projects/icarus/mapping/tSectorMap.h
 *
 * \author  Thomas Pfister
 *
 * \date    2014-01-27
 *
 * \brief   Contains tSectorMap
 *
 * \b tSectorMap
 *
 * The sector map declaration for the ICARUS project.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__mapping__tSectorMap_h__
#define __projects__icarus__mapping__tSectorMap_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/icarus/mapping/tSectorMapBase.h"

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * The sector map declaration for the ICARUS project.
 */
template <template<size_t> class TStateSpace, size_t TDimension>
class tSectorMap : public tSectorMapBase<TStateSpace, TDimension>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tSectorMapBase<TStateSpace, TDimension>::tStateSpace tStateSpace;
  typedef typename tSectorMapBase<TStateSpace, TDimension>::tMapType tMapType;

//  tSectorMap()
//  {}
//
//  virtual ~tSectorMap()
//  {}

  virtual void SetResolution(const typename tStateSpace::tCoordinate &resolution_)
  {
    const typename tStateSpace::tBounds & bounds = this->GetBounds();
    typename tStateSpace::tCoordinate resolution(resolution_);
    resolution[ tStateSpace::dimension - 1] = bounds.upper_bounds[tStateSpace::dimension - 1][tStateSpace::dimension - 1] -
        bounds.lower_bounds[tStateSpace::dimension - 1][tStateSpace::dimension - 1];
    tMapType::SetResolution(resolution);
  }

};


template <size_t TDimension>
class tSectorMap<rrlib::mapping::state_space::tPolar<TDimension, double, rrlib::math::angle::Degree>, TDimension> : public tSectorMapBase<rrlib::mapping::state_space::tPolar<TDimension, double, rrlib::math::angle::Degree>, TDimension>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef typename tSectorMapBase<rrlib::mapping::state_space::tPolar, TDimension>::tStateSpace tStateSpace;
  typedef typename tSectorMapBase<rrlib::mapping::state_space::tPolar, TDimension>::tMapType tMapType;

//  tSectorMap()
//  {}
//
//  virtual ~tSectorMap()
//  {}

  virtual void SetResolution(const typename tStateSpace::tCoordinate &resolution_)
  {
    const typename tStateSpace::tBounds & bounds = this->GetBounds();
    typename tStateSpace::tCoordinate resolution(resolution_);
    resolution.Length() = bounds.upper_bounds[tStateSpace::dimension - 1].Length() -
                          bounds.lower_bounds[tStateSpace::dimension - 1].Length();
    tMapType::SetResolution(resolution);
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
