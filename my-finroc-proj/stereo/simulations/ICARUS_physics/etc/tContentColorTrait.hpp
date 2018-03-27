//
// You received this file as part of RRLib
// Robotics Research Library
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
/*!\file    rrlib/mapping/handlers/tContentColorTrait.hpp
 *
 * \author  Michael Arndt
 *
 * \date    2013-07-11
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace mapping
{
namespace handlers
{


//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

// default representation for double values: 0 -> white ... 1 -> black
template <>
inline void tContentColorTrait<double>::GetColor(uint8_t &r, uint8_t &g, uint8_t &b, const double &value)
{
  double v = std::max(std::min(value, 1.), 0.);
  r = g = b = (1 - v) * 255;

  if (value == 1)  //red
  {
    r = 255;
    g = b = 0;
  }
  if (value == 2)  //yellow == blue
  {
    r = g = 0;
    b = 255;
  }
  if (value == 3)  //green
  {
    r = b = 0;
    g = 255;
  }
//  if (value == 4) //white
//  {
//    r = g = b = 255;
//  }
//  if (value == 5) //black
//  {
//    r = g = b = 0;
//  }

}

template <>
inline void tContentColorTrait<double>::GetBackgroundColor(uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = g = b = 255;
}

// default representation for bool values: false -> white, true -> black
template <>
inline void tContentColorTrait<bool>::GetColor(uint8_t &r, uint8_t &g, uint8_t &b, const bool &value)
{
  r = g = b = value ? 0 : 255;
}

template <>
inline void tContentColorTrait<bool>::GetBackgroundColor(uint8_t &r, uint8_t &g, uint8_t &b)
{
  r = g = b = 255;
}




//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

