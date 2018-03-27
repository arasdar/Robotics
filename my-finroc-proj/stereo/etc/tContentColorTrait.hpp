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
/*!\file    rrlib/mapping/handlers/canvas/tContentColorTrait.hpp
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
namespace canvas
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

template <>
void tContentColorTrait<double>::GetColor(int &r, int &g, int &b, const double &value)
{

  if (value < 0)
  {
    r = g = b = 128;
  }
  else if (value > 0.1)
  {
    r = g = b = 0;
  }
  else
  {
    r = g = b = 255;
  }
  if (value == 1)  //green
  {
    r = b = 0;
    g = 155;
  }
  if (value == 2)  //yellow
  {
    r = g = 255;
    b = 0;
  }
  if (value == 3)  //red
  {
    r = 255;
    g = b = 0;
  }
  if (value == 4)  //blue
  {
    r = 0;
    g = 0;
    b = 255;
  }
}

template <>
void tContentColorTrait<double>::GetBackgroundColor(int &r, int &g, int &b)
{
  r = g = b = 255;
}

template <>
void tContentColorTrait<bool>::GetColor(int &r, int &g, int &b, const bool &value)
{
  if (value)
  {
    r = g = b = 0;
  }
  else
  {
    r = g = b = 255;
  }
}

template <>
void tContentColorTrait<bool>::GetBackgroundColor(int &r, int &g, int &b)
{
  r = g = b = 255;
}




//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
