////
//// You received this file as part of Finroc
//// A framework for intelligent robot control
////
//// Copyright (C) Finroc GbR (finroc.org)
////
//// This program is free software; you can redistribute it and/or modify
//// it under the terms of the GNU General Public License as published by
//// the Free Software Foundation; either version 2 of the License, or
//// (at your option) any later version.
////
//// This program is distributed in the hope that it will be useful,
//// but WITHOUT ANY WARRANTY; without even the implied warranty of
//// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//// GNU General Public License for more details.
////
//// You should have received a copy of the GNU General Public License along
//// with this program; if not, write to the Free Software Foundation, Inc.,
//// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
////
////----------------------------------------------------------------------
///*!\file    rrlib/canvas/rtti.cpp
// *
// * \author  Tobias Föhst
// * 			Aras Dar
// *
// * \date    2015-12-01
// * 			2018-04-06
// *
// */
////----------------------------------------------------------------------
//
//#ifdef _LIB_RRLIB_RTTI_PRESENT_
//
////----------------------------------------------------------------------
//// External includes (system with <>, local with "")
////----------------------------------------------------------------------
//#include "rrlib/rtti/rtti.h"
//
////----------------------------------------------------------------------
//// Internal includes with ""
////----------------------------------------------------------------------
////#include "rrlib/canvas/tCanvas2D.h"
////#include "rrlib/canvas/tCanvas3D.h"
//#include "projects/test/tRaspberryPieCamera.h"
//
////----------------------------------------------------------------------
//// Debugging
////----------------------------------------------------------------------
//
////----------------------------------------------------------------------
//// Namespace usage
////----------------------------------------------------------------------
//
////----------------------------------------------------------------------
//// Namespace declaration
////----------------------------------------------------------------------
//namespace finroc
//{
//namespace test
//{
//
////----------------------------------------------------------------------
//// Const values
////----------------------------------------------------------------------
////const rrlib::rtti::tDataType<tCanvas2D> cINIT_TYPE_CANVAS2D;
////const rrlib::rtti::tDataType<tCanvas3D> cINIT_TYPE_CANVAS3D;
//const rrlib::rtti::tDataType<tRaspberryPieCamera> cINIT_TYPE_RaspberryPieCamera;
//
////----------------------------------------------------------------------
//// End of namespace declaration
////----------------------------------------------------------------------
//}
//}
//
//#endif
//
//
/*
 * rtti.cpp
 *
 *  Created on: Apr 8, 2018
 *      Author: arasdar
 */
#ifdef _LIB_RRLIB_RTTI_PRESENT_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/rtti.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
//#include "rrlib/xyz/tAnnotatedPose2D.h"
#include "projects/test/tRaspberryPieCamera.h"

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
//using namespace rrlib::xyz;
using namespace rrlib::rtti;

//----------------------------------------------------------------------
// Type initializers
//----------------------------------------------------------------------
//static tDataType<tAnnotatedPose2D> init_type_annotated_pose_2d;
static tDataType<finroc::test::tRaspberryPieCamera> init_type_camera;
//static tDataType<finroc::test::tAnnotatedPose2D> init_type_pose;

#endif
