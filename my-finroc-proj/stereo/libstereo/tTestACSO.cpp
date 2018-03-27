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
/*!\file    projects/icarus/sensor_processing/libstereo_test/tTestACSO.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-28
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestACSO.h"

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
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace aras
{
namespace libstereo
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

//----------------------------------------------------------------------
// tTestACSO constructors
//----------------------------------------------------------------------
tTestACSO::tTestACSO()
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  radius_ = 5;

  gamma_s_ = 15;
  gamma_c_ = 25;

  smoothness_strong_ = 100;
  smoothness_weak_ = 20;
}

//----------------------------------------------------------------------
// tTestACSO destructor
//----------------------------------------------------------------------
tTestACSO::~tTestACSO()
{}

//////////////////////////////////////////////////////////////////////////////
void
tTestACSO::compute_impl(unsigned char* ref_img, unsigned char* trg_img)
{

  /*
   * initializaing or constructing the variables
   */
  //spatial distance weight computations
  float *ds = new float[ 2 * radius_ + 1 ];
  for (int j = -radius_; j <= radius_; j++)
    ds[j + radius_] = static_cast<float>(exp(- abs(j) / gamma_s_));

  //LUT for color distance weight computation
  float lut[256];
  for (int j = 0; j < 256; j++)
    lut[j] = float(exp(-j / gamma_c_));

  /////////////////////////////////////
  // perform computation on host (to enable result comparison later)
  /////////////////////////////////////
  /*/
  // run kernel using openmp
  /*/
  Timer timer;
  timer.start();
  compute_impl_parallel_openmp(ref_img, trg_img,
                               ds, lut);
  double openmptime = timer.stop();
  std::cout << "OpenMP implementation   : ";
  std::cout << "Time: " << std::fixed << std::setprecision(4) << openmptime << "s";
  std::cout << std::endl;

  /*
   * deconstructing the variables on CPU
   */
  delete [] ds;

} //compute_impl_cuda

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
