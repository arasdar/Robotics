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
/*!\file    projects/stereo_traversability_experiments/control/mRecognition.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2016-01-22
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/MLR/control/mRecognition.h"

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
namespace control
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mRecognition> cCREATE_ACTION_FOR_M_RECOGNITION("Recognition");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mRecognition constructor
//----------------------------------------------------------------------
mRecognition::mRecognition(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
/*! This is the part for initializing the data ports*/
  camera_image("Camera Image"),
  ir_distance_front("IR Distance Front"),
  ir_distance_l("IR Distance L"),
  ir_distance_ll("IR Distance LL"),
  ir_distance_lll("IR Distance LLL"),
  ir_distance_r("IR Distance R"),
  ir_distance_rr("IR Distance RR"),
  ir_distance_rrr("IR Distance RRR"),
  ir_distance_rear("IR Distance Rear"),
  desired_velocity("Desired Velocity", data_ports::tBounds<rrlib::si_units::tVelocity<>>(-1, 1)),
  desired_angular_velocity("Desired Angular Velocity", data_ports::tBounds<rrlib::si_units::tAngularVelocity<>>(-1, 1)),
  desired_fork_position("Desired Fork Position"),
  mlr(new stereo_traversability_experiments::mlr::tMLR),
  pca(new mlr::learn_PCA::tPCA),
  filenames(),
  recorded_data(new mlr::RecordedData),
  cycle_counter(0),
  is_recognizer_loaded(false),
  are_filenames_loaded(false)
{}// mRecognition::mRecognition()

//----------------------------------------------------------------------
// mRecognition destructor
//----------------------------------------------------------------------
mRecognition::~mRecognition()
{}

//----------------------------------------------------------------------
// mRecognition OnStaticParameterChange
//----------------------------------------------------------------------
void mRecognition::OnStaticParameterChange()
{
  // if dir was given
  if (this->dir_path.HasChanged())
  {
    FINROC_LOG_PRINT(DEBUG, "this->dir_path.Get(): ", this->dir_path.Get());
  }// if dir_path
}// param function

//----------------------------------------------------------------------
// mRecognition OnParameterChange
//----------------------------------------------------------------------
void mRecognition::OnParameterChange()
{}// end of function

//----------------------------------------------------------------------
// mRecognition Update
//----------------------------------------------------------------------
void mRecognition::Update() // every single machine cycle - cpu cycle
{
/////////////////////////////////////////////////////////////////////////////////////////////////////startttttttttttttttttttttttttttttttttttttt
  // if camera enable // OK
  if (this->enable_camera.Get() /*== TRUE*/)
  {
    // if recognizer loaded //OK
    if (this->is_recognizer_loaded /*== true*/
        && /*and*/
        this->are_filenames_loaded /*== true*/) // meaning the recognizer is loaded and ready to get the new input && also all filenames are loaded too
    {
      // is recognizer_on --> start recognizing and generating the new output controlling data
      if (this->start_recognizer.Get() /*== true*/)
      {
        // if waiting time is over  --
        if (this->cycle_counter == 1/*cNUMBER_OF_WAITING_CYCLES*/) // thisis now boolean = if controller_data_available =true
        {
          // reset the cycle_counter{ cycle_counter = 0;
          this->cycle_counter = 0;

          // look if the input changed == (meaning) if the robot moved or the scene or environment changed
          if (this->InputChanged())
          {
            // well we have new input data --> because input data has changed, environment & robot location has changed
            // that s why we can start and do the recognition process & process the data & analyze..... --> recognizer them
            /*recognizer_*/process();
          }// if InputChanged()
          else // if input has not changed - still in the same SCENE - go ahead and move -- if changed stop and analysis and go ahead again
          {
            FINROC_LOG_PRINT(DEBUG, "if input has not changed");
          }// // if input has not changed

        }// if (this->cycle_counter == cNUMBER_OF_WAITING_CYCLES)
        else // if not over, then wait (++, increment) // // if (this->cycle_counter != cNUMBER_OF_WAITING_CYCLES)
        {
          // count cycles enough for move
          FINROC_LOG_PRINT(DEBUG, "Applying controller data to actuators and act --------------------");

          // counter ++ til it is 5 - we waited for 5 cycles - cpu cycles -- dependes on how power ful it is - freq  of the machine or CPU
          this->cycle_counter ++;
        }// if (this->cycle_counter != cNUMBER_OF_WAITING_CYCLES)

      } // if (this->start_recognizer.Get() /*== true*/)
      else
      {
        FINROC_LOG_PRINT(DEBUG, "please start recognizer");
      }// if (this->start_recognizer.Get() /*== true*/)

    }// if recognizer is loaded and all filenames as well
    else // if (!this->is_recognizer_loaded /*== FALSE*/   || !this->are_filenames_loaded /*== FALSE*/)
    {
      //FINROC_LOG_PRINT(DEBUG, "!this->is_recognizer_loaded /*== FALSE*/  || !this->are_filenames_loaded /*== FALSE*/ ==> please load recognizer");
      FINROC_LOG_PRINT(DEBUG, "please load recognizer");
      // the problem is every time any parameter change all of them change together
      // start loading PCA and searching for XML files and loading their path
      if (this->load_recognizer.Get() /*== true*/) // these all should be done only and only once
      {
        // reading the Learned PCs and learned database
        /*recognizer_*/load();
      } //        if (this->load_recognizer.Get()) // these all should be done only and only once
    }// // if (!this->is_recognizer_loaded /*== FALSE*/   || !this->are_filenames_loaded /*== FALSE*/)
  } // if camera enabled
  else // if camera not enabled
  {
    FINROC_LOG_PRINT(DEBUG, "please enable the camera");
  } // if camera not enabled
///////////////////////////////////////////////////////////////////////////////////////////// endddddddddddddddddddddddddddddd
}// end of Update() -- each cycle update --> void mRecognition::Update()

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
