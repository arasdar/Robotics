//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    mSimulation.h
 *
 * \author  Aras
 *
 * \date    2013-10-15
 *
 * \brief Contains mSimulation
 *
 * \b mSimulation
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__icarus__simulation_physics__mSimulation_h__
#define __projects__icarus__simulation_physics__mSimulation_h__


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "libraries/physics_simulation/mPhysicsSimulationSimVis.h"
#include "rrlib/physics_simulation_newton/tEngineNewton.h"
#include "rrlib/simvis3d/tFrameGrabberCoin.h"
#include "core/file_lookup.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace simulation_physics
{


//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
class mSimulation : public physics_simulation::mPhysicsSimulationSimVis
{

public:

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  mSimulation(core::tFrameworkElement *parent, const std::string &name = "ExtendedPhysicsSimulationNewton");

//----------------------------------------------------------------------
// Protected methods (no fields/variables)
//----------------------------------------------------------------------
protected:

  virtual rrlib::physics_simulation::tEngine* CreateClassicPhysicsEngine()
  {
    return new rrlib::physics_simulation::newton::tEngineNewton(core::GetFinrocFile("projects/stereo_traversability_experiments/simulation_physics/etc/materials.mat"));
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual void OnStaticParameterChange();

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
