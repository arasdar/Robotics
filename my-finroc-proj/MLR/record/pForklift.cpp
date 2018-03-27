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
/*!\file    pForklift.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-11-06
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "core/tRuntimeEnvironment.h"
#include "plugins/structure/tSenseControlModule.h"
#include "plugins/runtime_construction/dynamic_loading.h"

#include <boost/algorithm/string.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/forklift/gRobotInterface.h"
#include "projects/forklift/gMainControlBase.h"

#ifdef _LIB_FINROC_PROJECTS_FORKLIFT_HARDWARE_PRESENT_
#include "projects/forklift/hardware/gHardware.h"
#endif

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::forklift;
using namespace finroc::core;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the ForkliftTest module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

const std::string cROBOT_INTERFACE_CONTROLLER_INPUT_LINK = "/Forklift/RobotInterface/Controller Input";
const std::string cROBOT_INTERFACE_SENSOR_OUTPUT_LINK = "/Forklift/RobotInterface/Sensor Output";

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

static bool create_visualization = true;
static bool create_hardware = false;
static bool create_simulation_client = false;
static bool create_simulation_standalone = false;
static bool create_robot_interface = true;
static std::string control_to_load;
int cycle_time = 10;
tRobotInterfaceMode mode = eRIM_SIMULATION_ROBOT1;
static std::string mod_xml;

bool CycleTimeHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption port_option(name_to_option_map.at("cycle-time"));
  if (port_option->IsActive())
  {
    std::string time_string = rrlib::getopt::EvaluateValue(port_option);
    int time = atoi(time_string.c_str());
    if (time < 1 || time > 10000)
    {
      FINROC_LOG_PRINT_STATIC(ERROR, "Invalid cycle time '", time_string, "'. Using default: ", cycle_time, " ms");
    }
    else
    {
      FINROC_LOG_PRINT_STATIC(DEBUG, "Setting main thread cycle time to ", time, " ms.");
      cycle_time = time;
    }
  }

  return true;
}

bool ModHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption port_option(name_to_option_map.at("mod"));
  if (port_option->IsActive())
  {
    mod_xml = rrlib::getopt::EvaluateValue(port_option);
  }

  return true;
}

bool VisualizationHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption v(name_to_option_map.at("no-visualization"));
  create_visualization = !v->IsActive();
  return true;
}

bool MainControlHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption port_option(name_to_option_map.at("control"));
  if (port_option->IsActive())
  {
    std::string control(rrlib::getopt::EvaluateValue(port_option));
    control_to_load = "finroc_projects_forklift_control_" + control;
  }

  return true;
}

bool ModeHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption port_option(name_to_option_map.at("mode"));
  if (port_option->IsActive())
  {
    std::string mode_string(rrlib::getopt::EvaluateValue(port_option));
    if (boost::iequals(mode_string, "sim1"))
    {
      mode = eRIM_SIMULATION_ROBOT1;
    }
    else if (boost::iequals(mode_string, "sim2"))
    {
      mode = eRIM_SIMULATION_ROBOT2;
    }
    else if (boost::iequals(mode_string, "hw"))
    {
      mode = eRIM_HARDWARE;
      create_hardware = true;
    }
    else if (boost::iequals(mode_string, "rhw"))
    {
      mode = eRIM_HARDWARE;
      create_robot_interface = false;
      create_visualization = false;
    }
    else
    {
      FINROC_LOG_PRINT_STATIC(WARNING, "Unknown mode: ", mode_string, ". Using 'Simulation Robot1'.");
    }
  }

  return true;
}

bool SimulationClientHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption v(name_to_option_map.at("simulation-client"));
  create_simulation_client = v->IsActive();
  return true;
}
bool SimulationStandaloneHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption v(name_to_option_map.at("simulation-standalone"));
  create_simulation_standalone = v->IsActive();
  return true;
}
bool ScenarioHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption scenario_option(name_to_option_map.at("scenario"));
  if (scenario_option->IsActive())
  {
    tRuntimeEnvironment::GetInstance().AddCommandLineArgument("scenario", rrlib::getopt::EvaluateValue(scenario_option));
  }
  return true;
}

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{
  rrlib::getopt::AddValue("cycle-time", 't', "Cycle time of main thread in ms (default is 20)", &CycleTimeHandler);
  rrlib::getopt::AddValue("mod", 'm', "'Forklift-Mod': .finroc file for any further stuff in top-level container.", &ModHandler);
  rrlib::getopt::AddFlag("no-visualization", 'v', "Do not create Visualization.", &VisualizationHandler);
  rrlib::getopt::AddValue("control", 0, "Robot control to instantiate (libfinroc_projects_forklift_control_<name>.so containing 'MainControl').", &MainControlHandler);
  rrlib::getopt::AddValue("mode", 0, "Connect to simulation or use real robot? Options: 'sim1'=Simulation Robot1, 'sim2'=Simulation Robot2, 'hw'=Hardware, 'rhw'=Remote Hardware. Default is 'Simulation Robot1'.", &ModeHandler);
  rrlib::getopt::AddFlag("simulation-client", 0, "Connect to simulation server", &SimulationClientHandler);
  rrlib::getopt::AddFlag("simulation-standalone", 0, "Instantiate simulation in this part", &SimulationStandaloneHandler);
  rrlib::getopt::AddValue("scenario", 0, "Select scenario for simulation (see files in /etc/scenario - default is 'default')", &ScenarioHandler);
}

//----------------------------------------------------------------------
// CreateMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  finroc::structure::tTopLevelThreadContainer<> *main_thread = new finroc::structure::tTopLevelThreadContainer<>("Forklift", __FILE__".xml", true, make_all_port_links_unique);

  gRobotInterface* robot_interface = create_robot_interface ? new gRobotInterface(main_thread, "RobotInterface", mode) : nullptr;
  main_thread->SetCycleTime(cycle_time);

  // Create visualization?
  if (create_visualization)
  {
    try
    {
      finroc::runtime_construction::tCreateFrameworkElementAction& action = finroc::runtime_construction::LoadComponentType("finroc_projects_forklift_visualization", "Visualization");
      finroc::structure::tSenseControlModule* vis = static_cast<finroc::structure::tSenseControlModule*>(action.CreateModule(main_thread, "Visualization"));
      if (robot_interface)
      {
        vis->GetSensorInputs().ConnectByName(robot_interface->GetSensorOutputs(), false);
      }
      else
      {
        vis->GetSensorInputs().ConnectByName(cROBOT_INTERFACE_SENSOR_OUTPUT_LINK);
      }
    }
    catch (const std::exception& exception)
    {
      FINROC_LOG_PRINT_STATIC(ERROR, "Instantiating visualization failed: ", exception);
    }
  }

  // instantiate hardware?
  if (create_hardware)
  {
#ifdef _LIB_FINROC_PROJECTS_FORKLIFT_HARDWARE_PRESENT_
    new gHardware(main_thread, "Hardware");
#else
    RRLIB_LOG_PRINT_STATIC(ERROR, "libfinroc_projects_forklift_hardware.so not available. Cannot instantiate hardware access modules. Interface is in disconnected state.");
#endif
  }

  // Create control?
  if (control_to_load.length() > 0)
  {
    try
    {
      finroc::runtime_construction::tCreateFrameworkElementAction& action = finroc::runtime_construction::LoadComponentType(control_to_load, "MainControl");
      tFrameworkElement* c = action.CreateModule(main_thread, "MainControl");
      gMainControlBase* control = dynamic_cast<gMainControlBase*>(c);
      if (!control)
      {
        FINROC_LOG_PRINT(ERROR, "'MainControl' in ", control_to_load, " is not derived from 'gMainControlBase'");
        abort();
      }
      if (robot_interface)
      {
        control->GetSensorInputs().ConnectByName(robot_interface->GetSensorOutputs(), false);
        control->GetControllerOutputs().ConnectByName(robot_interface->GetControllerInputs(), false);
      }
      else
      {
        control->GetSensorInputs().ConnectByName(cROBOT_INTERFACE_SENSOR_OUTPUT_LINK);
        control->GetControllerOutputs().ConnectByName(cROBOT_INTERFACE_CONTROLLER_INPUT_LINK);
      }

      // attach config file to main control again - so that parameters can be connected and saved
      finroc::parameters::tConfigFile* config_file = finroc::parameters::tConfigFile::Find(*main_thread);
      if (config_file)
      {
        control->AddAnnotation(* new finroc::parameters::tConfigFile(config_file->GetFilename()));
      }

      // Open main_control in finstruct
      control->InitiallyShowInTools(10);

      // Allow control to modify robot interface
      if (robot_interface)
      {
        control->ModifyRobotInterface(*robot_interface);
      }
    }
    catch (const std::exception& exception)
    {
      FINROC_LOG_PRINT_STATIC(ERROR, "Instantiating control failed: ", exception);
      abort();
    }
  }

  // Instantiate mod? (Do this last so it can connect everything to control)
  //if (mod_xml.length() > 0)
  //{
  //  main_thread->xml_file.Set(mod_xml);
  //}

  if (create_simulation_client)
  {
    try
    {
      finroc::runtime_construction::LoadComponentType("libfinroc_projects_forklift_simulation_client.so", "SimulationClient").CreateModule(main_thread, "Simulation");
    }
    catch (const std::exception& exception)
    {
      FINROC_LOG_PRINT_STATIC(ERROR, "Instantiating simulation module failed: ", exception);
      abort();
    }
  }
  else if (create_simulation_standalone)
  {
    try
    {
      finroc::runtime_construction::LoadComponentType("libfinroc_projects_forklift_simulation_standalone.so", "Simulation").CreateModule(main_thread, "Simulation");
    }
    catch (const std::exception& exception)
    {
      FINROC_LOG_PRINT_STATIC(ERROR, "Instantiating simulation module failed: ", exception);
      abort();
    }
  }

}// end
