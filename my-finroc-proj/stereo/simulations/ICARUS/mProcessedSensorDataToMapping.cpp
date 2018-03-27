//
// You received this file as part of Finroc
// A Framework for intelligent robot control
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
/*!\file    projects/icarus/simulation/mProcessedSensorDataToMapping.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2013-07-23
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/simulation/mProcessedSensorDataToMapping.h"

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
namespace icarus
{
namespace simulation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<mProcessedSensorDataToMapping> cCREATE_ACTION_FOR_M_PROCESSEDSENSORDATATOMAPPING("ProcessedSensorDataToMapping");
const int scanner_radius_max_threshold = 19;
const int bound_limit = 40; //80;
const double cell_resolution = 0.1; //0.1 //1
const double mapping_scale = 0.1; //1; //0.1;
#define PI 3.14159265
const int threshNumPointsInCell_scanners = 3;




//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::secletmap_planar_scanner_front()
{

  typedef rrlib::mapping::tMapGridPolar2D<double> tMap;
  tMap map_main;

  typedef tMap::tCoordinate tVec;

  data_ports::tPortDataPointer<tMap> map_pointer = so_map_polar_front_scanner.GetUnusedBuffer();
  tMap& map = *map_pointer;
  map.Clear(0);

  rrlib::mapping::tMapGridPolar2D<double>::tBounds bounds;
  bounds.upper_bounds[0] = tVec(rrlib::math::tAngleDeg(90), 0); // first dimension: 90 degress till
  bounds.lower_bounds[0] = tVec(rrlib::math::tAngleDeg(-90), 0);  // -90 degrees
  bounds.upper_bounds[1] = tVec(0, 30); // second dimension: 20 meters to
  bounds.lower_bounds[1] = tVec(0, 0); // 0 meters
  map.SetBounds(bounds);
  map.SetResolution(tVec(rrlib::math::tAngleDeg(18), 1)); // resolution: 18 degrees, 1 meter

  // sector map for front laser scanner
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();

  if (distance_data->DistanceDataFormat() != rrlib::distance_data::eDF_POLAR_2D_DOUBLE)
  {
    FINROC_LOG_PRINT(WARNING, "Illegal input distance data format, eDF_POLAR_2D_DOUBLE required!\n");
    distance_data->Print();
    return;
  }
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *distance_data);
  assert(distance_data->DistanceDataFormat() == rrlib::distance_data::eDF_POLAR_2D_DOUBLE);
  /*-- NOTE: now we are sure that each sample is represented as tVec2 in polar form, i.e. (angle, radius) --*/

  // access distance data samples from source
  const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *data = reinterpret_cast<const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *>(distance_data->ConstDataPtr());

  // iterating over it
  int it_length, it_angle, counterNumPointsInEachCell[map.GetNumberOfCells()];
  for (int i = 0; i < map.GetNumberOfCells(); i++)
    counterNumPointsInEachCell[i] = 0;

  for (auto it = data; it != data + distance_data->Dimension(); ++it)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, abs(it->Length()));
    it_length = round(it->Length() / 1000); //unit is meter
    it_angle = round((it->Alpha() * 180 / PI)); //3.14
    if (it_length > 20 || it_length == 20)
      it_length = 19;
    if (it_length < 0 || it_length == 0)
      it_length = 0;
    if (it_angle > 90 || it_angle == 90)
      it_angle = 89;
    if (it_angle < -90 || it_angle == -90)
      it_angle = -90;
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1,  "sample with length: ", it_length, " and ------> at angle: ", it_angle); //static_cast<double>(it_angle)

    counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] ++;
    if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] > 4)
      map.GetCellByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length)) = 4; //obstacle
    if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] > 1 &&
        counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] < 4)
      map.GetCellByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length)) = -1; //semi-obstacle
  } ///for iteration

  so_map_polar_front_scanner.Publish(map_pointer);

}// secletmap_planar_scanner_front

void mProcessedSensorDataToMapping::secletmap_planar_scanner_rear()
{

  typedef rrlib::mapping::tMapGridPolar2D<double> tMap;
  tMap map_main;

  typedef tMap::tCoordinate tVec;

  data_ports::tPortDataPointer<tMap> map_pointer = so_map_polar_rear_scanner.GetUnusedBuffer();
  tMap& map = *map_pointer;
  map.Clear(0);

  rrlib::mapping::tMapGridPolar2D<double>::tBounds bounds;
  bounds.upper_bounds[0] = tVec(rrlib::math::tAngleDeg(-90), 0); // first dimension: 90 degress till
  bounds.lower_bounds[0] = tVec(rrlib::math::tAngleDeg(90), 0);  // -90 degrees
  bounds.upper_bounds[1] = tVec(0, 30); // second dimension: 20 meters to
  bounds.lower_bounds[1] = tVec(0, 0); // 0 meters
  map.SetBounds(bounds);
  map.SetResolution(tVec(rrlib::math::tAngleDeg(18), 1)); // resolution: 18 degrees, 1 meter

  // sector map for front laser scanner
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_rear_scanner.GetPointer();

  if (distance_data->DistanceDataFormat() != rrlib::distance_data::eDF_POLAR_2D_DOUBLE)
  {
    FINROC_LOG_PRINT(WARNING, "Illegal input distance data format, eDF_POLAR_2D_DOUBLE required!\n");
    distance_data->Print();
    return;
  }
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *distance_data);
  assert(distance_data->DistanceDataFormat() == rrlib::distance_data::eDF_POLAR_2D_DOUBLE);
  // NOTE: now we are sure that each sample is represented as tVec2 in polar form, i.e. (angle, radius)

  // access distance data samples from source
  const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *data = reinterpret_cast<const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *>(distance_data->ConstDataPtr());

  // ierating over it
  int it_length, it_angle, counterNumPointsInEachCell[map.GetNumberOfCells()];
  for (int i = 0; i < map.GetNumberOfCells(); i++)
    counterNumPointsInEachCell[i] = 0;

  for (auto it = data; it != data + distance_data->Dimension(); ++it)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);
    it_length = round(it->Length() / 1000); //20000 max length
    it_angle = round((it->Alpha() * 180 / PI)); //3.14

    //// radius bounds
    if (it_length > 20 || it_length == 20)
      it_length = scanner_radius_max_threshold;
    if (it_length < 0 || it_length == 0)
      it_length = 0;

    if (it_angle > 0)
      it_angle = -180 + it_angle;
    else
      it_angle = 180 + it_angle;

    if (it_angle == 0)
      it_angle = 180;
    if (it_angle == -90)  //upper bound //it_angle > -90 ||
      it_angle = -91;

    FINROC_LOG_PRINT(DEBUG_VERBOSE_1,  "sample with length: ", it_length, " and ------> at angle: ", it_angle); //static_cast<double>(it_angle)
    counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] ++;
    if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] > 4)
      map.GetCellByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length)) = 4; //obstacle
    if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] > 1 &&
        counterNumPointsInEachCell[map.GetCellIDByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length))] < 4)
      map.GetCellByCoordinate(tVec(rrlib::math::tAngleDeg(it_angle), it_length)) = -1; //semi-obstacle
  } ///for iteration

  so_map_polar_rear_scanner.Publish(map_pointer);
}// sector_map_rear_scanner

void mProcessedSensorDataToMapping::gridmap_planar_scanner()
{

  /*
   * -----------------------> 1- initializing the gird map <-----------------------------*/
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> map_pointer = so_gridmap_planar_scanner.GetUnusedBuffer();
  rrlib::mapping::tMapGridCartesian2D<double>& map = *map_pointer; //rrlib
  map.Clear(0);

  map.SetBounds(bounds_planar_scanner_env);
  map.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1)); // resolution: 0.1 meter, 0.1 meter //10 cm

  /*! reseting the gridmap for environment - the main one */
  if (ci_reset_gridmap_planar_scanner_env.Get() == 1)
  {
    map_planar_scanner_env.Clear(0);
  }// if ci


  /*
   * ---------------> front laser scanner <----------------------*/
  // reading the port
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();

  // access robot pose from source ... init absolute_sensor_pose
  rrlib::math::tPose3D absolute_sensor_pose(distance_data->RobotPose());

  // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
  absolute_sensor_pose.ApplyRelativePoseTransformation(distance_data->SensorPose());
  absolute_sensor_pose.Scale(0.001); // for displaying the minimum distance sample we need unit meter

  //saving the transformation matrix
  const rrlib::math::tMat4x4d robot2sensorTransformationMatrix = absolute_sensor_pose.GetTransformationMatrix();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose.Position());
  pose_front_scanner = absolute_sensor_pose.Position(); //rrlib::math::tVec3f


  // access distance data samples from source
  const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *data = reinterpret_cast<const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *>(distance_data->ConstDataPtr());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *data);


  /*
   * ------------------> filling out the grid cells for rear scanner*/
  int counterNumPointsInEachCell[map.GetNumberOfCells()];
  for (int i = 0; i < map.GetNumberOfCells(); i++)
    counterNumPointsInEachCell[i] = 0;

  for (auto it = data; it != data + distance_data->Dimension(); ++it)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);

    // save sensor relative position of min_distance_sample to be used independent of distance_data_bb
    rrlib::math::tVec2f data_position = it->GetCartesianVector() * 0.001; // for displaying the minimum distance sample we need unit meter
    double gridmap_x = data_position.X() * mapping_scale; //prev_ground_image->at(j, i).x; //image coordinate //minus right and plus left
    double gridmap_y = data_position.Y() * mapping_scale; //prev_ground_image->at(j, i).z;
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x, " ------- actuated scanner");
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y, "------- actuated scanner");

    absolute_sensor_pose.Set(robot2sensorTransformationMatrix * rrlib::math::tPose3D(data_position.X(), data_position.Y(), 0, 0, 0, 0).GetTransformationMatrix());
    rrlib::math::tVec3f position = absolute_sensor_pose.Position();
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "scanned point pose in WCS:  ", position);

    float position_x = position.X() * mapping_scale;
    float position_y = position.Y() * mapping_scale;

    //counting the number of points in each cell and filling it out for the front scanner
    if (it->Length() / 1000 < scanner_radius_max_threshold)
    {
      counterNumPointsInEachCell[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] ++;
//      if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] == 1
//          ||
//          counterNumPointsInEachCell[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] == 2)
      if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] < threshNumPointsInCell_scanners
          &&
          counterNumPointsInEachCell[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] > 0)
      {
//          map.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = -1; // semi-obstacle
        map.GetCellByCoordinate(rrlib::math::tVec2d(position_x, position_y)) = -1; //colorful obstacle for the environment mapping
      }// if semi

      if (counterNumPointsInEachCell[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] >= threshNumPointsInCell_scanners)
      {
//          map.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 4; // semi-obstacle
        map.GetCellByCoordinate(rrlib::math::tVec2d(position_x, position_y)) = 4; //colorful obstacle for the environment mapping
        map_planar_scanner_env.GetCellByCoordinate(rrlib::math::tVec2d(position_x, position_y)) = 4; //colorful obstacle for the environment mapping
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x, " ------- planar scanner");
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y, "------- planar scanner");
      }// obstacle

    } //if length
  }//for

  /*
   * ---------------> rear laser scanner <----------------------*/
  // reading the port
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data_rear_scanner = si_distance_data_rear_scanner.GetPointer();

  // access robot pose from source ... absolute_sensor_pose
  rrlib::math::tPose3D absolute_sensor_pose_rear_scanner(distance_data_rear_scanner->RobotPose());

  // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
  absolute_sensor_pose_rear_scanner.ApplyRelativePoseTransformation(distance_data_rear_scanner->SensorPose());
  absolute_sensor_pose_rear_scanner.Scale(0.001); // for displaying the minimum distance sample we need unit meter

  //saving the transformation matrix
  const rrlib::math::tMat4x4d robot2sensorTransformationMatrix_rear_scanner = absolute_sensor_pose_rear_scanner.GetTransformationMatrix();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_rear_scanner.Position());
  pose_rear_scanner = absolute_sensor_pose_rear_scanner.Position(); //rrlib::math::tVec3f

  // access distance data samples from source
  const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *data_rear_scanner = reinterpret_cast<const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *>(distance_data_rear_scanner->ConstDataPtr());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *data_rear_scanner);


  /*filling out the grid cells for rear scanner*/
  int counterNumPointsInEachCell_rear_scanner[map.GetNumberOfCells()];
  for (int i = 0; i < map.GetNumberOfCells(); i++)
  {
    counterNumPointsInEachCell_rear_scanner[i] = 0;
  }// for


  for (auto it = data_rear_scanner; it != data_rear_scanner + distance_data_rear_scanner->Dimension(); ++it)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, it->Length());

    // save sensor relative position of min_distance_sample to be used independent of distance_data_bb
    rrlib::math::tVec2f data_position = it->GetCartesianVector() * 0.001; // for displaying the minimum distance sample we need unit meter
    double gridmap_x = data_position.X() * mapping_scale; //prev_ground_image->at(j, i).x; //image coordinate //minus right and plus left
    double gridmap_y = data_position.Y() * mapping_scale; //prev_ground_image->at(j, i).z;
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x, " ------- actuated scanner");
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y, "------- actuated scanner");

    absolute_sensor_pose_rear_scanner.Set(robot2sensorTransformationMatrix_rear_scanner * rrlib::math::tPose3D(data_position.X(), data_position.Y(), 0, 0, 0, 0).GetTransformationMatrix());
    rrlib::math::tVec3f position = absolute_sensor_pose_rear_scanner.Position();
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "scanned point pose in WCS:  ", position);

    float position_x = position.X() * mapping_scale;
    float position_y = position.Y() * mapping_scale;


    //thresholding the points, counting the number of points in each cell and filling it out for the front scanner
    counterNumPointsInEachCell_rear_scanner[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] ++;
    if (it->Length() / 1000 < scanner_radius_max_threshold)
    {

//      if (counterNumPointsInEachCell_rear_scanner[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale))] == 1
//          ||
//          counterNumPointsInEachCell_rear_scanner[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale))] == 2)
      if (counterNumPointsInEachCell_rear_scanner[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] < threshNumPointsInCell_scanners
          &&
          counterNumPointsInEachCell_rear_scanner[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] > 0)
      {
        map.GetCellByCoordinate(rrlib::math::tVec2d(position_x, position_y)) = -1; //obstacle
      }// IF SEMI-OBSTACLE

      if (counterNumPointsInEachCell_rear_scanner[map.GetCellIDByCoordinate(rrlib::math::tVec2d(position_x, position_y))] >= threshNumPointsInCell_scanners)
      {
        map.GetCellByCoordinate(rrlib::math::tVec2d(position_x, position_y)) = 4; //obstacle
        map_planar_scanner_env.GetCellByCoordinate(rrlib::math::tVec2d(position_x, position_y)) = 4; //obstacle
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x, " ------- actuated scanner");
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y, "------- actuated scanner");
      }// if 2

    }// if length
  } ///for iteration



  /*---------------> pose of robot in temporary map <----------------------*/
  map.GetCellByCoordinate(rrlib::math::tVec2d(pose_rear_scanner.X() * mapping_scale, pose_rear_scanner.Y() * mapping_scale)) = -1; //rear scanner //distance_data->SensorPose()
  map.GetCellByCoordinate(rrlib::math::tVec2d(pose_front_scanner.X() * mapping_scale, pose_front_scanner.Y() * mapping_scale)) = 1; //front scanner

  /* --------------> pose of robot in canvas for global environment main map <------------------------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas_map_env = si_canvas_planar_scanner_env.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas_map_env = so_canvas_planar_scanner_env.GetUnusedBuffer();
  rrlib::canvas::tCanvas2D *out_canvas_map_env = output_canvas_map_env.Get();
  memcpy(out_canvas_map_env, input_canvas_map_env, sizeof(*input_canvas_map_env));
  out_canvas_map_env->DrawArrow(pose_rear_scanner.X() * mapping_scale, pose_rear_scanner.Y() * mapping_scale, pose_front_scanner.X() * mapping_scale, pose_front_scanner.Y() * mapping_scale, false);
//  //void DrawBox(T bottom_left_x, T bottom_left_y, T width, T height = -1);
//  out_canvas_map_env_new->DrawBox( (float)pose_rear_scanner.X() / 10, (float)pose_rear_scanner.Y() / 10, (double)0.2, -1); //(double)0.3

  /*------------------> last step for publishing the port for the canvas <--------------------------*/
  so_gridmap_planar_scanner.Publish(map_pointer);
  so_canvas_planar_scanner_env.Publish(output_canvas_map_env);
  so_gridmap_planar_scanner_env.Publish(map_planar_scanner_env);

}// gridmap_planar_scanner

void mProcessedSensorDataToMapping::gridmap_actuated_scanner()
{
  /*   * -----------------------> actuated scanner <-----------------------------*/
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> grid_map_actuated_scanner = so_gridmap_actuated_scanner.GetUnusedBuffer();
  rrlib::mapping::tMapGridCartesian2D<double>& map_actuated = *grid_map_actuated_scanner;
  map_actuated.Clear(0); //need to know and maintain the location of obstacles

  //the boundaries, size of the grid and the cell resolution
  map_actuated.SetBounds(bounds_planar_scanner_env);
  map_actuated.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1)); // resolution: 0.1 meter, 0.1 meter


  /*! reseting the gridmap for environment - the main one */
  if (ci_reset_gridmap_actuated_scanner_env.Get() == 1)
  {
    map_actuated_scanner_env.Clear(0);
  }


  /*
   * ---------------> mapping the actuated laser scanner <----------------------*/
  // reading the port
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data_env_actuated = si_distance_data_actuated_scanner.GetPointer();

  // access robot pose from source ... absolute_sensor_pose
  rrlib::math::tPose3D absolute_sensor_pose_env_actuated(distance_data_env_actuated->RobotPose());

  // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
  absolute_sensor_pose_env_actuated.ApplyRelativePoseTransformation(distance_data_env_actuated->SensorPose());
  absolute_sensor_pose_env_actuated.Scale(0.001); // for displaying the minimum distance sample we need unit meter

  //saving the transformation matrix
  const rrlib::math::tMat4x4d robot2sensorTransformationMatrix_env_actuated = absolute_sensor_pose_env_actuated.GetTransformationMatrix();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_env_actuated.Position());

  // access distance data samples from source in RCS (robot coordinate system)
  const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *data_env_actuated = reinterpret_cast<const rrlib::math::tVector<2, double, rrlib::math::vector::Polar> *>(distance_data_env_actuated->ConstDataPtr());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *data_env_actuated);

  /*
   * -------------------> filling out the grid cells for actuated scanner scanner*/
  int counterNumPointsInEachCell_env_actuated[map_actuated.GetNumberOfCells()];
  for (int i = 0; i < map_actuated.GetNumberOfCells(); i++)
  {
    counterNumPointsInEachCell_env_actuated[i] = 0;
  }//fori

  /*
   * ----- thresholding on height of the distance data -----------*/
  float min_z = 0.5 * pose_front_scanner.Z() / 100;
  float max_z = 5 * pose_front_scanner.Z() / 100;
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "MIN_Z: ", min_z, " max_z: ", max_z);

  double angle_actuated = si_angle_actuated.Get();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ANGLE for actuated: ", angle_actuated);

  rrlib::math::tMat3x3d rotation = rrlib::math::tPose3D(0, 0, 0, 0, 0, angle_actuated).GetRotationMatrix();  //yaw
  rrlib::math::tMat4x4d rotation_mat4x4 = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                          rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                          rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                          0, 0, 0, 1);

  for (auto it = data_env_actuated; it != data_env_actuated + distance_data_env_actuated->Dimension(); ++it)
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);

    // save sensor relative position of min_distance_sample to be used independent of distance_data_bb
    rrlib::math::tVec6f data_position = it->GetCartesianVector() * 0.001; // xyz alpha theta phi (rooll pitch yaw)
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ACTUATED SCANNER X: ", data_position.X());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ACTUATED SCANNER Y:", data_position.Y());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "1st ANGLE roll:  ", data_position.Roll());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "1st ANGLE pitch:  ", data_position.Pitch());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "1st ANGLE yaw:  ", data_position.Yaw());
    double gridmap_x = data_position.X() * mapping_scale; //prev_ground_image->at(j, i).x; //image coordinate //minus right and plus left
    double gridmap_y = data_position.Y() * mapping_scale; //prev_ground_image->at(j, i).z;
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x, " ------- actuated scanner");
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y, "------- actuated scanner");

    /*! transformation of the points to WCS */
    absolute_sensor_pose_env_actuated.Set(rotation_mat4x4 * robot2sensorTransformationMatrix_env_actuated * rrlib::math::tPose3D(data_position.X(), data_position.Y(), 0, 0, 0, 0).GetTransformationMatrix());
    rrlib::math::tVec6f position = absolute_sensor_pose_env_actuated.Position();
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "scanned point pose in WCS:  ", position.Z() / 100);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "2nd ANGLE roll:  ", position.Roll());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "2nd ANGLE pitch:  ", position.Pitch());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "2nd ANGLE yaw:  ", position.Yaw());
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "it-> length()/1000    : ", it->Length() / 1000);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position.Length()/1000: ", position.Length() / 1000);


    //counting the number of points in each cell, and checking the length and finally filling it out for the front scanner
    if (position.Z() / 100 > min_z && position.Z() / 100 < max_z && it->Length() / 1000 < scanner_radius_max_threshold)   //it->Length() < 19 //position.Length() /1000 < 19 &&
    {
      counterNumPointsInEachCell_env_actuated[map_actuated.GetCellIDByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale))] ++;

      if (counterNumPointsInEachCell_env_actuated[map_actuated.GetCellIDByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale))] == 1
          ||
          counterNumPointsInEachCell_env_actuated[map_actuated.GetCellIDByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale))] == 2)
      {
//        map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = -1; // semi-obstacle
//        map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = -1; // semi-obstacle
        map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale)) = -1; // obstacle

      }// if
      if (counterNumPointsInEachCell_env_actuated[map_actuated.GetCellIDByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale))] > 2)
      {
        map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale)) = 4; // obstacle
//          map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 4; // semi-obstacle
        map_actuated_scanner_env.GetCellByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale)) = 4;  //obstacle colorful
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x, " ------- actuated scanner");
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y, "------- actuated scanner");
      }
    }// if counter

    // ground
    if (position.Z() / 100 < 0)   //position.Z() / 100 > -1 * min_z && position.Z() / 100 < min_z   //it->Length() < 19 //position.Length() /1000 < 19 &&
    {
      //test
      map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale)) = 2; // green ground and traversabile

      if (map_actuated_scanner_env.GetCellByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale)) != 4) // not marked as red before
      {
        map_actuated_scanner_env.GetCellByCoordinate(rrlib::math::tVec2d(position.X() * mapping_scale, position.Y() * mapping_scale)) = 2; //green and traversable
      }// if already red
    }// if ground

  }//for



  /*---------------> pose of robot in temporary environment <----------------------*/
  map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(pose_rear_scanner.X() * mapping_scale, pose_rear_scanner.Y() * mapping_scale)) = -1; //rear scanner
  map_actuated.GetCellByCoordinate(rrlib::math::tVec2d(pose_front_scanner.X() * mapping_scale, pose_front_scanner.Y() * mapping_scale)) = 1; //front scanner

  /*------------------> last step for publishing the port for the canvas <--------------------------*/
  so_gridmap_actuated_scanner.Publish(grid_map_actuated_scanner);
  so_gridmap_actuated_scanner_env.Publish(map_actuated_scanner_env);

  /*---------------> pose of robot in environment <----------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas_map_env_actuated = si_canvas_actuated_env.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas_map_env_actuated = so_canvas_actuated_env.GetUnusedBuffer();
  rrlib::canvas::tCanvas2D *out_canvas_map_env_actuated = output_canvas_map_env_actuated.Get();
  memcpy(out_canvas_map_env_actuated, input_canvas_map_env_actuated, sizeof(*input_canvas_map_env_actuated));
  out_canvas_map_env_actuated->DrawArrow(pose_rear_scanner.X() * mapping_scale, pose_rear_scanner.Y() * mapping_scale, pose_front_scanner.X() * mapping_scale, pose_front_scanner.Y() * mapping_scale, false);
  so_canvas_actuated_env.Publish(output_canvas_map_env_actuated);

}// gridmap_actuated_scanner

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping constructor
//----------------------------------------------------------------------
mProcessedSensorDataToMapping::mProcessedSensorDataToMapping(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name)
{
  /*---------------------- mapping the environment ---------------------*/
  bounds_planar_scanner_env.upper_bounds[0] = rrlib::math::tVec2d(bound_limit, 0);
  bounds_planar_scanner_env.upper_bounds[1] = rrlib::math::tVec2d(0, bound_limit);
  bounds_planar_scanner_env.lower_bounds[0] = rrlib::math::tVec2d(-bound_limit, 0);
  bounds_planar_scanner_env.lower_bounds[1] = rrlib::math::tVec2d(0, -bound_limit);

  /*------------------ mapping planar laser scanner rear and front 2d --------------------------*/
  map_planar_scanner_env.SetBounds(bounds_planar_scanner_env);
  map_planar_scanner_env.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1));  //0.1 cell resolution is one meter in simulation //LUGV 3m - 3 cell size - 0.3 in grid map

  /*------------------- mapping actuated scanner ------------------*/
  map_actuated_scanner_env.SetBounds(bounds_planar_scanner_env);
  map_actuated_scanner_env.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.01, 0.01));
}// constructor

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping destructor
//----------------------------------------------------------------------
mProcessedSensorDataToMapping::~mProcessedSensorDataToMapping()
{}

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping OnStaticParameterChange
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::OnStaticParameterChange()
{}

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping OnParameterChange
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::OnParameterChange()
{}

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping Sense
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::Sense()
{
  if (this->SensorInputChanged())
  {
    /*
     * At least one of your sensor input ports has changed. Do something useful with its data.
     * However, using the .HasChanged() method on each port you can check in more detail.
     * */
    secletmap_planar_scanner_front();
    secletmap_planar_scanner_rear();
    gridmap_planar_scanner();
    gridmap_actuated_scanner();
  } //if

}// sense

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping Control
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::Control()
{}// control

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

