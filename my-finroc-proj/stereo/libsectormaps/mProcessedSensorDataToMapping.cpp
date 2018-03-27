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
#include "projects/icarus/simulation_physx_ogre/mProcessedSensorDataToMapping.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
//#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace std;

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace simulation_physx_ogre
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<mProcessedSensorDataToMapping> cCREATE_ACTION_FOR_M_PROCESSEDSENSORDATATOMAPPING("ProcessedSensorDataToMapping");

const double front_scanner_length_threshold = 19.0;
const int gridmap_bound_limit = 80; //10 //20 //40; //80;
const double gridmap_cell_resolution = 0.1; //0.05; //meter as unit
const double robot_width = 2, robot_height = 3.3; //this is based on vehicle size
const double sector_cell_height = 1;
const double sector_polar_cell_angle = 9;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void mProcessedSensorDataToMapping::GridmapShiftable()
{
  typedef rrlib::mapping::tMapGridCartesian2DShiftable<double> tMap;
  tMap& map_shift = gridmap_shift;
  map_shift.Clear(0);

  /*---------------> perception and sensor processing using front laser scanner for obstacle detection <----------------------*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data_front_scanner = si_distance_data_front_scanner.GetPointer();

  const rrlib::math::tVector<2, float, rrlib::math::vector::Polar> *sensor_data = reinterpret_cast<const rrlib::math::tVector<2, float, rrlib::math::vector::Polar> *>(distance_data_front_scanner->ConstDataPtr());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *sensor_data);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "distance_data_front_scanner->SensorPose(): ", distance_data_front_scanner->SensorPose());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "distance_data_front_scanner->RobotPose(): ", distance_data_front_scanner->RobotPose());

  /*rrlib::math::tPose3D*/ robot_pose = si_robot_pose.Get();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "robot_pose: ", robot_pose);

  for (auto it = sensor_data; it != sensor_data + distance_data_front_scanner->Dimension(); ++it)
  {

    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);

    rrlib::math::tVec2f sensor_data_position = it->GetCartesianVector(); // for displaying the minimum distance sample we need unit meter
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "sensor_data_position", sensor_data_position);

    /*! obstacles in map */
    rrlib::math::tMat4x4d matrix_obst = robot_pose.GetTransformationMatrix() *
                                        distance_data_front_scanner->SensorPose().GetTransformationMatrix() *
                                        rrlib::math::tPose3D(sensor_data_position.X(), sensor_data_position.Y(), 0, 0, 0, 0).GetTransformationMatrix();

    rrlib::math::tVec2d position_obst_scaled = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

    /*! sensor obstacle position*/
    rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();
    rrlib::math::tMat4x4d matrix_robot_rotation = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
        rotation[1][0], rotation[1][1], rotation[1][2], 0,
        rotation[2][0], rotation[2][1], rotation[2][2], 0,
        0, 0, 0, 1);

    rrlib::math::tMat4x4d matrix_obst_sensor = matrix_robot_rotation *
        distance_data_front_scanner->SensorPose().GetTransformationMatrix() *
        rrlib::math::tPose3D(sensor_data_position.X(), sensor_data_position.Y(), 0, 0, 0, 0).GetTransformationMatrix();

    rrlib::math::tVec2i position_obst_sensor_int = rrlib::math::tVec2i(matrix_obst_sensor[0][3], matrix_obst_sensor[1][3]);

    if (map_shift.IsCoordinateInBounds(position_obst_scaled))
    {

      /*        //mapping the sensor data
       *        rrlib::math::tVec2d position_obst_sensor = rrlib::math::tVec2d(matrix_obst_sensor[0][3], matrix_obst_sensor[1][3]);
       map_shift.GetCellByCoordinate(position_obst_sensor) = 1; //colorful obstacle for the environment mapping
       *
       */


      if (it->Length() < front_scanner_length_threshold && it->Length() > 2)
      {

        if (data_obst_all.size() < 1) //200 //some sample  for testing
        {
          data_obst_all.push_back(position_obst_scaled);
        }
        else
        {

          bool obstExist = false;
          for (auto it = data_obst_all.begin(); it != data_obst_all.end(); ++it)
          {

            rrlib::math::tVec2d position_obst_input = rrlib::math::tVec2d(it->X(), it->Y()); // for displaying the minimum distance sample we need unit meter

            rrlib::math::tMat4x4d matrix_obst = rrlib::math::tPose3D(position_obst_input.X(), position_obst_input.Y(), 0 , 0 , 0, 0).GetTransformationMatrix();

            rrlib::math::tVec2d position_obst = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

            rrlib::math::tMat4x4d matrix_trans = rrlib::math::tPose3D(robot_pose.X(), robot_pose.Y(), 0, 0, 0, 0).GetTransformationMatrix();

            rrlib::math::tMat4x4d matrix_trans_inverse = matrix_trans.Inverse() *
                rrlib::math::tPose3D(position_obst.X(), position_obst.Y(), 0, 0, 0, 0).GetTransformationMatrix();

            rrlib::math::tVec2i position_obst_scaled_int = rrlib::math::tVec2i(matrix_trans_inverse[0][3], matrix_trans_inverse[1][3]);

            if (position_obst_scaled_int == position_obst_sensor_int)
            {
              obstExist = true;
            }
          }

          if (!obstExist)
          {
            data_obst_all.push_back(position_obst_scaled);
          }
        }//else

      }

      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, *it);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "sensor_data_position", sensor_data_position);

    }// if inside the map

  }//for iterating through all sensor data

  /*
   * sectormap initialization
   */
  /*! translation to origin*/
  rrlib::math::tVec3d robot_trans_center = rrlib::math::tVec3d(-robot_height / 2, robot_width / 2, 0);

  /*! sector mapping polar   */
  tSectorMapPolar2D* sectormap_polar_front_left = new tSectorMapPolar2D;
  sectormap_polar_front_left->position = mapping::tSectorMapPosition::eFRONT_LEFT_POLAR;
  sectormap_polar_front_left->origin.Set(robot_height, 0, 0);
  sectormap_polar_front_left->origin += robot_trans_center;

  tSectorMapPolar2D* sectormap_polar_front_right = new tSectorMapPolar2D;
  sectormap_polar_front_right->position = mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR;
  sectormap_polar_front_right->origin.Set(robot_height, -robot_width, 0);
  sectormap_polar_front_right->origin += robot_trans_center;

  tSectorMapPolar2D* sectormap_polar_rear_left = new tSectorMapPolar2D;
  sectormap_polar_rear_left->position = mapping::tSectorMapPosition::eREAR_LEFT_POLAR;
  sectormap_polar_rear_left->origin.Set(0, -0, 0);
  sectormap_polar_rear_left->origin += robot_trans_center;

  tSectorMapPolar2D* sectormap_polar_rear_right = new tSectorMapPolar2D;
  sectormap_polar_rear_right->position = mapping::tSectorMapPosition::eREAR_RIGHT_POLAR;
  sectormap_polar_rear_right->origin.Set(0, -robot_width, 0);
  sectormap_polar_rear_right->origin += robot_trans_center;

  /*! sector mapping cartesian   */
  tSectorMapCartesian2D* sectormap_cartesian_left_rear = new tSectorMapCartesian2D;
  sectormap_cartesian_left_rear->position = mapping::tSectorMapPosition::eLEFT_REAR_CARTESIAN;
  sectormap_cartesian_left_rear->origin.Set(0, 0, 0);
  sectormap_cartesian_left_rear->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_left_front = new tSectorMapCartesian2D;
  sectormap_cartesian_left_front->position = mapping::tSectorMapPosition::eLEFT_FRONT_CARTESIAN;
  sectormap_cartesian_left_front->origin.Set(robot_height / 2, 0, 0);
  sectormap_cartesian_left_front->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_front_left = new tSectorMapCartesian2D;
  sectormap_cartesian_front_left->position = mapping::tSectorMapPosition::eFRONT_LEFT_CARTESIAN;
  sectormap_cartesian_front_left->origin.Set(robot_height, 0, 0);
  sectormap_cartesian_front_left->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_front_right = new tSectorMapCartesian2D;
  sectormap_cartesian_front_right->position = mapping::tSectorMapPosition::eFRONT_RIGHT_CARTESIAN;
  sectormap_cartesian_front_right->origin.Set(robot_height, -robot_width / 2, 0);
  sectormap_cartesian_front_right->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_right_rear = new tSectorMapCartesian2D;
  sectormap_cartesian_right_rear->position = mapping::tSectorMapPosition::eRIGHT_REAR_CARTESIAN;
  sectormap_cartesian_right_rear->origin.Set(0, -robot_width, 0);
  sectormap_cartesian_right_rear->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_right_front = new tSectorMapCartesian2D;
  sectormap_cartesian_right_front->position = mapping::tSectorMapPosition::eRIGHT_FRONT_CARTESIAN;
  sectormap_cartesian_right_front->origin.Set(robot_height / 2, -robot_width, 0);
  sectormap_cartesian_right_front->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_rear_left = new tSectorMapCartesian2D;
  sectormap_cartesian_rear_left->position = mapping::tSectorMapPosition::eREAR_LEFT_CARTESIAN;
  sectormap_cartesian_rear_left->origin.Set(0, 0, 0);
  sectormap_cartesian_rear_left->origin += robot_trans_center;


  tSectorMapCartesian2D* sectormap_cartesian_rear_right = new tSectorMapCartesian2D;
  sectormap_cartesian_rear_right->position = mapping::tSectorMapPosition::eREAR_RIGHT_CARTESIAN;
  sectormap_cartesian_rear_right->origin.Set(0, -robot_width / 2, 0);
  sectormap_cartesian_rear_right->origin += robot_trans_center;

  /*! sector mapping polar big second set   */
  tSectorMapPolar2D* sectormap_polar_front_left_big = new tSectorMapPolar2D;
  sectormap_polar_front_left_big->position = mapping::tSectorMapPosition::eFRONT_LEFT_POLAR_BIG;
  sectormap_polar_front_left_big->origin.Set(robot_height, -robot_width / 2, 0);
  sectormap_polar_front_left_big->origin += robot_trans_center;

  tSectorMapPolar2D* sectormap_polar_front_right_big = new tSectorMapPolar2D;
  sectormap_polar_front_right_big->position = mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR_BIG;
  sectormap_polar_front_right_big->origin.Set(robot_height, -robot_width / 2, 0);
  sectormap_polar_front_right_big->origin += robot_trans_center;

  tSectorMapPolar2D* sectormap_polar_rear_left_big = new tSectorMapPolar2D;
  sectormap_polar_rear_left_big->position = mapping::tSectorMapPosition::eREAR_LEFT_POLAR_BIG;
  sectormap_polar_rear_left_big->origin.Set(0, -robot_width / 2, 0);
  sectormap_polar_rear_left_big->origin += robot_trans_center;

  tSectorMapPolar2D* sectormap_polar_rear_right_big = new tSectorMapPolar2D;
  sectormap_polar_rear_right_big->position = mapping::tSectorMapPosition::eREAR_RIGHT_POLAR_BIG;
  sectormap_polar_rear_right_big->origin.Set(0, -robot_width / 2, 0);
  sectormap_polar_rear_right_big->origin += robot_trans_center;

  /*! sector mapping cartesian big second set   */
  tSectorMapCartesian2D* sectormap_cartesian_front = new tSectorMapCartesian2D;
  sectormap_cartesian_front->position = mapping::tSectorMapPosition::eFRONT_CARTESIAN;
  sectormap_cartesian_front->origin.Set(robot_height, 0, 0);
  sectormap_cartesian_front->origin += robot_trans_center;

  tSectorMapCartesian2D* sectormap_cartesian_rear = new tSectorMapCartesian2D;
  sectormap_cartesian_rear->position = mapping::tSectorMapPosition::eREAR_CARTESIAN;
  sectormap_cartesian_rear->origin.Set(0, 0, 0);
  sectormap_cartesian_rear->origin += robot_trans_center;

  /*
   * displaying all obstacles in map and detecting them on map using sector maps
   */
  for (auto it = data_obst_all.begin(); it != data_obst_all.end(); ++it)
  {

    rrlib::math::tVec2d position_obst_input = rrlib::math::tVec2d(it->X(), it->Y()); // for displaying the minimum distance sample we need unit meter

    rrlib::math::tMat4x4d matrix_obst = rrlib::math::tPose3D(position_obst_input.X(), position_obst_input.Y(), 0 , 0 , 0, 0).GetTransformationMatrix();

    rrlib::math::tVec2d position_obst = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

    rrlib::math::tMat4x4d matrix_trans = rrlib::math::tPose3D(robot_pose.X(), robot_pose.Y(), 0, 0, 0, 0).GetTransformationMatrix();

    rrlib::math::tMat4x4d matrix_trans_inverse = matrix_trans.Inverse() *
        rrlib::math::tPose3D(position_obst.X(), position_obst.Y(), 0, 0, 0, 0).GetTransformationMatrix();

    rrlib::math::tVec2d position_obst_scaled = rrlib::math::tVec2d(matrix_trans_inverse[0][3], matrix_trans_inverse[1][3]);

    if (map_shift.IsCoordinateInBounds(position_obst_scaled))
    {
      map_shift.GetCellByCoordinate(position_obst_scaled) = 3; //colorful obstacle for the environment mapping

      /*! filling the sectormaps*/
      /*! measuring the distance to obstacle in each cell in polar sectormap*/
      dist(*sectormap_polar_front_left, position_obst_scaled);
      dist(*sectormap_polar_front_right, position_obst_scaled);
      dist(*sectormap_polar_rear_left, position_obst_scaled);
      dist(*sectormap_polar_rear_right, position_obst_scaled);

      dist_cart(*sectormap_cartesian_left_rear, position_obst_scaled);
      dist_cart(*sectormap_cartesian_left_front, position_obst_scaled);
      dist_cart(*sectormap_cartesian_front_left, position_obst_scaled);
      dist_cart(*sectormap_cartesian_front_right, position_obst_scaled);
      dist_cart(*sectormap_cartesian_right_rear, position_obst_scaled);
      dist_cart(*sectormap_cartesian_right_front, position_obst_scaled);
      dist_cart(*sectormap_cartesian_rear_left, position_obst_scaled);
      dist_cart(*sectormap_cartesian_rear_right, position_obst_scaled);

      dist_polar_second(*sectormap_polar_front_left_big, position_obst_scaled);
      dist_polar_second(*sectormap_polar_front_right_big, position_obst_scaled);
      dist_polar_second(*sectormap_polar_rear_left_big, position_obst_scaled);
      dist_polar_second(*sectormap_polar_rear_right_big, position_obst_scaled);

      dist_cart_second(*sectormap_cartesian_rear, position_obst_scaled);
      dist_cart_second(*sectormap_cartesian_front, position_obst_scaled);
    }//if inside map

  }// for iterating through obstacle map

  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "data_obst_all.size: ", data_obst_all.size());
  so_gridmap_number_obst_total.Publish(data_obst_all.size());

  /* --------------> drawing robot in map <------------------------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas = si_canvas_gridmap_shift.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas = so_canvas_gridmap_shift.GetUnusedBuffer();
  rrlib::canvas::tCanvas2D *output_canvas_temp = output_canvas.Get();
  memcpy(output_canvas_temp, input_canvas, sizeof(*input_canvas));

  /*! visualizing the cartesian sectormaps */
  if (ci_sectormap_cart.Get())
  {
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ci_sectormap_cart.Get(): ", ci_sectormap_cart.Get());

    DrawSectorPolarTest(*output_canvas_temp, *sectormap_polar_front_left);
    DrawSectorPolarTest(*output_canvas_temp, *sectormap_polar_front_right);
    DrawSectorPolarTest(*output_canvas_temp, *sectormap_polar_rear_left);
    DrawSectorPolarTest(*output_canvas_temp, *sectormap_polar_rear_right);

    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_left_rear);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_left_front);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_front_left);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_front_right);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_right_rear);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_right_front);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_rear_left);
    DrawSectorCartesianTest(*output_canvas_temp, *sectormap_cartesian_rear_right);
  }

  if (ci_sectormap_polar_2.Get())
  {
    DrawSectorPolarTestSecond(*output_canvas_temp, *sectormap_polar_front_left_big);
    DrawSectorPolarTestSecond(*output_canvas_temp, *sectormap_polar_front_right_big);
    DrawSectorPolarTestSecond(*output_canvas_temp, *sectormap_polar_rear_left_big);
    DrawSectorPolarTestSecond(*output_canvas_temp, *sectormap_polar_rear_right_big);
  }

  if (ci_sectormap_cart_2.Get())
  {
    DrawSectorCartesianTestSecond(*output_canvas_temp, *sectormap_cartesian_front);
    DrawSectorCartesianTestSecond(*output_canvas_temp, *sectormap_cartesian_rear);
  }


  /*! draw robot in Current robot position */
  rrlib::math::tPose2D robot_pose_yaw = rrlib::math::tPose2D(0 , 0, robot_pose.Yaw()); //si_robot_pose.Get().
  //rrlib::math::tPose2D robot_pose_yaw = rrlib::math::tPose2D(robot_pose.X() , robot_pose.Y(), robot_pose.Yaw()); //si_robot_pose.Get().
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "robot_pose for canvas: ", robot_pose_yaw);
  output_canvas_temp->SetAlpha(200);
  output_canvas_temp->Transform(robot_pose_yaw);
  DrawRobot(*output_canvas_temp);

  so_canvas_gridmap_shift.Publish(output_canvas);
  so_gridmap_shift.Publish(gridmap_shift);
}

void mProcessedSensorDataToMapping::DrawRobot(rrlib::canvas::tCanvas2D& canvas)
{
  canvas.SetFill(true);
  canvas.SetEdgeColor(0, 0, 0);
  canvas.SetFillColor(200, 200, 255);

  rrlib::math::tVec2d robot_bottom_left = rrlib::math::tVec2d(-robot_height / 2, -robot_width / 2);

  rrlib::math::tVec2d robot_pose_arrow_start = rrlib::math::tVec2d(-robot_height / 2, 0.0);
  rrlib::math::tVec2d robot_pose_arrow_end   = rrlib::math::tVec2d(robot_height / 2, 0.0);

  canvas.DrawBox(robot_bottom_left, robot_height, robot_width);
  canvas.DrawArrow(robot_pose_arrow_start, robot_pose_arrow_end);
}

void mProcessedSensorDataToMapping::dist_cart_second(tSectorMapCartesian2D& sectormap_cartesian, rrlib::math::tVec2d position_obst)
{
  /*! projection of obstacle points to the sectormap*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;

  rrlib::mapping::tCartesian2D::tCoordinate coordinate(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y()); //scaled to 10

  double sectormap_width, sector_cell_width;

  double sector_cell_height_test = (2 * sector_cell_height);

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eFRONT_CARTESIAN)
  {
    sectormap_width = robot_width;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(-90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.X(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eREAR_CARTESIAN)
  {
    sectormap_width = robot_width;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(-robot_width, 0, 0, 0, 0, 0).GetTransformationMatrix();
  }

  rrlib::math::tMat4x4d matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst.X(), position_obst.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  rrlib::math::tVec2d position_obst_double = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

  vector<rrlib::math::tVec2d> position_obst_cell;
  position_obst_cell.push_back(position_obst_double);
//  rrlib::math::tVec2i position_obst_int = rrlib::math::tVec2i(matrix_obst[0][3], matrix_obst[1][3]);
//  position_obst_cell.push_back(position_obst_int);
//  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y()));
//  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y() + gridmap_cell_resolution));
//  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X(), position_obst_int.Y() + gridmap_cell_resolution));

  tSectorMapCartesian2D::tBounds bounds;
  bounds.lower_bounds[0] = coordinate;
  bounds.lower_bounds[1] = coordinate;
  coordinate += rrlib::mapping::tCartesian2D::tCoordinate(sectormap_width, sector_cell_height_test);
  bounds.upper_bounds[0] = coordinate;
  bounds.upper_bounds[1] = coordinate;
  sectormap_cartesian.SetBounds(bounds);
  coordinate = rrlib::mapping::tCartesian2D::tCoordinate(sector_cell_width, sector_cell_height_test);
  sectormap_cartesian.SetResolution(coordinate);

  /* first, draw the background of the map */
  tSectorMapCartesian2D& map = sectormap_cartesian;


  /* Iterating through all sectormap polar cells to load them with obstacle information */
  for (auto it = map.GridIteratorBegin();
       it != map.GridIteratorEnd();
       ++it)
  {
    auto bounds = map.GetBoundsByCellID((*it).index);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, bounds);

    rrlib::math::tVector<2, double, rrlib::math::vector::Cartesian> low_x = bounds.lower_bounds[0];

    for (auto iter = position_obst_cell.begin(); iter != position_obst_cell.end(); ++iter)
    {

      auto& position_origin_obst = *iter;

      if (position_origin_obst.X() > low_x.X()
          &&
          position_origin_obst.X() < low_x.X() + sector_cell_width)       //the obstacle is in that sectormap zone
      {
        if (position_origin_obst.Y() <  sector_cell_height_test
            &&
            position_origin_obst.Y() > 0) //map.origin.X()
        {

          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "LOW_X: ", low_x);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst: ", position_origin_obst);
          map.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.Y();
          map.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", map.GetCellByCellID((*it).index).distance_to_obstacle);
        } //if y limitation
      } //if x limitation


    }// iterating through the obst points corners

  }// for iterating in sector cells
}

void mProcessedSensorDataToMapping::DrawSectorCartesianTestSecond(rrlib::canvas::tCanvas2D& canvas, tSectorMapCartesian2D& sectormap_cartesian)
{
  /*! the sector cells arrow transformation*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;

  rrlib::mapping::tCartesian2D::tCoordinate coordinate(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y()); //scaled to 10

  double sectormap_width, sector_cell_width;

  double sector_cell_height_test = (2 * sector_cell_height);

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eFRONT_CARTESIAN)
  {
    sectormap_width = robot_width;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(-90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.X(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eREAR_CARTESIAN)
  {
    sectormap_width = robot_width;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(-robot_width, 0, 0, 0, 0, 0).GetTransformationMatrix();
  }

  tSectorMapCartesian2D::tBounds bounds;
  bounds.lower_bounds[0] = coordinate;
  bounds.lower_bounds[1] = coordinate;
  coordinate += rrlib::mapping::tCartesian2D::tCoordinate(sectormap_width, sector_cell_height_test);
  bounds.upper_bounds[0] = coordinate;
  bounds.upper_bounds[1] = coordinate;
  sectormap_cartesian.SetBounds(bounds);
  coordinate = rrlib::mapping::tCartesian2D::tCoordinate(sector_cell_width, sector_cell_height_test);
  sectormap_cartesian.SetResolution(coordinate);

  /* first, draw the background of the map */
  tSectorMapCartesian2D& map = sectormap_cartesian;

  /* now draw all the cell contents */
  canvas.SetFill(true);
  for (auto it = map.GridIteratorBegin();
       it != map.GridIteratorEnd();
       ++it)
  {

    auto bounds = map.GetBoundsByCellID((*it).index);
    RRLIB_LOG_PRINT(DEBUG_VERBOSE_1, "Bounds: ", bounds);

    rrlib::math::tVector<2, double, rrlib::math::vector::Cartesian> low_x = bounds.lower_bounds[0];

    rrlib::math::tMat4x4d matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X(), low_x.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d low_x_trans = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width, low_x.Y()          , 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d low_x_trans_2 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    /*! drawing the arrows to check the accuracy*/
    matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width / 2, low_x.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d sector_origin = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width / 2, low_x.Y() + sector_cell_height_test, 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d sector_target = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    if (map.GetCellByCellID((*it).index).distance_to_obstacle < sector_cell_height_test)
    {

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width, low_x.Y() + map.GetCellByCellID((*it).index).distance_to_obstacle, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_3 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X()           , low_x.Y() + map.GetCellByCellID((*it).index).distance_to_obstacle, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_4 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      canvas.SetFillColor(128, 0, 0); //red
      canvas.StartShape(low_x_trans);
      canvas.AppendLineSegment(low_x_trans_2);
      canvas.AppendLineSegment(low_x_trans_3);
      canvas.AppendLineSegment(low_x_trans_4);

      canvas.DrawArrow(sector_origin, map.GetCellByCellID((*it).index).obstacle_position);
    }
    else
    {
      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width, low_x.Y() + sector_cell_height_test, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_3 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X()           , low_x.Y() + sector_cell_height_test, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_4 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      canvas.SetFillColor(0, 255, 0); //green
      canvas.StartShape(low_x_trans);
      canvas.AppendLineSegment(low_x_trans_2);
      canvas.AppendLineSegment(low_x_trans_3);
      canvas.AppendLineSegment(low_x_trans_4);

      canvas.DrawArrow(sector_origin, sector_target);
    }


  }//iterating through the map
}

void mProcessedSensorDataToMapping::dist_polar_second(tSectorMapPolar2D& sectormap_polar, rrlib::math::tVec2d position_obst)
{
  /*! projection of obstacle points to the sectormap*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;
  matrix_canvas *= rrlib::math::tPose3D(sectormap_polar.origin.X(), sectormap_polar.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();



  /*! obst fromt original to sectormap space and then obst and cell corners*/
  rrlib::math::tMat4x4d matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst.X(), position_obst.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  rrlib::math::tVec2i position_obst_int = rrlib::math::tVec2i(matrix_obst[0][3], matrix_obst[1][3]);
  rrlib::math::tVec2d position_obst_double = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

  vector<rrlib::math::tVec2d> position_obst_cell;
  position_obst_cell.push_back(position_obst_double);
  position_obst_cell.push_back(position_obst_int);
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y()));
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y() + gridmap_cell_resolution));
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X(), position_obst_int.Y() + gridmap_cell_resolution));


//  /*! obst and cell corner fromt original to sectormap space*/
//    rrlib::math::tVec2d position_obst_double = rrlib::math::tVec2d(position_obst.X(), position_obst.Y());
//    rrlib::math::tVec2i position_obst_int = rrlib::math::tVec2i(position_obst_double.X(), position_obst_double.Y());
//    rrlib::math::tVec2i position_obst_int_1 = rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution , position_obst_int.Y());
//    rrlib::math::tVec2i position_obst_int_2 = rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution , position_obst_int.Y() + gridmap_cell_resolution);
//    rrlib::math::tVec2i position_obst_int_3 = rrlib::math::tVec2i(position_obst_int.X() , position_obst_int.Y() + gridmap_cell_resolution);
//
//  rrlib::math::tMat4x4d matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst_double.X(), position_obst_double.Y(), 0, 0, 0, 0).GetTransformationMatrix();
//  rrlib::math::tVec2d position_obst_double_trans = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);
//
//  matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst_int.X(), position_obst_int.Y(), 0, 0, 0, 0).GetTransformationMatrix();
//  rrlib::math::tVec2d position_obst_int_trans = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);
//
//  matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst_int_1.X(), position_obst_int_1.Y(), 0, 0, 0, 0).GetTransformationMatrix();
//  rrlib::math::tVec2d position_obst_int_1_trans = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);
//
//  matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst_int_2.X(), position_obst_int_2.Y(), 0, 0, 0, 0).GetTransformationMatrix();
//  rrlib::math::tVec2d position_obst_int_2_trans = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);
//
//  matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst_int_3.X(), position_obst_int_3.Y(), 0, 0, 0, 0).GetTransformationMatrix();
//  rrlib::math::tVec2d position_obst_int_3_trans = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);
//
//  vector<rrlib::math::tVec2d> position_obst_cell;
//  position_obst_cell.push_back(position_obst_double_trans);
//  position_obst_cell.push_back(position_obst_int_trans);
//  position_obst_cell.push_back(position_obst_int_1_trans);
//  position_obst_cell.push_back(position_obst_int_2_trans);
//  position_obst_cell.push_back(position_obst_int_3_trans);


  /*! sectormap stuff*/
  int starting_angle;
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_LEFT_POLAR_BIG)
  {
    starting_angle = 0;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_LEFT_POLAR_BIG)
  {
    starting_angle = 90;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_RIGHT_POLAR_BIG)
  {
    starting_angle = 180;
  }
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR_BIG)
  {
    starting_angle = -90 ; //270;
  }

  tSectorMapPolar2D::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(starting_angle, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(starting_angle + 90, (2 * sector_cell_height));
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  sectormap_polar.SetBounds(bounds_polar);
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(sector_polar_cell_angle, (2 * sector_cell_height)); //test to make sure about overwriting the legth of sectors
  sectormap_polar.SetResolution(coordinate_polar);


  /* Iterating through all sectormap polar cells to load them with obstacle information */
  for (auto it = sectormap_polar.GridIteratorBegin();
       it != sectormap_polar.GridIteratorEnd();
       ++it)
  {
    auto bounds = sectormap_polar.GetBoundsByCellID((*it).index);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, bounds);


    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> low_angle = bounds.lower_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> high_angle = bounds.upper_bounds[0];


    for (auto iter = position_obst_cell.begin(); iter != position_obst_cell.end(); ++iter)
    {

      auto& position_origin_obst = *iter;


      /*! if within the cell range and length smaller that 1*/
      if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_LEFT_POLAR_BIG)  //from 0 to 90 degree
      {

        if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(starting_angle)
            &&
            rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(starting_angle + 90))       //the obstacle is in that sectormap zone
        {
          if (position_origin_obst.GetPolarVector().Length() <= (2 * sector_cell_height))
          {
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar:", position_origin_obst.GetPolarVector());

            if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(low_angle.Alpha())
                &&
                rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(high_angle.Alpha()))    //with in one cell
            {
              sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
              sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
              FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
              FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
            }
          }
        }
      }

      if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR_BIG)  //from -90 to 0 degree
      {

        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "low angle:  ", low_angle, " high_angle:  ", high_angle, "  tAngleDegUnsigned(high_angle.Alpha()):  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "unsigned low angle:  ", rrlib::math::tAngleDegUnsigned(low_angle.Alpha()), " unsigned high_angle:  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "signed low angle:  ", rrlib::math::tAngleDegSigned(low_angle.Alpha()), " signed high_angle:  ", rrlib::math::tAngleDegSigned(high_angle.Alpha()));

        if (position_origin_obst.GetPolarVector().Length() <= (2 * sector_cell_height))
        {
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar FOR eFRONT_RIGHT_POLAR:", position_origin_obst.GetPolarVector());

          if (rrlib::math::tAngleDegSigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegSigned(low_angle.Alpha())
              &&
              rrlib::math::tAngleDegSigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegSigned(high_angle.Alpha()))    //with in one cell
          {
            sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
            sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
          }
        }
      }

      if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_LEFT_POLAR_BIG) //from 90 to 180 degree
      {

        /*! some debugging regarding the signed degree*/
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "low angle:  ", low_angle, " high_angle:  ", high_angle, "  tAngleDegUnsigned(high_angle.Alpha()):  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "  tAngleDegUnsigned(-90):  ", rrlib::math::tAngleDegUnsigned(-90));

        if (position_origin_obst.GetPolarVector().Length() <= (2 * sector_cell_height))
        {
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar FOR eREAR_LEFT_POLAR:", position_origin_obst.GetPolarVector());

          if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(low_angle.Alpha())
              &&
              rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(high_angle.Alpha()))    //with in one cell
          {
            sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
            sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
          }
        }
      }

      if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_RIGHT_POLAR_BIG) //from 180 to 270 degree
      {

        /*! some debugging regarding the signed degree*/
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "low angle:  ", low_angle, " high_angle:  ", high_angle);
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "  tAngleDegUnsigned(-90):  ", rrlib::math::tAngleDegUnsigned(-90));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "unsigned low angle:  ", rrlib::math::tAngleDegUnsigned(low_angle.Alpha()), " unsigned high_angle:  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "signed low angle:  ", rrlib::math::tAngleDegSigned(low_angle.Alpha()), " signed high_angle:  ", rrlib::math::tAngleDegSigned(high_angle.Alpha()));


        if (position_origin_obst.GetPolarVector().Length() <= (2 * sector_cell_height))
        {
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar FOR eREAR_LEFT_POLAR:", position_origin_obst.GetPolarVector());

          if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(low_angle.Alpha())
              &&
              rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(high_angle.Alpha()))    //with in one cell
          {
            sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
            sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
          }
        }
      }

    }// iterating through all obstacle point corners

  }// for iterating in sector cells
}

void mProcessedSensorDataToMapping::DrawSectorPolarTestSecond(rrlib::canvas::tCanvas2D& canvas, tSectorMapPolar2D& sectormap_polar)
{

  /*! the sector cells arrow transformation*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;
  matrix_canvas *= rrlib::math::tPose3D(sectormap_polar.origin.X(), sectormap_polar.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();


  /*! sectormap stuff*/
  int starting_angle;
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_LEFT_POLAR_BIG)
  {
    starting_angle = 0;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_LEFT_POLAR_BIG)
  {
    starting_angle = 90;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_RIGHT_POLAR_BIG)
  {
    starting_angle = 180;
  }
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR_BIG)
  {
    starting_angle = 270;
  }

  tSectorMapPolar2D::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(starting_angle, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(starting_angle + 90, (2 * sector_cell_height));
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  sectormap_polar.SetBounds(bounds_polar);
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(sector_polar_cell_angle, (2 * sector_cell_height)); //test to make sure about overwriting the legth of sectors
  sectormap_polar.SetResolution(coordinate_polar);

  /* Draw sectormap polar cells */
  /* first, draw the background of the map */
  canvas.SetFillColor(0, 255, 0);
  canvas.SetFill(true);
  for (auto it = sectormap_polar.GridIteratorBegin();
       it != sectormap_polar.GridIteratorEnd();
       ++it)
  {
    auto bounds = sectormap_polar.GetBoundsByCellID((*it).index);

    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> low_angle = bounds.lower_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> low_length = bounds.lower_bounds[1];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> high_angle = bounds.upper_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> high_length = bounds.upper_bounds[1];

    if (sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle < (2 * sector_cell_height))
    {
      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  first_corner;
      first_corner.Length() = low_length.Length();
      first_corner[0] = rrlib::math::tAngleDegUnsigned(low_angle[0]);

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  second_corner;
      second_corner.Length() = sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle; //high_length.Length();
      second_corner[0] = rrlib::math::tAngleDegUnsigned(low_angle[0]);

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  third_corner;
      third_corner.Length() = sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle; //high_length.Length();
      third_corner[0] = rrlib::math::tAngleDegUnsigned(high_angle[0]);

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  fourth_corner;
      fourth_corner.Length() = low_length.Length();
      fourth_corner[0] = rrlib::math::tAngleDegUnsigned(high_angle[0]);

      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "fisrt corner: ", first_corner);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "2nd corner: ", second_corner);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "3d corner: ", third_corner);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "4th corner: ", fourth_corner);

      /*! the sector cell arrow*/
      rrlib::math::tMat4x4d matrix_rear = matrix_canvas
                                          *  rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                          * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_rear = rrlib::math::tVec2d(matrix_rear[0][3], matrix_rear[1][3]);

      rrlib::math::tMat4x4d matrix_corners = matrix_canvas * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_first = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(second_corner.GetCartesianVector().X(), second_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_second = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(third_corner.GetCartesianVector().X(), third_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_third = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(fourth_corner.GetCartesianVector().X() , fourth_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_fourth = rrlib::math::tVec2d(matrix_corners[0][3] , matrix_corners[1][3]);

      canvas.SetFillColor(128, 0, 0);
      canvas.DrawPolygon(position_first, position_second, position_third, position_fourth);

      canvas.DrawArrow(position_rear.X(), position_rear.Y(), sectormap_polar.GetCellByCellID((*it).index).obstacle_position.X(), sectormap_polar.GetCellByCellID((*it).index).obstacle_position.Y(), false);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "distance to obstacle: ", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position of obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "cell ID: ", sectormap_polar.GetBoundsByCellID((*it).index));

    }
    else     if (sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle > (2 * sector_cell_height))
    {
      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  first_corner;
      first_corner.Length() = low_length.Length();
      first_corner[0] = rrlib::math::tAngleDegUnsigned(low_angle[0]);

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  second_corner;
      second_corner.Length() = high_length.Length();
      second_corner[0] = rrlib::math::tAngleDegUnsigned(low_angle[0]);

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  third_corner;
      third_corner.Length() = high_length.Length();
      third_corner[0] = rrlib::math::tAngleDegUnsigned(high_angle[0]);

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  fourth_corner;
      fourth_corner.Length() = low_length.Length();
      fourth_corner[0] = rrlib::math::tAngleDegUnsigned(high_angle[0]);

      //canvas.DrawPolygon(first_corner.GetCartesianVector(), second_corner.GetCartesianVector(), third_corner.GetCartesianVector(), fourth_corner.GetCartesianVector());


      /*! the sector cell arrow*/
      rrlib::math::tMat4x4d matrix_rear = matrix_canvas
                                          *  rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                          * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_rear = rrlib::math::tVec2d(matrix_rear[0][3], matrix_rear[1][3]);

      rrlib::math::tMat4x4d matrix_front = matrix_canvas
                                           *  rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                           * rrlib::math::tPose3D(second_corner.GetCartesianVector().X() , second_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_front = rrlib::math::tVec2d(matrix_front[0][3] , matrix_front[1][3]);

      rrlib::math::tMat4x4d matrix_corners = matrix_canvas * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_first = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(second_corner.GetCartesianVector().X(), second_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_second = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(third_corner.GetCartesianVector().X(), third_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_third = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(fourth_corner.GetCartesianVector().X() , fourth_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_fourth = rrlib::math::tVec2d(matrix_corners[0][3] , matrix_corners[1][3]);

      canvas.SetFillColor(0, 255, 0);
      canvas.DrawPolygon(position_first, position_second, position_third, position_fourth);

      canvas.DrawArrow(position_rear.X(), position_rear.Y(), position_front.X(), position_front.Y(), false);
    }

  }// for iterating in sector cells
}

void mProcessedSensorDataToMapping::DrawSectorCartesianTest(rrlib::canvas::tCanvas2D& canvas, tSectorMapCartesian2D& sectormap_cartesian)
{
  /*! the sector cells arrow transformation*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;

  rrlib::mapping::tCartesian2D::tCoordinate coordinate(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y()); //scaled to 10

  double sectormap_width, sector_cell_width;
  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eLEFT_REAR_CARTESIAN)
  {
    sectormap_width = robot_height / 2;
    sector_cell_width = robot_height / 20;
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X() + (robot_height / 2), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eLEFT_FRONT_CARTESIAN)
  {
    sectormap_width = robot_height / 2;
    sector_cell_width = robot_height / 20;
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eFRONT_LEFT_CARTESIAN
      ||
      sectormap_cartesian.position == mapping::tSectorMapPosition::eFRONT_RIGHT_CARTESIAN)
  {
    sectormap_width = robot_width / 2;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(-90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.X(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eRIGHT_REAR_CARTESIAN
      ||
      sectormap_cartesian.position == mapping::tSectorMapPosition::eRIGHT_FRONT_CARTESIAN)
  {
    sectormap_width = robot_height / 2;
    sector_cell_width = robot_height / 20;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(-180)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(-robot_height / 2, 0, 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eREAR_LEFT_CARTESIAN
      ||
      sectormap_cartesian.position == mapping::tSectorMapPosition::eREAR_RIGHT_CARTESIAN)
  {
    sectormap_width = robot_width / 2;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(-robot_width / 2, 0, 0, 0, 0, 0).GetTransformationMatrix();
  }

  tSectorMapCartesian2D::tBounds bounds;
  bounds.lower_bounds[0] = coordinate;
  bounds.lower_bounds[1] = coordinate;
  coordinate += rrlib::mapping::tCartesian2D::tCoordinate(sectormap_width, sector_cell_height);
  bounds.upper_bounds[0] = coordinate;
  bounds.upper_bounds[1] = coordinate;
  sectormap_cartesian.SetBounds(bounds);
  coordinate = rrlib::mapping::tCartesian2D::tCoordinate(sector_cell_width, sector_cell_height);
  sectormap_cartesian.SetResolution(coordinate);

  /* first, draw the background of the map */
  tSectorMapCartesian2D& map = sectormap_cartesian;

  /* now draw all the cell contents */
  canvas.SetFill(true);
  for (auto it = map.GridIteratorBegin();
       it != map.GridIteratorEnd();
       ++it)
  {

    auto bounds = map.GetBoundsByCellID((*it).index);
    RRLIB_LOG_PRINT(DEBUG_VERBOSE_1, "Bounds: ", bounds);

    rrlib::math::tVector<2, double, rrlib::math::vector::Cartesian> low_x = bounds.lower_bounds[0];

    rrlib::math::tMat4x4d matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X(), low_x.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d low_x_trans = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width, low_x.Y()          , 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d low_x_trans_2 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    /*! drawing the arrows to check the accuracy*/
    matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width / 2, low_x.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d sector_origin = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width / 2, low_x.Y() + sector_cell_height, 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d sector_target = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

    if (map.GetCellByCellID((*it).index).distance_to_obstacle < sector_cell_height)
    {

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width, low_x.Y() + map.GetCellByCellID((*it).index).distance_to_obstacle, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_3 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X()           , low_x.Y() + map.GetCellByCellID((*it).index).distance_to_obstacle, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_4 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      canvas.SetFillColor(128, 0, 0); //red
      canvas.StartShape(low_x_trans);
      canvas.AppendLineSegment(low_x_trans_2);
      canvas.AppendLineSegment(low_x_trans_3);
      canvas.AppendLineSegment(low_x_trans_4);

      canvas.DrawArrow(sector_origin, map.GetCellByCellID((*it).index).obstacle_position);
    }
    else
    {
      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X() + sector_cell_width, low_x.Y() + sector_cell_height, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_3 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(low_x.X()           , low_x.Y() + sector_cell_height, 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d low_x_trans_4 = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      canvas.SetFillColor(0, 255, 0); //green
      canvas.StartShape(low_x_trans);
      canvas.AppendLineSegment(low_x_trans_2);
      canvas.AppendLineSegment(low_x_trans_3);
      canvas.AppendLineSegment(low_x_trans_4);

      canvas.DrawArrow(sector_origin, sector_target);
    }


  }//iterating through the map
}

void mProcessedSensorDataToMapping::dist_cart(tSectorMapCartesian2D& sectormap_cartesian, rrlib::math::tVec2d position_obst)
{
  /*! projection of obstacle points to the sectormap*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;

  rrlib::mapping::tCartesian2D::tCoordinate coordinate(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y()); //scaled to 10

  double sectormap_width, sector_cell_width;
  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eLEFT_REAR_CARTESIAN)
  {
    sectormap_width = robot_height / 2;
    sector_cell_width = robot_height / 20;
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X() + (robot_height / 2), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eLEFT_FRONT_CARTESIAN)
  {
    sectormap_width = robot_height / 2;
    sector_cell_width = robot_height / 20;
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eFRONT_LEFT_CARTESIAN
      ||
      sectormap_cartesian.position == mapping::tSectorMapPosition::eFRONT_RIGHT_CARTESIAN)
  {
    sectormap_width = robot_width / 2;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(-90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, sectormap_cartesian.origin.X(), 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eRIGHT_REAR_CARTESIAN
      ||
      sectormap_cartesian.position == mapping::tSectorMapPosition::eRIGHT_FRONT_CARTESIAN)
  {
    sectormap_width = robot_height / 2;
    sector_cell_width = robot_height / 20;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(-180)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(-robot_height / 2, 0, 0, 0, 0, 0).GetTransformationMatrix();
  }

  if (sectormap_cartesian.position == mapping::tSectorMapPosition::eREAR_LEFT_CARTESIAN
      ||
      sectormap_cartesian.position == mapping::tSectorMapPosition::eREAR_RIGHT_CARTESIAN)
  {
    sectormap_width = robot_width / 2;
    sector_cell_width = robot_width / 10;
    coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0, 0); //scaled to 10
    matrix_canvas *= rrlib::math::tPose3D(sectormap_cartesian.origin.X(), sectormap_cartesian.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(90)).GetTransformationMatrix();
    matrix_canvas *= rrlib::math::tPose3D(-robot_width / 2, 0, 0, 0, 0, 0).GetTransformationMatrix();
  }

  rrlib::math::tMat4x4d matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst.X(), position_obst.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  rrlib::math::tVec2i position_obst_int = rrlib::math::tVec2i(matrix_obst[0][3], matrix_obst[1][3]);
  rrlib::math::tVec2d position_obst_double = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

  vector<rrlib::math::tVec2d> position_obst_cell;
  position_obst_cell.push_back(position_obst_double);
  position_obst_cell.push_back(position_obst_int);
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y()));
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y() + gridmap_cell_resolution));
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X(), position_obst_int.Y() + gridmap_cell_resolution));

  tSectorMapCartesian2D::tBounds bounds;
  bounds.lower_bounds[0] = coordinate;
  bounds.lower_bounds[1] = coordinate;
  coordinate += rrlib::mapping::tCartesian2D::tCoordinate(sectormap_width, sector_cell_height);
  bounds.upper_bounds[0] = coordinate;
  bounds.upper_bounds[1] = coordinate;
  sectormap_cartesian.SetBounds(bounds);
  coordinate = rrlib::mapping::tCartesian2D::tCoordinate(sector_cell_width, sector_cell_height);
  sectormap_cartesian.SetResolution(coordinate);

  /* first, draw the background of the map */
  tSectorMapCartesian2D& map = sectormap_cartesian;


  /* Iterating through all sectormap polar cells to load them with obstacle information */
  for (auto it = map.GridIteratorBegin();
       it != map.GridIteratorEnd();
       ++it)
  {
    auto bounds = map.GetBoundsByCellID((*it).index);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, bounds);

    rrlib::math::tVector<2, double, rrlib::math::vector::Cartesian> low_x = bounds.lower_bounds[0];

    for (auto iter = position_obst_cell.begin(); iter != position_obst_cell.end(); ++iter)
    {

      auto& position_origin_obst = *iter;

      if (position_origin_obst.X() > low_x.X()
          &&
          position_origin_obst.X() < low_x.X() + sector_cell_width)       //the obstacle is in that sectormap zone
      {
        if (position_origin_obst.Y() <  sector_cell_height
            &&
            position_origin_obst.Y() > 0) //map.origin.X()
        {

          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "LOW_X: ", low_x);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst: ", position_origin_obst);
          map.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.Y();
          map.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", map.GetCellByCellID((*it).index).distance_to_obstacle);
        } //if y limitation
      } //if x limitation


    }// iterating through the obst points corners

  }// for iterating in sector cells
}

void mProcessedSensorDataToMapping::dist(tSectorMapPolar2D& sectormap_polar, rrlib::math::tVec2d position_obst)
{
  /*! projection of obstacle points to the sectormap*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;
  matrix_canvas *= rrlib::math::tPose3D(sectormap_polar.origin.X(), sectormap_polar.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();

  rrlib::math::tMat4x4d matrix_obst = matrix_canvas.Inverse() * rrlib::math::tPose3D(position_obst.X(), position_obst.Y(), 0, 0, 0, 0).GetTransformationMatrix();
  rrlib::math::tVec2i position_obst_int = rrlib::math::tVec2i(matrix_obst[0][3], matrix_obst[1][3]);
  rrlib::math::tVec2d position_obst_double = rrlib::math::tVec2d(matrix_obst[0][3], matrix_obst[1][3]);

  vector<rrlib::math::tVec2d> position_obst_cell;
  position_obst_cell.push_back(position_obst_double);
  position_obst_cell.push_back(position_obst_int);
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y()));
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X() + gridmap_cell_resolution, position_obst_int.Y() + gridmap_cell_resolution));
  position_obst_cell.push_back(rrlib::math::tVec2i(position_obst_int.X(), position_obst_int.Y() + gridmap_cell_resolution));


  /*! sectormap stuff*/
  int starting_angle;
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_LEFT_POLAR)
  {
    starting_angle = 0;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_LEFT_POLAR)
  {
    starting_angle = 90;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_RIGHT_POLAR)
  {
    starting_angle = 180;
  }
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR)
  {
    starting_angle = -90 ; //270;
  }

  tSectorMapPolar2D::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(starting_angle, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar += rrlib::mapping::tPolar2D::tCoordinate(starting_angle + 90, sector_cell_height);
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  sectormap_polar.SetBounds(bounds_polar);
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(sector_polar_cell_angle, sector_cell_height); //test to make sure about overwriting the legth of sectors
  sectormap_polar.SetResolution(coordinate_polar);


  /* Iterating through all sectormap polar cells to load them with obstacle information */
  for (auto it = sectormap_polar.GridIteratorBegin();
       it != sectormap_polar.GridIteratorEnd();
       ++it)
  {
    auto bounds = sectormap_polar.GetBoundsByCellID((*it).index);
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, bounds);


    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> low_angle = bounds.lower_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> high_angle = bounds.upper_bounds[0];


    for (auto iter = position_obst_cell.begin(); iter != position_obst_cell.end(); ++iter)
    {

      auto& position_origin_obst = *iter;


      /*! if within the cell range and length smaller that 1*/
      if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_LEFT_POLAR)  //from 0 to 90 degree
      {

        if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(starting_angle)
            &&
            rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(starting_angle + 90))       //the obstacle is in that sectormap zone
        {
          if (position_origin_obst.GetPolarVector().Length() <=  sector_cell_height)
          {
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar:", position_origin_obst.GetPolarVector());

            if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(low_angle.Alpha())
                &&
                rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(high_angle.Alpha()))    //with in one cell
            {
              sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
              sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
              FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
              FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
            }
          }
        }
      }

      if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR)  //from -90 to 0 degree
      {

        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "low angle:  ", low_angle, " high_angle:  ", high_angle, "  tAngleDegUnsigned(high_angle.Alpha()):  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "unsigned low angle:  ", rrlib::math::tAngleDegUnsigned(low_angle.Alpha()), " unsigned high_angle:  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "signed low angle:  ", rrlib::math::tAngleDegSigned(low_angle.Alpha()), " signed high_angle:  ", rrlib::math::tAngleDegSigned(high_angle.Alpha()));

        if (position_origin_obst.GetPolarVector().Length() <=  sector_cell_height)
        {
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar FOR eFRONT_RIGHT_POLAR:", position_origin_obst.GetPolarVector());

          if (rrlib::math::tAngleDegSigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegSigned(low_angle.Alpha())
              &&
              rrlib::math::tAngleDegSigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegSigned(high_angle.Alpha()))    //with in one cell
          {
            sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
            sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
          }
        }
      }

      if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_LEFT_POLAR) //from 90 to 180 degree
      {

        /*! some debugging regarding the signed degree*/
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "low angle:  ", low_angle, " high_angle:  ", high_angle, "  tAngleDegUnsigned(high_angle.Alpha()):  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "  tAngleDegUnsigned(-90):  ", rrlib::math::tAngleDegUnsigned(-90));

        if (position_origin_obst.GetPolarVector().Length() <=  sector_cell_height)
        {
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar FOR eREAR_LEFT_POLAR:", position_origin_obst.GetPolarVector());

          if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(low_angle.Alpha())
              &&
              rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(high_angle.Alpha()))    //with in one cell
          {
            sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
            sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
          }
        }
      }

      if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_RIGHT_POLAR) //from 180 to 270 degree
      {

        /*! some debugging regarding the signed degree*/
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "low angle:  ", low_angle, " high_angle:  ", high_angle);
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "  tAngleDegUnsigned(-90):  ", rrlib::math::tAngleDegUnsigned(-90));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "unsigned low angle:  ", rrlib::math::tAngleDegUnsigned(low_angle.Alpha()), " unsigned high_angle:  ", rrlib::math::tAngleDegUnsigned(high_angle.Alpha()));
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "signed low angle:  ", rrlib::math::tAngleDegSigned(low_angle.Alpha()), " signed high_angle:  ", rrlib::math::tAngleDegSigned(high_angle.Alpha()));


        if (position_origin_obst.GetPolarVector().Length() <=  sector_cell_height)
        {
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position obst polar FOR eREAR_LEFT_POLAR:", position_origin_obst.GetPolarVector());

          if (rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) > rrlib::math::tAngleDegUnsigned(low_angle.Alpha())
              &&
              rrlib::math::tAngleDegUnsigned(position_origin_obst.GetPolarVector().Alpha()) < rrlib::math::tAngleDegUnsigned(high_angle.Alpha()))    //with in one cell
          {
            sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle = position_origin_obst.GetPolarVector().Length();
            sectormap_polar.GetCellByCellID((*it).index).obstacle_position = position_obst; //this is only for test
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obst distance", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position.GetPolarVector());
          }
        }
      }

    }// iterating through all obstacle point corners

  }// for iterating in sector cells
}

void mProcessedSensorDataToMapping::DrawSectorPolarTest(rrlib::canvas::tCanvas2D& canvas, tSectorMapPolar2D& sectormap_polar)
{

  /*! the sector cells arrow transformation*/
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();
  //rrlib::math::tMat3x3d rotation = distance_data->RobotPose().GetRotationMatrix();
  rrlib::math::tMat3x3d rotation = robot_pose.GetRotationMatrix();

  rrlib::math::tMat4x4d matrix_rot = rrlib::math::tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], 0,
                                     rotation[1][0], rotation[1][1], rotation[1][2], 0,
                                     rotation[2][0], rotation[2][1], rotation[2][2], 0,
                                     0, 0, 0, 1);

  rrlib::math::tMat4x4d matrix_canvas = matrix_rot;
  matrix_canvas *= rrlib::math::tPose3D(sectormap_polar.origin.X(), sectormap_polar.origin.Y(), 0, 0, 0, 0).GetTransformationMatrix();


  /*! sectormap stuff*/
  int starting_angle;
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_LEFT_POLAR)
  {
    starting_angle = 0;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_LEFT_POLAR)
  {
    starting_angle = 90;
  }

  if (sectormap_polar.position == mapping::tSectorMapPosition::eREAR_RIGHT_POLAR)
  {
    starting_angle = 180;
  }
  if (sectormap_polar.position == mapping::tSectorMapPosition::eFRONT_RIGHT_POLAR)
  {
    starting_angle = 270;
  }

  tSectorMapPolar2D::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(starting_angle, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(starting_angle + 90, sector_cell_height);
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  sectormap_polar.SetBounds(bounds_polar);
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(sector_polar_cell_angle, sector_cell_height / 10); //test to make sure about overwriting the legth of sectors
  sectormap_polar.SetResolution(coordinate_polar);

  /* Draw sectormap polar cells */
  /* first, draw the background of the map */
  canvas.SetFillColor(0, 255, 0);
  canvas.SetFill(true);
  for (auto it = sectormap_polar.GridIteratorBegin();
       it != sectormap_polar.GridIteratorEnd();
       ++it)
  {
    auto bounds = sectormap_polar.GetBoundsByCellID((*it).index);

    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed> low_angle = bounds.lower_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed> low_length = bounds.lower_bounds[1];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed> high_angle = bounds.upper_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed> high_length = bounds.upper_bounds[1];

    if (sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle < sector_cell_height)
    {
      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  first_corner;
      first_corner.Length() = low_length.Length();
      first_corner[0] = low_angle[0];

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  second_corner;
      second_corner.Length() = sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle; //high_length.Length();
      second_corner[0] = low_angle[0];

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  third_corner;
      third_corner.Length() = sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle; //high_length.Length();
      third_corner[0] = high_angle[0];

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  fourth_corner;
      fourth_corner.Length() = low_length.Length();
      fourth_corner[0] = high_angle[0];

      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "fisrt corner: ", first_corner);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "2nd corner: ", second_corner);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "3d corner: ", third_corner);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "4th corner: ", fourth_corner);

      /*! the sector cell arrow*/
      rrlib::math::tMat4x4d matrix_rear = matrix_canvas
                                          *  rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                          * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_rear = rrlib::math::tVec2d(matrix_rear[0][3], matrix_rear[1][3]);

      rrlib::math::tMat4x4d matrix_corners = matrix_canvas * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_first = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(second_corner.GetCartesianVector().X(), second_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_second = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(third_corner.GetCartesianVector().X(), third_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_third = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(fourth_corner.GetCartesianVector().X() , fourth_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_fourth = rrlib::math::tVec2d(matrix_corners[0][3] , matrix_corners[1][3]);

      canvas.SetFillColor(128, 0, 0);
      canvas.DrawPolygon(position_first, position_second, position_third, position_fourth);

      canvas.DrawArrow(position_rear.X(), position_rear.Y(), sectormap_polar.GetCellByCellID((*it).index).obstacle_position.X(), sectormap_polar.GetCellByCellID((*it).index).obstacle_position.Y(), false);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "distance to obstacle: ", sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "position of obstacle: ", sectormap_polar.GetCellByCellID((*it).index).obstacle_position);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "cell ID: ", sectormap_polar.GetBoundsByCellID((*it).index));

    }
    else     if (sectormap_polar.GetCellByCellID((*it).index).distance_to_obstacle > sector_cell_height)
    {
      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  first_corner;
      first_corner.Length() = low_length.Length();
      first_corner[0] = low_angle[0];

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  second_corner;
      second_corner.Length() = high_length.Length();
      second_corner[0] = low_angle[0];

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  third_corner;
      third_corner.Length() = high_length.Length();
      third_corner[0] = high_angle[0];

      rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Signed>  fourth_corner;
      fourth_corner.Length() = low_length.Length();
      fourth_corner[0] = high_angle[0];

      //canvas.DrawPolygon(first_corner.GetCartesianVector(), second_corner.GetCartesianVector(), third_corner.GetCartesianVector(), fourth_corner.GetCartesianVector());


      /*! the sector cell arrow*/
      rrlib::math::tMat4x4d matrix_rear = matrix_canvas
                                          *  rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                          * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_rear = rrlib::math::tVec2d(matrix_rear[0][3], matrix_rear[1][3]);

      rrlib::math::tMat4x4d matrix_front = matrix_canvas
                                           *  rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                           * rrlib::math::tPose3D(second_corner.GetCartesianVector().X() , second_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_front = rrlib::math::tVec2d(matrix_front[0][3] , matrix_front[1][3]);

      rrlib::math::tMat4x4d matrix_corners = matrix_canvas * rrlib::math::tPose3D(first_corner.GetCartesianVector().X(), first_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_first = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(second_corner.GetCartesianVector().X(), second_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_second = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(third_corner.GetCartesianVector().X(), third_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_third = rrlib::math::tVec2d(matrix_corners[0][3], matrix_corners[1][3]);

      matrix_corners = matrix_canvas * rrlib::math::tPose3D(fourth_corner.GetCartesianVector().X() , fourth_corner.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_fourth = rrlib::math::tVec2d(matrix_corners[0][3] , matrix_corners[1][3]);

      canvas.SetFillColor(0, 255, 0);
      canvas.DrawPolygon(position_first, position_second, position_third, position_fourth);

      canvas.DrawArrow(position_rear.X(), position_rear.Y(), position_front.X(), position_front.Y(), false);
    }

  }// for iterating in sector cells
}

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping constructor
//----------------------------------------------------------------------
mProcessedSensorDataToMapping::mProcessedSensorDataToMapping(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name)
{
  /*---------------------- mapping the environment ---------------------*/
  gridmap_shift_bounds.upper_bounds[0] = rrlib::math::tVec2d(gridmap_bound_limit, 0);
  gridmap_shift_bounds.upper_bounds[1] = rrlib::math::tVec2d(0, gridmap_bound_limit);
  gridmap_shift_bounds.lower_bounds[0] = rrlib::math::tVec2d(-gridmap_bound_limit, 0);
  gridmap_shift_bounds.lower_bounds[1] = rrlib::math::tVec2d(0, -gridmap_bound_limit);

  /*------------------ shift gridmap using front scanner --------------------------*/
  gridmap_shift.SetBounds(gridmap_shift_bounds);
  gridmap_shift.SetResolution(rrlib::math::tVec2d(gridmap_cell_resolution, gridmap_cell_resolution)); // resolution: 1 meter, 1 meter

  //map_obst_all.resize(gridmap_bound_limit*gridmap_bound_limit);
  data_obst_all.resize(gridmap_shift.GetNumberOfCells());
  data_obst_all.clear();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "MAP_OBST_ALL.SIZE:  ", data_obst_all.size());
}// constructor

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping destructor
//----------------------------------------------------------------------
mProcessedSensorDataToMapping::~mProcessedSensorDataToMapping() {}

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping OnStaticParameterChange
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::OnStaticParameterChange() {}

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping OnParameterChange
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::OnParameterChange() {}

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
    GridmapShiftable();
  } //if

}// sense

//----------------------------------------------------------------------
// mProcessedSensorDataToMapping Control
//----------------------------------------------------------------------
void mProcessedSensorDataToMapping::Control()
{

  /*! reseting the gridmap for environment - the main one */
  if (ci_reset_gridmap_shift.Get() == 1)
  {
    gridmap_shift.Clear(0);
    data_obst_all.clear();
  }// if ci

}// control

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

