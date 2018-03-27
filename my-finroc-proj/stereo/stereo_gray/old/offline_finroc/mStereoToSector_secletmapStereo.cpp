
#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc/mStereoToSectorMap.h"
#include "rrlib/math/tPose3D.h"
//#include <algorithm>

using namespace finroc::icarus::sensor_processing::stereo_gray::offline;

typedef rrlib::mapping::tMapGridPolar2D<double> tMap;
typedef tMap::tCoordinate tVec;

const double sector_cell_height = 30;
const double sector_polar_cell_angle = 6; //9;
const double sector_lower_bound_angle = 45; //0;
const double sector_upper_bound_angle = 135; //359;

void drawDirectionArrow(rrlib::canvas::tCanvas2D& canvas, const rrlib::mapping::tMapGridPolar2D<double>& map);

void mStereoToSectorMap::secletmapStereo()
{

  /* --------------> copy the gridmap <------------------------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas = input_canvas_secletmap.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas = output_canvas_secletmap.GetUnusedBuffer();

  rrlib::canvas::tCanvas2D& temp = *output_canvas;
  memcpy(&temp, input_canvas, sizeof(rrlib::canvas::tCanvas2D));

  data_ports::tPortDataPointer<const rrlib::mapping::tMapGridPolar2D<double>> input_secletmap_ptr = input_secletmap.GetPointer();
  drawDirectionArrow(*output_canvas, *input_secletmap_ptr);

  output_canvas_secletmap.Publish(output_canvas);

}

void drawDirectionArrow(rrlib::canvas::tCanvas2D& canvas, const rrlib::mapping::tMapGridPolar2D<double>& map)
{

  const unsigned numSectors = (sector_upper_bound_angle - sector_lower_bound_angle) / sector_polar_cell_angle;
  rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> max_array[numSectors];
  rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> min_array[numSectors];

  // this is a vector to define the best direction arrow base on number of green cells
//    std::vector<unsigned> numGreenCells_eachSector;
  unsigned numGreenCells_array[numSectors];
  std::memset(numGreenCells_array, 0, sizeof(unsigned) * numSectors); //*/

  for (unsigned i = 0; i < numSectors; ++i)
  {

    unsigned angle = sector_lower_bound_angle + (i * sector_polar_cell_angle);

    max_array[i].Alpha() = rrlib::math::tAngleDegUnsigned(0);
    max_array[i].Length() = 0;
    min_array[i].Alpha() = rrlib::math::tAngleDegUnsigned(0);
    min_array[i].Length() = 0;

    bool minFound = false, maxFound = false;
    unsigned numGreenCells = 0;

    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> min;
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> max;

    for (unsigned r = 0; r < sector_cell_height; r++)
    {
      if (map.GetConstCellByCoordinate(tVec(rrlib::math::tAngleDegUnsigned(angle), r)) == 1 &&
          !minFound &&
          numGreenCells == 0)
      {
        minFound = true;
        numGreenCells ++;
        min.Alpha() = rrlib::math::tAngleDegUnsigned(angle);
        min.Length() = r;
        min_array[i].Alpha() = min.Alpha();
        min_array[i].Length() = min.Length();
      }
      if (minFound &&
          !maxFound &&
          numGreenCells > 0)
      {
        if (map.GetConstCellByCoordinate(tVec(rrlib::math::tAngleDegUnsigned(angle), r)) == 1)
        {
          numGreenCells ++;
        }
        else  //if the cell turns into white or not green anymore
        {
          maxFound = true;
          max.Alpha() = min.Alpha();
          max.Length() = r;
          max_array[i].Alpha() = max.Alpha();
          max_array[i].Length() = max.Length();
          numGreenCells_array[i] = numGreenCells;
        }
      }
    }// for radius


    /*
    //    if (numGreenCells > 0)
    //    {
    //      rrlib::math::tMat4x4d matrix_rear = rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegUnsigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
    //                                          * rrlib::math::tPose3D(min_array[i].GetCartesianVector().X(), min_array[i].GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
    //      rrlib::math::tVec2d position_rear = rrlib::math::tVec2d(matrix_rear[0][3], matrix_rear[1][3]);
    //
    //      rrlib::math::tMat4x4d matrix_front = rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
    //                                           * rrlib::math::tPose3D(max_array[i].GetCartesianVector().X() , max_array[i].GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
    //      rrlib::math::tVec2d position_front = rrlib::math::tVec2d(matrix_front[0][3] , matrix_front[1][3]);
    //
    //      canvas.DrawArrow(position_rear, position_front, false);
    //    }
    */

  }// for angle

  unsigned max_numGreenCells = *std::max_element(numGreenCells_array, numGreenCells_array + numSectors);
  double sum_alpha = 0.0, num = 0.0, sum_length = 0.0;

  for (unsigned i = 0; i < numSectors; ++i)
  {

    if (numGreenCells_array[i] == max_numGreenCells && max_numGreenCells > 0)
    {
      rrlib::math::tMat4x4d matrix_rear = rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegUnsigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                          * rrlib::math::tPose3D(min_array[i].GetCartesianVector().X(), min_array[i].GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_rear = rrlib::math::tVec2d(matrix_rear[0][3], matrix_rear[1][3]);

      rrlib::math::tMat4x4d matrix_front = rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
                                           * rrlib::math::tPose3D(max_array[i].GetCartesianVector().X() , max_array[i].GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
      rrlib::math::tVec2d position_front = rrlib::math::tVec2d(matrix_front[0][3] , matrix_front[1][3]);

      canvas.DrawArrow(position_rear, position_front, false);


      // the overall direction from all the sectors for measuring the average direction
      sum_alpha += max_array[i].Alpha();
      sum_length += max_array[i].Length();
      num ++;
    }

  }

  rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> average;
  average.Alpha() = rrlib::math::tAngleDegUnsigned(sum_alpha / num);
  average.Length() = sum_length / num;

  if (max_numGreenCells)
  {
    rrlib::math::tMat4x4d matrix_front_average = rrlib::math::tPose3D(0, 0, 0, 0, 0, rrlib::math::tAngleDegSigned(sector_polar_cell_angle / 2)).GetTransformationMatrix()
        * rrlib::math::tPose3D(average.GetCartesianVector().X() , average.GetCartesianVector().Y(), 0, 0, 0, 0).GetTransformationMatrix();
    rrlib::math::tVec2d position_front_average = rrlib::math::tVec2d(matrix_front_average[0][3] , matrix_front_average[1][3]);

    canvas.DrawArrow(rrlib::math::tVec2d(0, 0), position_front_average, false);
  }
}
