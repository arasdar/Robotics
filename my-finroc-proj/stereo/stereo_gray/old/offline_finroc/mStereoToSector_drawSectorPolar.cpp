/*
 * mStereoToSectorMap_drawPolarSector.cpp
 *
 *  Created on: May 7, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc/mStereoToSectorMap.h"
#include "rrlib/math/tPose3D.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline;

const double sector_cell_height = 30;
const double sector_polar_cell_angle = 6; //9;
const double sector_lower_bound_angle = 45; //0;
const double sector_upper_bound_angle = 135; //359;


void mStereoToSectorMap::drawSectorPolar(rrlib::canvas::tCanvas2D& canvas, tSectorMapPolar2D& map)
{

  tSectorMapPolar2D::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(sector_lower_bound_angle, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(sector_upper_bound_angle, sector_cell_height);
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  map.SetBounds(bounds_polar);
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(sector_polar_cell_angle, sector_cell_height); //test to make sure about overwriting the legth of sectors
  map.SetResolution(coordinate_polar);


  /*//  canvas.SetFillColor(0, 255, 0);*/
  canvas.SetFill(false);
  for (auto it = map.GridIteratorBegin();
       it != map.GridIteratorEnd();
       ++it)
  {
    auto bounds = map.GetBoundsByCellID((*it).index);

    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> low_angle = bounds.lower_bounds[0];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> low_length = bounds.lower_bounds[1];
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> high_angle = bounds.upper_bounds[0];
    /*//    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> high_length = bounds.upper_bounds[1];*/

    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  first_corner;
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  second_corner;
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  third_corner;
    rrlib::math::tVector<2, double, rrlib::math::vector::Polar, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned>  fourth_corner;

    for (unsigned int length = 0; length <= sector_cell_height; length++)
    {
      first_corner.Length() = length; //low_length.Length();
      first_corner[0] = rrlib::math::tAngleDegUnsigned(low_angle[0]);

      second_corner.Length() = (length + 1); //high_length.Length();
      second_corner[0] = rrlib::math::tAngleDegUnsigned(low_angle[0]);

      third_corner.Length() = (length + 1); //high_length.Length();
      third_corner[0] = rrlib::math::tAngleDegUnsigned(high_angle[0]);

      fourth_corner.Length() = length; //low_length.Length();
      fourth_corner[0] = rrlib::math::tAngleDegUnsigned(high_angle[0]);

      canvas.DrawPolygon(first_corner.GetCartesianVector(), second_corner.GetCartesianVector(), third_corner.GetCartesianVector(), fourth_corner.GetCartesianVector());
      /*//    canvas.SetFillColor(0, 255, 0);*/

    }// for different cells

  }// for iterating in sector cells

}
