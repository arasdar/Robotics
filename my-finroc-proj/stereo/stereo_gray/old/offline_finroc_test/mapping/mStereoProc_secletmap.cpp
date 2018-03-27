/*
 * mStereoProc_secletmap.cpp
 *
 *  Created on: May 8, 2014
 *      Author: aras
 */



#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;


const unsigned sector_cell_height = 30;
const unsigned seclet_cell_angle = 6; //9;
const unsigned seclet_cell_height = 1;
const double sector_lower_bound_angle = 45, sector_upper_bound_angle = 135;


void mStereoGrayOffline::secletmap()
{

  data_ports::tPortDataPointer<rrlib::mapping::tMapGridPolar2D<double>> output_secletmap_ptr = output_secletmap.GetUnusedBuffer();

  typedef rrlib::mapping::tMapGridPolar2D<double> tMap;
  tMap& map = *output_secletmap_ptr;

  typedef tMap::tCoordinate tVec;

  rrlib::mapping::tMapGridPolar2D<double>::tBounds bounds;
  bounds.upper_bounds[0] = tVec(rrlib::math::tAngleDeg(sector_upper_bound_angle), 0); // first dimension: 45 degress till
  bounds.lower_bounds[0] = tVec(rrlib::math::tAngleDeg(sector_lower_bound_angle), 0);  // -45 degrees
  bounds.upper_bounds[1] = tVec(0, sector_cell_height); // second dimension: 10 meters to
  bounds.lower_bounds[1] = tVec(0, 0); // 0 meters

  map.SetBounds(bounds);
  map.SetResolution(tVec(rrlib::math::tAngleDeg(seclet_cell_angle), seclet_cell_height)); // resolution: 15 degrees, 1 meter
  map.Clear(0);

  /*
   * --------------------- environment mapping ----------------
   */
  // drawing the mapping model on image //model fitting on image ==> to know about the location of cells
  for (unsigned int i = 0; i < prev_ground_image->height; i++) //height y row  vertical
  {
    for (unsigned int j = 0; j < prev_ground_image->width; j++) //width x column  horizon horizontal
    {

      /*
       * ! if green or traversible
       * */
      if (prev_label_image->at(j, i).r == 0 && prev_label_image->at(j, i).g == 255 && prev_label_image->at(j, i).b == 0)
      {

        //local map in RCS
        double gridmap_x = prev_ground_image->at(j, i).x; //image coordinate
        double gridmap_y = prev_ground_image->at(j, i).z;

        rrlib::math::tVec2d gridmap_point(gridmap_x, gridmap_y);

        rrlib::math::tAngle<double, rrlib::math::angle::Degree, rrlib::math::angle::Unsigned> alpha_test(gridmap_point.GetPolarVector().Alpha());

        // make sure the points are within certain bounds
        if (gridmap_point.GetPolarVector().Length() < sector_cell_height)//(map.isCoordinateInBound(tVec(alpha_test, gridmap_point.GetPolarVector().Length()))) //(abs(gridmap_x) < bound_limit && gridmap_y < bound_limit)
        {
          output_secletmap_ptr->GetCellByCoordinate(tVec(alpha_test, gridmap_point.GetPolarVector().Length())) = 1;
        }

      }// if

    }//for j
  } // for i

  output_secletmap.Publish(output_secletmap_ptr);
}






