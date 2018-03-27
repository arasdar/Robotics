/*
 * mStereoProc_gridmap.cpp
 *
 *  Created on: May 6, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;

void mStereoGrayOffline::gridmap()
{
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap_ptr = output_gridmap.GetUnusedBuffer();

  const int bound_limit = 100; //100; //500 //80 //40 //20
  const double cell_resolution = 0.5; //0.3; //0.05; //0.05; //1; //0.13; //0.05; //meter //0.1 //1
  rrlib::mapping::tMapGridCartesian2D<double>::tBounds bounds_stereo_env;

  bounds_stereo_env.upper_bounds[0] = rrlib::math::tVec2d(bound_limit, 0);
  bounds_stereo_env.lower_bounds[0] = rrlib::math::tVec2d(-bound_limit, 0);
  bounds_stereo_env.upper_bounds[1] = rrlib::math::tVec2d(0, bound_limit);
  bounds_stereo_env.lower_bounds[1] = rrlib::math::tVec2d(0, -bound_limit);

  output_gridmap_ptr->SetBounds(bounds_stereo_env);
  output_gridmap_ptr->SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1));  //0.1 cell resolution is one meter in simulation //LUGV 3m - 3 cell size - 0.3 in grid map

  output_gridmap_ptr->Clear(0);

  /*
   * --------------------- environment mapping ----------------
   */
  // drawing the mapping model on image //model fitting on image ==> to know about the location of cells
  for (int i = 0; i < prev_ground_image->height; i++) //height y row  vertical
  {
    for (int j = 0; j < prev_ground_image->width; j++) //width x column  horizon horizontal
    {

      /*
       * ! if green or traversible
       * */
      if (prev_label_image->at(j, i).r == 0 && prev_label_image->at(j, i).g == 255 && prev_label_image->at(j, i).b == 0)
      {

        //local map in RCS
        double gridmap_x = prev_ground_image->at(j, i).x; //image coordinate
        double gridmap_y = prev_ground_image->at(j, i).z;
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x);
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y);

        rrlib::math::tVec2d gridmap_point(gridmap_x, gridmap_y);

        // make sure the points are within certain bounds
        if (abs(gridmap_x) < bound_limit && gridmap_y < bound_limit)
        {
          output_gridmap_ptr->GetCellByCoordinate(gridmap_point) = 1;
        }
      }// if

    }//for j
  } // for i


  //publishing the port
  output_gridmap.Publish(output_gridmap_ptr);
}// gridmap






