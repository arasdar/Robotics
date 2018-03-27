/*
 * tStereoProc_processCloud_trav_stepAnalysis.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: aras
 */

#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

#include <pcl/sample_consensus/sac_model_plane.h> //pointToPlaneDistance

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void mStereoProcessing::processCloud_trav_stepAnalysis()
{

  vector<PointIndices> segment_indices(prev_cloud_segments);

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

  CloudPtr cloud_final(new Cloud);
  *cloud_final = *prev_cloud;

  Eigen::Vector3f gravity_vector(0, -1.0, 0);

  //applying the point cloud quality check and dominant ground plane check
  if (!prev_frame_isBad) // if not bad
  {
    //Traversability parameters
    float cos_slope_traversable = cosf(deg2rad(max_slope));

    for (unsigned i = 0; i < segment_indices.size(); ++i)
    {
      for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
      {
        if (segment_indices[i].indices.size() > thresh_segments)
        {
          Eigen::Vector4f plane = prev_cloud_planes[i];
          Eigen::Vector3f plane_normal(plane[0], plane[1], plane[2]);

          float cos_theta = abs(plane_normal.dot(gravity_vector));

          double X = cloud_final->points[segment_indices[i].indices[j]].x; //width
          double Y = cloud_final->points[segment_indices[i].indices[j]].z; //depth

          //RRLIB_UNIT_TESTS_ASSERT_MESSAGE("Coordinate (0, -11) must NOT be in bounds", !map.IsCoordinateInBounds(tVec2d(0, -11)));
          if ((*output_gridmap).IsCoordinateInBounds(rrlib::math::tVec2d(X, Y)))
          {

            // traversable area --> green
            if (cos_theta > cos_slope_traversable)
            {

              cloud->points[segment_indices[i].indices[j]].g = 255;
              cloud_final->points[segment_indices[i].indices[j]].g = 255;
              (*output_gridmap).GetCellByCoordinate(rrlib::math::tVec2d(X, Y)) = 3; //green
            }// if trav

            else // obstacles -- red
            {
              cloud->points[segment_indices[i].indices[j]].r = 255;
              cloud_final->points[segment_indices[i].indices[j]].r = 255;
              if ((*output_gridmap).GetConstCellByCoordinate(rrlib::math::tVec2d(X, Y)) != 3)
              {
                (*output_gridmap).GetCellByCoordinate(rrlib::math::tVec2d(X, Y)) = 1; //red
              }
            }

          }

        }// if thresh

        else // SMALL SEGMENTS && noisy area  -- white
        {
          cloud_final->points[segment_indices[i].indices[j]].r = 255;
          cloud_final->points[segment_indices[i].indices[j]].g = 255;
          cloud_final->points[segment_indices[i].indices[j]].b = 255;
        }

      }
    }// if i

    // disparity not available  -- black
    for (unsigned i = 0; i < cloud->points.size(); ++i)
    {
      if (!isFinite(cloud->points[i]))
      {
        cloud_final->points[i].r = 0;
        cloud_final->points[i].g = 0;
        cloud_final->points[i].b = 0;
      }
    }
  }// if frame is bad

  prev_cloud_trav_stepAnalysis = cloud;
  prev_cloud_trav_final = cloud_final;
}


