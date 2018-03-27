/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void mStereoProcessing::processCloud_trav_slopeAnalysis()
{
  vector<PointIndices> segment_indices(prev_cloud_segments);
  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

  //      Eigen::Vector4f gravity(0, -1, 0, 0); //should be adapted to vehicle
  //      <part file="iv/stereo_camera_system.iv" name="LUGV_LOWRES_CAM"  attached_to="LUGV_chassi"
  //      pose_offset="3.2 -1.2 0  0    3.1    0" />
  //      pose_offset="x    y   z roll pitch yaw" />

//  Eigen::Vector3f gravity_vector(0, -9.81, 0);
  Eigen::Vector3f gravity_vector(0, -1.0, 0);

//  //adjust for camera tilt
////  FINROC_LOG_PRINT(DEBUG, "UnitX(): ", Eigen::Vector3f::UnitX());
////  FINROC_LOG_PRINT(DEBUG, "UnitY(): ", Eigen::Vector3f::UnitY());
//  rrlib::math::tAngleRad camera_tilt_pitch(deg2rad(3.1));
//
//  rrlib::math::tPose3D chassi_pose = si_ugv_chassi_pose.Get();
////  FINROC_LOG_PRINT(DEBUG, "chassi pose: ", chassi_pose);
//  rrlib::math::tPose3D camera_pose(0, 0, 0, chassi_pose.Roll(), chassi_pose.Pitch() + camera_tilt_pitch, rrlib::math::tAngleRad(0));
////  FINROC_LOG_PRINT(DEBUG, "camera pose: ", camera_pose);
//
//  Eigen::Vector3f gravity_camera = Eigen::AngleAxisf(-1 * camera_pose.Roll().Value(), Eigen::Vector3f::UnitX()) * gravity_vector;
////  FINROC_LOG_PRINT(DEBUG, "gravity_camera for X: ", gravity_camera);
//
//  gravity_camera = Eigen::AngleAxisf(-1 * camera_pose.Pitch().Value(), Eigen::Vector3f::UnitY()) * gravity_camera;
//  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gravity in camera system final: ", gravity_camera);



  for (unsigned i = 0; i < segment_indices.size(); ++i)
  {
    if (segment_indices[i].indices.size() > thresh_segments)
    {
      Eigen::Vector4f plane = prev_cloud_planes[i];
      Eigen::Vector3f plane_normal(plane[0], plane[1], plane[2]);

//      float cos_theta = abs(plane_normal.dot(gravity_camera));
      float cos_theta = abs(plane_normal.dot(gravity_vector));
      //cout << "cos_theta: "<< cos_theta << endl;

      float cos_slope_traversable = cosf(deg2rad(max_slope));
//      cout << "slope traversable: " << cos_slope_traversable << endl;


      for (unsigned j = 0; j < segment_indices[i].indices.size(); ++j)
      {
        if (cos_theta >= cos_slope_traversable)  // the angle is small
        {
          cloud->points[segment_indices[i].indices[j]].g = 255;
          prev_segm_sizes_trav.push_back(segment_indices[i].indices.size());
        }
        else
        {
          cloud->points[segment_indices[i].indices[j]].r = 255;
          prev_segm_sizes_non.push_back(segment_indices[i].indices.size());
        }

        prev_segm_sizes.push_back(segment_indices[i].indices.size());
      }
    }
  }

  prev_cloud_trav_slopeAnalysis = cloud;
}




