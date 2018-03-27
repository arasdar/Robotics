/*
 * tStereoProc_processCloud_trav_functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/stereo_color/offline/tStereoProcessing.h"

#include <pcl/common/centroid.h>

using namespace finroc::stereo_traversability_experiments::aras::stereo_color::offline;

void tStereoProcessing::processCloud_trav_segm2plane()
{
  std::vector<PointIndices> segment_indices(prev_cloud_segments);
  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

  vector<PointXYZ> np1_vec, np2_vec;
  char str[50];
  unsigned index = 0;

  //cleaning the previous segment normals first
  for (unsigned idx = 0; idx < prev_idx; ++idx)
  {
    sprintf(str, "segm_norm_%01d", idx);
    viewer_proc_trav->removeShape(str);
  }

  prev_cloud_planes.clear();
  prev_cloud_planes.resize(segment_indices.size());

  prev_cloud_planes_cent_pt.clear();
  prev_cloud_planes_cent_pt.resize(segment_indices.size());

  for (unsigned int i = 0; i < segment_indices.size(); i++)
  {

    prev_cloud_planes[i] = Eigen::Vector4f(0, 0, 0, 0);

    if (segment_indices[i].indices.size() > thresh_segments)
    {

      /*! Compute plane info*/
      Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
      Eigen::Matrix3f clust_cov;
      pcl::computeMeanAndCovarianceMatrix(*cloud, segment_indices[i].indices, clust_cov, clust_centroid);

      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
      pcl::eigen33(clust_cov, eigen_value, eigen_vector);
      Eigen::Vector4f plane_params;
      plane_params[0] = eigen_vector[0];
      plane_params[1] = eigen_vector[1];
      plane_params[2] = eigen_vector[2];
      plane_params[3] = 0;

      /*! 1- D*/
      plane_params[3] = -1 * plane_params.dot(clust_centroid);

      /*! 2- D*/
      Eigen::Vector4f vp = Eigen::Vector4f::Zero();
      vp -= clust_centroid;
      float cos_theta = vp.dot(plane_params);
      if (cos_theta < 0)
      {
        plane_params *= -1;
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot(clust_centroid);
      }

      /*! Show the segment plane normal*/
      PointXYZ np1(clust_centroid[0], clust_centroid[1], clust_centroid[2]);
      PointXYZ np2(clust_centroid[0] + plane_params[0],
                   clust_centroid[1] + plane_params[1],
                   clust_centroid[2] + plane_params[2]);


      sprintf(str, "segm_norm_%01d", index);
      viewer_proc_trav->addArrow(np2, np1, 1.0, 0.0, 0, false, str);
      index ++;

      prev_cloud_planes[i] = plane_params;
      prev_cloud_planes_cent_pt[i] = np1;

    } //if region
  }// for i

  prev_idx = index;
}



