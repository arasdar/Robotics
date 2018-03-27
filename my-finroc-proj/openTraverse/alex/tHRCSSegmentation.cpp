//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/stereo_traversability_experiments/openTraverse/alex/tHRCSSegmentation.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-10-20
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/openTraverse/alex/tHRCSSegmentation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/sample_consensus/sac_model_plane.h>

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
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace alex
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tHRCSSegmentation constructors
//----------------------------------------------------------------------
tHRCSSegmentation::tHRCSSegmentation():
  prev_cloud(new Cloud),
  prev_normal_cloud(new pcl::PointCloud<pcl::Normal>),
  prev_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  prev_ground_image(new Cloud),
  prev_label_image(new Cloud),
  road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  // Set up the normal estimation
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f

  // Set up the groundplane comparator
  // If the camera was pointing straight out, the normal would be:
  Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
  // Adjust for camera tilt:
  Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
  road_comparator->setExpectedGroundNormal(tilt_road_normal);
  road_comparator->setGroundAngularThreshold(pcl::deg2rad(10.0f));
  road_comparator->setAngularThreshold(pcl::deg2rad(3.0f));
}

//----------------------------------------------------------------------
// tHRCSSegmentation destructor
//----------------------------------------------------------------------
tHRCSSegmentation::~tHRCSSegmentation()
{}

////----------------------------------------------------------------------
//// tHRCSSegmentation SomeExampleMethod functions implementations
////----------------------------------------------------------------------
//void tHRCSSegmentation::SomeExampleMethod()
//{
//  This is an example for a method. Replace it by your own methods!
//}

void
tHRCSSegmentation::processCloud(const CloudConstPtr& cloud)
//tHRCSSegmentation::processCloud()
{
  // Compute the normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(cloud);
  ne.compute(*normal_cloud);

  // Set up the groundplane comparator
  road_comparator->setInputCloud(cloud);
  road_comparator->setInputNormals(normal_cloud);

  // Run segmentation
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> region_indices;
  road_segmentation.setInputCloud(cloud);
  road_segmentation.segment(labels, region_indices);

  // Draw the segmentation result
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  CloudPtr ground_image(new Cloud);
  CloudPtr label_image(new Cloud);
  *ground_image = *cloud;
  *label_image = *cloud;

  Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
  Eigen::Vector4f vp = Eigen::Vector4f::Zero();
  Eigen::Matrix3f clust_cov;
  pcl::ModelCoefficients model;
  model.values.resize(4);

  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids;
  std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> covariances;
  std::vector<pcl::PointIndices> inlier_indices;

  for (unsigned int i = 0; i < region_indices.size(); i++)
  {
    if (region_indices[i].indices.size() > 1000)
    {

      for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
      {
        pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                cloud->points[region_indices[i].indices[j]].y,
                                cloud->points[region_indices[i].indices[j]].z);
        ground_cloud->points.push_back(ground_pt);
        ground_image->points[region_indices[i].indices[j]].g = static_cast<pcl::uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
        label_image->points[region_indices[i].indices[j]].r = 0;
        label_image->points[region_indices[i].indices[j]].g = 255;
        label_image->points[region_indices[i].indices[j]].b = 0;
      }

      // Compute plane info
      pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);
      Eigen::Vector4f plane_params;

      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
      pcl::eigen33(clust_cov, eigen_value, eigen_vector);
      plane_params[0] = eigen_vector[0];
      plane_params[1] = eigen_vector[1];
      plane_params[2] = eigen_vector[2];
      plane_params[3] = 0;
      plane_params[3] = -1 * plane_params.dot(clust_centroid);

      vp -= clust_centroid;
      float cos_theta = vp.dot(plane_params);
      if (cos_theta < 0)
      {
        plane_params *= -1;
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot(clust_centroid);
      }

      model.values[0] = plane_params[0];
      model.values[1] = plane_params[1];
      model.values[2] = plane_params[2];
      model.values[3] = plane_params[3];
      model_coefficients.push_back(model);
      inlier_indices.push_back(region_indices[i]);
      centroids.push_back(clust_centroid);
      covariances.push_back(clust_cov);
    }
  }

  //Refinement
  std::vector<bool> grow_labels;
  std::vector<int> label_to_model;
  grow_labels.resize(region_indices.size(), false);
  label_to_model.resize(region_indices.size(), 0);

  for (size_t i = 0; i < model_coefficients.size(); i++)
  {
    int model_label = (labels)[inlier_indices[i].indices[0]].label;
    label_to_model[model_label] = static_cast<int>(i);
    grow_labels[model_label] = true;
  }

  boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
  *labels_ptr = labels;
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
  refinement_compare->setInputCloud(cloud);
  refinement_compare->setDistanceThreshold(0.15f);
  refinement_compare->setLabels(labels_ptr);
  refinement_compare->setModelCoefficients(model_coefficients);
  refinement_compare->setRefineLabels(grow_labels);
  refinement_compare->setLabelToModel(label_to_model);
  mps.setRefinementComparator(refinement_compare);
  mps.setMinInliers(500);
  mps.setAngularThreshold(pcl::deg2rad(3.0));
  mps.setDistanceThreshold(0.02);
  mps.setInputCloud(cloud);
  mps.setInputNormals(normal_cloud);
  mps.refine(model_coefficients,
             inlier_indices,
             centroids,
             covariances,
             labels_ptr,
             region_indices);

  //Note the regions that have been extended
  pcl::PointCloud<PointT> extended_ground_cloud;
  for (unsigned int i = 0; i < region_indices.size(); i++)
  {
    if (region_indices[i].indices.size() > 1000)
    {
      for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
      {
        // Check to see if it has already been labeled
        if (ground_image->points[region_indices[i].indices[j]].g == ground_image->points[region_indices[i].indices[j]].b)
        {
          pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                  cloud->points[region_indices[i].indices[j]].y,
                                  cloud->points[region_indices[i].indices[j]].z);
          ground_cloud->points.push_back(ground_pt);
          ground_image->points[region_indices[i].indices[j]].r = static_cast<pcl::uint8_t>((cloud->points[region_indices[i].indices[j]].r + 255) / 2);
          ground_image->points[region_indices[i].indices[j]].g = static_cast<pcl::uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
          label_image->points[region_indices[i].indices[j]].r = 128;
          label_image->points[region_indices[i].indices[j]].g = 128;
          label_image->points[region_indices[i].indices[j]].b = 0;
        }

      }
    }
  }

  // Segment Obstacles (Disabled by default)
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);

  if (ground_cloud->points.size() > 0)
  {
    ground_centroid = centroids[0];
    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
  }

  if (detect_obstacles)
  {
    pcl::PointCloud<PointT>::CloudVectorType clusters;
    if (ground_cloud->points.size() > 0)
    {
      std::vector<bool> plane_labels;
      plane_labels.resize(region_indices.size(), false);
      for (size_t i = 0; i < region_indices.size(); i++)
      {
        if (region_indices[i].indices.size() > mps.getMinInliers())
        {
          plane_labels[i] = true;
        }
      }

      pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

      euclidean_cluster_comparator_->setInputCloud(cloud);
      euclidean_cluster_comparator_->setLabels(labels_ptr);
      euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
      euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);

      pcl::PointCloud<pcl::Label> euclidean_labels;
      std::vector<pcl::PointIndices> euclidean_label_indices;
      pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
      euclidean_segmentation.setInputCloud(cloud);
      euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

      for (size_t i = 0; i < euclidean_label_indices.size(); i++)
      {
        if ((euclidean_label_indices[i].indices.size() > 200))
        {
          pcl::PointCloud<PointT> cluster;
          pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
          clusters.push_back(cluster);

          Eigen::Vector4f cluster_centroid;
          Eigen::Matrix3f cluster_cov;
          pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);

          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

          if ((ptp_dist > 0.5) && (ptp_dist < 3.0))
          {

            for (unsigned int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
            {
              ground_image->points[euclidean_label_indices[i].indices[j]].r = 255;
              label_image->points[euclidean_label_indices[i].indices[j]].r = 255;
              label_image->points[euclidean_label_indices[i].indices[j]].g = 0;
              label_image->points[euclidean_label_indices[i].indices[j]].b = 0;
            }

          }

        }
      }

    }
  }

  // note the NAN points in the image as well
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if (!pcl::isFinite(cloud->points[i]))
    {
      ground_image->points[i].b = static_cast<pcl::uint8_t>((cloud->points[i].b + 255) / 2);
      label_image->points[i].r = 0;
      label_image->points[i].g = 0;
      label_image->points[i].b = 255;
    }
  }

  prev_cloud = cloud;
  prev_normal_cloud = normal_cloud;
  prev_ground_cloud = ground_cloud;
  prev_ground_image = ground_image;
  prev_label_image = label_image;
  prev_ground_normal = ground_plane_params;
  prev_ground_centroid = ground_centroid;

  /*
  //  // Update info for the visualization thread
  //  {
  //    cloud_mutex.lock();
  //    prev_cloud = cloud;
  //    prev_normal_cloud = normal_cloud;
  //    prev_ground_cloud = ground_cloud;
  //    prev_ground_image = ground_image;
  //    prev_label_image = label_image;
  //    prev_ground_normal = ground_plane_params;
  //    prev_ground_centroid = ground_centroid;
  //    cloud_mutex.unlock();
  //  }
  */
}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
