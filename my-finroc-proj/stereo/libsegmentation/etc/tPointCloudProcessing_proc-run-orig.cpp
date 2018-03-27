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
/*!\file    projects/icarus/sensor_processing/libsegmentation/tPointCloudProcessing.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-04-30
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/libsegmentation/tests/tPointCloudProcessing.h"

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
namespace sensor_processing
{
namespace libsegmentation
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

void
PointCloudProcessing::processCloud_orig(const CloudConstPtr& cloud)
{
  /*Compute the normals*/
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(cloud); //input
  ne.compute(*normal_cloud); //output

  /*Set up the ground plane comparator*/
  road_comparator->setInputCloud(cloud);
  road_comparator->setInputNormals(normal_cloud);

  /*Run segmentation*/
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> region_indices;
  road_segmentation.setInputCloud(cloud);
  road_segmentation.segment(labels, region_indices);

  /*! Draw the segmentation result*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  CloudPtr ground_image(new Cloud);
  CloudPtr label_image(new Cloud);
  *ground_image = *cloud;
  *label_image = *cloud;


  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids;
  std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> covariances;
  std::vector<pcl::PointIndices> inlier_indices;
  std::vector<float> region_indices_eigvals;


  /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
  std::vector<int> region_indices_size;
  vector<float> dominant_plane_y; //y in pcd which is height
  for (unsigned int i = 0; i < region_indices.size(); i++)
  {
    region_indices_size.push_back(region_indices[i].indices.size());
  }
  std::sort(region_indices_size.begin(), region_indices_size.end());  //from small to big descending order
  unsigned int largest_1 = region_indices_size.at(region_indices_size.size() - 1);

  unsigned int threshold_min_dominant_plane_cloud = (cloud->width / 10) * (cloud->height / 10) * 1.5 * 0.5;  //2500
  cout << "threshold_min_dominant_plane_cloud: " << threshold_min_dominant_plane_cloud << endl;

  /*! traversable regions - green*/
  for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
  {

    /*looking for only the largest region*/
    if (region_indices[i].indices.size() == largest_1 && largest_1 >= threshold_min_dominant_plane_cloud) //100 original //min_inliers - green -- for each segment 1000 original
    {

      /*! Compute plane info*/
      Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
      Eigen::Matrix3f clust_cov;
      pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);

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

      pcl::ModelCoefficients model;
      model.values.resize(4);
      model.values[0] = plane_params[0];
      model.values[1] = plane_params[1];
      model.values[2] = plane_params[2];
      model.values[3] = plane_params[3];
      model_coefficients.push_back(model);
      inlier_indices.push_back(region_indices[i]);
      centroids.push_back(clust_centroid);
      covariances.push_back(clust_cov);
      region_indices_eigvals.push_back(eigen_value);

      /*//std::cout << "plane_params A B C D (eigen vector): "  << plane_params << '\n';
      //std::cout << "eigen_value: "  << eigen_value << '\n';
      //std::cout << "dominant segmentaed plane inliers number: " << largest_1 << std::endl;*/

      /*visualizing the result by colorizing the point cloud */ /*! traversable regions - green*/
      for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
      {
        ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
        label_image->points[region_indices[i].indices[j]].r = 0;
        label_image->points[region_indices[i].indices[j]].g = 255 ;
        label_image->points[region_indices[i].indices[j]].b = 0;
        pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                cloud->points[region_indices[i].indices[j]].y,
                                cloud->points[region_indices[i].indices[j]].z);
        ground_cloud->points.push_back(ground_pt);

        // getting the max and min y for filtering
        dominant_plane_y.push_back(cloud->points[region_indices[i].indices[j]].y);
      }// for


    } //if region
  }// for i


  /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
  if (ground_cloud->points.size() > 0)
  {
    ground_centroid = centroids[0];
    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
  }// if


  /*// note the NAN points in the image as well*/
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if (!pcl::isFinite(cloud->points[i]))
    {
      ground_image->points[i].b = static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
      label_image->points[i].r = 0;
      label_image->points[i].g = 0;
      label_image->points[i].b = 255;
    }
  }

  /*! Update info for the visualization thread*/
  {
    cloud_mutex.lock();
    prev_cloud = cloud;
    prev_normal_cloud = normal_cloud;
    prev_ground_cloud = ground_cloud;
    prev_ground_image = ground_image;
    prev_ground_image_test = ground_image;  //testttttttttttttt
    prev_label_image = label_image;
    prev_ground_normal = ground_plane_params;
    prev_ground_centroid = ground_centroid;
    cloud_mutex.unlock();
  }
}

void PointCloudProcessing::run_orig()
{

  while (!viewer->wasStopped())
  {

    if (frames_number_total == images_idx)
    {
      cout << "frames_number_total == images_idx -> " << images_idx << std::endl;
      break;
    } // if

    /*// Process a new cloud*/
    if (trigger || continuous)
    {
      // showing the input images
      cout << "images_idx: " << images_idx << endl;
      cout << "frames_number_total: " << frames_number_total << endl;

      CloudPtr left_cloud(new Cloud);
      pcl::PCDReader pcd;
      pcd.read(left_images[images_idx], *left_cloud);
      viewer->removeText3D("cloud");
      viewer_test->removeText3D("cloud_test");
      image_viewer->removeLayer("line");  //const std::string &layer_id
      image_viewer_test->removeLayer("line");
      processCloud(left_cloud);

      trigger = false;
      images_idx ++;
    }

    /*Draw visualizations*/
    /*//if (cloud_mutex.try_lock())  //undefined ref to pthread with new gcc 4.8
    //{         cloud_mutex.unlock();      }//if*/

    if (!viewer->updatePointCloud(prev_ground_image, "cloud")) //&& !viewer_test->updatePointCloud(prev_ground_image_test, "cloud_test")
    {

      viewer->addPointCloud(prev_ground_image, "cloud");
      //viewer_test->addPointCloud(prev_ground_image_test, "cloud_test");

    }// if

    if (!viewer_test->updatePointCloud(prev_ground_image_test, "cloud_test"))
    {
      viewer_test->addPointCloud(prev_ground_image_test, "cloud_test");
    }// if


    if (prev_cloud->points.size() > 1000)
    {
      image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
      image_viewer_test->addRGBImage<PointT>(prev_ground_image_test, "rgb_image_test", 0.3);
    }// if

    /*// showing normals on point clouds*/
    if (prev_normal_cloud->points.size() > 1000 && display_normals)
    {
      viewer->removePointCloud("normals");
      viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
    }

    /*comparing the normals of real ground and expected ground plane for normal*/
    /*//if (prev_ground_cloud->points.size() > 0) {        }// if*/

    /*// Show the ground plane normal*/
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);

    /*// Adjust for camera tilt*/
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;

    /*// Show the ground plane normal*/
    pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
    pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                      prev_ground_centroid[1] + prev_ground_normal[1],
                      prev_ground_centroid[2] + prev_ground_normal[2]);
    pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                      prev_ground_centroid[1] + tilt_road_normal[1],
                      prev_ground_centroid[2] + tilt_road_normal[2]);

    /*// comparing the normals of real ground and expected ground plane for normal*/
    viewer->removeShape("ground_norm");
    viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
    viewer->removeShape("expected_ground_norm");
    viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");

    /*//prev_ground_cloud->removeAllPointClouds();
    //prev_ground_cloud->clear();*/


    viewer->spinOnce(100);
    image_viewer->spinOnce(100);
    viewer_test->spinOnce(100);
    image_viewer_test->spinOnce(100);

  }//while

}//run

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
