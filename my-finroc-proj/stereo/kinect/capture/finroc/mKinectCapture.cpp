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
/*!\file    projects/icarus/sensor_processing/kinect/capture/finroc/mKinectCapture.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-01-15
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/kinect/capture/finroc/mKinectCapture.h"

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
namespace kinect
{
namespace capture
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mKinectCapture> cCREATE_ACTION_FOR_M_KINECTCAPTURE("KinectCapture");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void
mKinectCapture::cloud_cb_(const CloudConstPtr& cloud)
{
  boost::mutex::scoped_lock lock(cloud_mutex_);
  cloud_ = cloud;
}

CloudConstPtr
mKinectCapture::getLatestCloud()
{
  //lock while we swap our cloud and reset it.
  boost::mutex::scoped_lock lock(cloud_mutex_);
  CloudConstPtr temp_cloud;
  temp_cloud.swap(cloud_);  //here we set cloud_ to null, so that it is safe to set it again from ourcallback
  return (temp_cloud);
}

void mKinectCapture::processCloud(const CloudConstPtr& cloud)
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

      //std::cout << "plane_params A B C D (eigen vector): "  << plane_params << '\n';
      //std::cout << "eigen_value: "  << eigen_value << '\n';
      //std::cout << "dominant segmentaed plane inliers number: " << largest_1 << std::endl;

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

  /*checking the number of point in the cloud for verification*/
  //dominant plane y limit -- height limits
  float dominant_plane_y_max, dominant_plane_y_min;
  bool frame_isGood = true;
  if (ground_cloud->points.size() > 0)
  {
    //std::sort(region_indices_size.begin(), region_indices_size.end());  //from small to big descending order
    sort(dominant_plane_y.begin(), dominant_plane_y.end());
    //int largest_1 = region_indices_size.at(region_indices_size.size() - 1);
    dominant_plane_y_max = dominant_plane_y.at(dominant_plane_y.size() - 1);
    dominant_plane_y_min = dominant_plane_y.at(0);
    cout << "max y: " << dominant_plane_y_max << endl;
    cout << "min y: " << dominant_plane_y_min << endl;
  }// if ground

  /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
  if (ground_cloud->points.size() > 0)
  {
    ground_centroid = centroids[0];
    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
  }// if

  /*filtering*/
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  //pass.setFilterFieldName("x");
  //pass.setFilterLimits(-10, 10);
  pass.setFilterFieldName("y");
  //pass.setFilterLimits(dominant_plane_y_min - 3, dominant_plane_y_max + 1);
  pass.setFilterLimits(dominant_plane_y_min, dominant_plane_y_max);
  //pass.setFilterFieldName("z");
  //pass.setFilterLimits(0, 50.0);
  CloudPtr cloud_filtered(new Cloud);
  pass.filter(*cloud_filtered);
  //CloudPtr ground_image_test(new Cloud);
  //pass.filter(*ground_image_test);
  cout << "size: " << cloud_filtered->points.size() << endl;
  //cout << "size: " << ground_image_test->points.size() << endl;
  //cout << "size 2: " << ground_image_test->size() << endl;
  unsigned int threshold_dominant_plane_cloud = (cloud->width / 10) * (cloud->height / 10) * 1.5 * 10;
  cout << "threshold dominant_plane_cloud: " << threshold_dominant_plane_cloud << endl;
  if (cloud_filtered->size() < threshold_dominant_plane_cloud) //5000 =~ 80 * 60 = 4800
  {
    cout << "bad point cloud and not relibale ........................................... because cloud size is : " << cloud_filtered->size() << endl;
    frame_isGood = false;

    //put text on the cloud or images
    /* add text:*/
    std::stringstream ss;
    ss << "BAD FRAME" ;
    double  textScale = 0.5; //0.3;  //0.4; //0.1; //1.0;
    double  r = 1.0;
    double  g = 0.0;
    double  b = 0.0;
    //char str[10] = "text_id";
    //char str_2[10] = "text_id_2";
    PointT position; //(centroids[0])
    position.x = -2; //centroids[0][0]; //0;
    position.y = 0; //centroids[0][1];//0;
    position.z = 5; //centroids[0][2];//1;
//    viewer->addText3D(ss.str(), position, textScale, r, g, b,  "cloud");

    unsigned int x_min = 0, y_min = 0, x_max = cloud->width, y_max = cloud->height; //image.width=800, image.height=600
    double opacity = 1.0;
//    image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    x_min = cloud->width;
    y_min = 0;
    x_max = 0;
    y_max = cloud->height;
//    image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
  }

  /*! other traversable regions in dominant plane limits - green*/
  if (frame_isGood)
  {
    /*! other traversable regions in dominant plane limits - green*/
    for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
    {

      /*looking for other the largest region*/
      if (region_indices[i].indices.size() > threshold_min_dominant_plane_cloud &&
          region_indices[i].indices.size() <= largest_1 &&
          region_indices[i].indices.size() > largest_1 / 2) //100 original //min_inliers - green -- for each segment 1000 original
      {

        /*! Compute plane info*/
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
        Eigen::Matrix3f clust_cov;
        pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);

        //cout << "cluster_centroid: " << clust_centroid << endl;
        cout << "cluster_centroid_y: " << clust_centroid[1] << endl;


        pcl::PointXYZ centroid_pt(clust_centroid[0], clust_centroid[1], clust_centroid[2]);
        //double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
        double ptp_dist =  pcl::pointToPlaneDistance(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

        //if (plane_params[3] < dominant_plane_y_max && plane_params[3] > dominant_plane_y_min)
        if (clust_centroid[1] <= dominant_plane_y_max && clust_centroid[1] >= dominant_plane_y_min && ptp_dist < 1)
        {

          cout << "cluster_centroid meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;

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

          }// for
        }//if
        //else
        //{
        //cout << "cluster_centroid NOT meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
        //}//else

      } //if region
    }// for i
  }// frame

  /*plane refinement by mps*/
  if (frame_isGood && ground_cloud->points.size() > 0)
  {

    pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
    refinement_compare->setInputCloud(cloud);
    refinement_compare->setInputNormals(normal_cloud);
    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    refinement_compare->setDistanceThreshold(distance_threshold, depth_dependent);
    refinement_compare->setModelCoefficients(model_coefficients);

    /*the angle between the normals is supposedly pretty noisy that s using refineCOmparator */
    //float angular_threshold = pcl::deg2rad(20.0f); //the tolerance in radians
    //refinement_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle capability //these are noisy regions for normals

    std::vector<bool> grow_labels;
    std::vector<int> label_to_model;
    grow_labels.resize(region_indices.size(), false);
    label_to_model.resize(region_indices.size(), 0);
    //vector<float> model_coefficients_float; //PlaneCoefficientComparator

    for (size_t i = 0; i < model_coefficients.size(); i++) //this is only the biggest plane
    {
      int model_label = (labels)[inlier_indices[i].indices[0]].label;
      label_to_model[model_label] = static_cast<int>(i);
      grow_labels[model_label] = true;
      //model_coefficients_float.push_back(model_coefficients[i].values[3]); //PlaneCoefficientComparator
    }

    refinement_compare->setRefineLabels(grow_labels);
    refinement_compare->setLabelToModel(label_to_model);

    boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
    *labels_ptr = labels;
    refinement_compare->setLabels(labels_ptr);

    /* organized multi plane segmentation demo using different comparators*/
    OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps;
    mps.setInputCloud(cloud);

    /*Provide a pointer to the vector of indices that represents the input data.*/
    //mps_test.setIndices(const IndicesPtr &  indices); //not needed since we have the input

    mps.setInputNormals(normal_cloud);

    /*Set the minimum number of inliers required for a plane.  * image is 800* 600  = width*height */
    unsigned min_inliers = 800; //500 first time //this shows how stable and reliable a plane is //max w and height of the image
    mps.setMinInliers(min_inliers);

    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
    mps.setAngularThreshold(angular_threshold);

    /*this is only for detecting segmenting plane without using comparator*/
    float distance_threshold_mps = 0.1f;  //the tolerance in meters (at 1m)
    mps.setDistanceThreshold(distance_threshold_mps);

    /*Set the maximum curvature allowed for a planar region.* The tolerance for maximum curvature after fitting a plane.
    Used to remove smooth, but non-planar regions.*/
    double maximum_curvature = 0.01;
    mps.setMaximumCurvature(maximum_curvature);  // a small curvature

    /*Set whether or not to project boundary points to the plane, or leave them in the original 3D space.*/
    bool project_points = true;
    mps.setProjectPoints(project_points);

    /*Provide a pointer to the comparator to be used for refinement.*/
    mps.setRefinementComparator(refinement_compare);

    /*Perform a refinement of an initial segmentation, by comparing points to adjacent regions detected by the initial segmentation.*/
    mps.refine(model_coefficients,
               inlier_indices,
               centroids,
               covariances,
               labels_ptr,
               region_indices); //not the region indices

    /*! Draw the segmentation result testtttttt*/
    //CloudPtr ground_image_test(new Cloud);
    //*ground_image_test = *cloud;
    //vector<PointIndices> region_indices_test(inlier_indices); //region_indices

    //Note the regions that have been extended
    for (unsigned int i = 0; i < region_indices.size(); i++)
    {
      if (region_indices[i].indices.size() >= 1000)
      {
        for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
        {
          if (label_image->points[region_indices[i].indices[j]].g != 255)  //region_indices_test[i].indices.size() >= 1000
          {
            //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
            //ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].r = 255;
            ground_image->points[region_indices[i].indices[j]].g = 255;
            ground_image->points[region_indices[i].indices[j]].b = 0;
            label_image->points[region_indices[i].indices[j]].r = 255;
            label_image->points[region_indices[i].indices[j]].g = 255;
            label_image->points[region_indices[i].indices[j]].b = 0;
          }//if
        }//for
      }//if
    }// for

  }//if for plane refinement

//  /*! Segment Obstacles */
//  if (frame_isGood && ground_cloud->points.size() > 0)
//  {
//    //Cloud::CloudVectorType clusters;
//    //if (ground_cloud->points.size() > 0)
//    //{    } //if ground cloud
//
//    std::vector<bool> plane_labels;
//    plane_labels.resize(region_indices.size(), false);
//    for (size_t i = 0; i < region_indices.size(); i++)
//    {
//      if (region_indices[i].indices.size() > 2800) //mps.getMinInliers()
//      {
//        plane_labels[i] = true;
//      }
//    }
//
//    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
//
//    euclidean_cluster_comparator_->setInputCloud(cloud);
//    boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
//    *labels_ptr = labels;
//    euclidean_cluster_comparator_->setLabels(labels_ptr);
//    euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
//    euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);
//
//    pcl::PointCloud<pcl::Label> euclidean_labels;
//    std::vector<pcl::PointIndices> euclidean_label_indices;
//    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
//    euclidean_segmentation.setInputCloud(cloud);
//    euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);
//
//    for (size_t i = 0; i < euclidean_label_indices.size(); i++)
//    {
//      if ((euclidean_label_indices[i].indices.size() > 2800))
//      {
//        //          Cloud cluster;
//        //          pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
//        //          clusters.push_back(cluster);
//
//        Eigen::Vector4f cluster_centroid;
//        Eigen::Matrix3f cluster_cov;
//        pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);
//
//        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//        pcl::eigen33(cluster_cov, eigen_value, eigen_vector);
//        Eigen::Vector4f plane_params;
//        plane_params[0] = eigen_vector[0];
//        plane_params[1] = eigen_vector[1];
//        plane_params[2] = eigen_vector[2];
//        plane_params[3] = 0;
//
//        float cos_theta = ground_plane_params.dot(plane_params);
//        if (abs(cos_theta) < 0.5)  //cos (60 deg) = 0.5 //theta > 60 deg
//        {
//          for (int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
//          {
//            ground_image->points[euclidean_label_indices[i].indices[j]].r = 255;
//            label_image->points[euclidean_label_indices[i].indices[j]].r = 255;
//            label_image->points[euclidean_label_indices[i].indices[j]].g = 0;
//            label_image->points[euclidean_label_indices[i].indices[j]].b = 0;
//          }
//        }
//
////          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
////          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
////
////          if ((ptp_dist > 0.5) && (ptp_dist < 3.0)) //point-to-plane dist
////          {}
//
//      }
//    } //for size
//
//  } //obstacles


  /*! NAN points in the image as well */
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
    prev_label_image = label_image;
    prev_ground_normal = ground_plane_params;
    prev_ground_centroid = ground_centroid;
    cloud_mutex.unlock();
  }

}// processCloud

//void mKinectCapture::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
//{
//  if (event.keyUp())
//  {
//    switch (event.getKeyCode())
//    {
//      //normalls by pressing n
//    case 'n':
//      display_normals = ! display_normals;
//      break;
//      //quit by pressing esc TODO
//    }
//  }
//}// keyboardCallback

void mKinectCapture::saveCloud(const CloudConstPtr& cloud)
{
  std::stringstream ss;
  if (images_idx < 10)
  {
    ss << dir_name_ << "/" << file_name_ << "_000" << images_idx << ".pcd";  //sprintf(filename, "left/left%.4d.png", images_idx);
  }
  else if (images_idx < 100)
  {
    ss << dir_name_ << "/" << file_name_ << "_00" << images_idx << ".pcd" ;
  }
  else if (images_idx < 1000)
  {
    ss << dir_name_ << "/" << "file_name_" << "_0" << images_idx << ".pcd";
  }
  else if (images_idx < 10000)
  {
    ss << dir_name_ << "/" << file_name_ << "_" << images_idx << ".pcd";
  }
  cout << "Saved " << ss.str() << endl;

  if (format_ & 1)
  {
    writer_.writeBinary<PointT> (ss.str(), *cloud);
    //std::cerr << "Data saved in BINARY format to " << ss.str () << std::endl;
  }

  if (format_ & 2)
  {
    writer_.writeBinaryCompressed<PointT> (ss.str(), *cloud);
    //std::cerr << "Data saved in BINARY COMPRESSED format to " << ss.str () << std::endl;
  }

  if (format_ & 4)
  {
    writer_.writeBinaryCompressed<PointT> (ss.str(), *cloud);
    //std::cerr << "Data saved in BINARY COMPRESSED format to " << ss.str () << std::endl;
  }

}// saveCloud

void mKinectCapture::gridmap()
{
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap_ptr = output_gridmap.GetUnusedBuffer();
  //  rrlib::mapping::tMapGridCartesian2D<double>& map_stereo = *output_gridmap_ptr;
  output_gridmap_ptr->Clear(0);

  bounds_stereo_env.upper_bounds[0] = rrlib::math::tVec2d(bound_limit, 0);
  bounds_stereo_env.lower_bounds[0] = rrlib::math::tVec2d(-bound_limit, 0);
  bounds_stereo_env.upper_bounds[1] = rrlib::math::tVec2d(0, bound_limit);
  bounds_stereo_env.lower_bounds[1] = rrlib::math::tVec2d(0, -bound_limit);

  output_gridmap_ptr->SetBounds(bounds_stereo_env);
  output_gridmap_ptr->SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1));  //0.1 cell resolution is one meter in simulation //LUGV 3m - 3 cell size - 0.3 in grid map

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
        output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x , gridmap_y)) = 1; //green

      }// if

      if (prev_label_image->at(j, i).r == 255 && prev_label_image->at(j, i).g == 255 && prev_label_image->at(j, i).b == 0) //yellow
      {
        double gridmap_x = prev_ground_image->at(j, i).x;
        double gridmap_y = prev_ground_image->at(j, i).z;
        output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 2; //yellow
      }// if yellow

//      if (prev_label_image->at(j, i).r == 255 && prev_label_image->at(j, i).g == 0 && prev_label_image->at(j, i).b == 0) //blue unknown //out of range
//      {
//        double gridmap_x = prev_ground_image->at(j, i).x;
//        double gridmap_y = prev_ground_image->at(j, i).z;
//        output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 3; //red obstacle
//      }// if red obstacle

    }//for j
  } // for i

  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "width: ", prev_ground_image->width, "----- height: ", prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ALL IMAGE PIXELS: ", prev_ground_image->width * prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "NUMBER OF CELLS: ", output_gridmap_ptr->GetNumberOfCells());

  /*---------------> 4- pose of robot and environment <----------------------*/
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, 0)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.1)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.2)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.3)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.4)) = 0.2; //origin and location of stereo camera -- left one

  //publishing the port
  output_gridmap.Publish(output_gridmap_ptr);
}// gridmap

//----------------------------------------------------------------------
// mKinectCapture constructor
//----------------------------------------------------------------------
mKinectCapture::mKinectCapture(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  grabber_("#1", depth_mode, image_mode)//grabber
  ,
  road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
//  ,
//  image_viewer(new pcl::visualization::ImageViewer("image viewer")),
//  viewer(new pcl::visualization::PCLVisualizer("3D viewer")),
//  image_viewer_debugging(new pcl::visualization::ImageViewer("image viewer debugging"))
  ,
  writer_(),
  file_name_("frames_pcd"),
  dir_name_("frames"),
  format_(4)
{

  /* processCloud -- Set up the normal estimation based on kinematic capability and the terrain for GroundPlaneComparator*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

  bool use_depth_dependent_smoothing = false;
  ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

  Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
  Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
  road_comparator->setExpectedGroundNormal(tilt_road_normal);

  float   ground_angular_threshold = pcl::deg2rad(20.0f);
  road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original

  /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
  float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
  road_comparator->setAngularThreshold(angular_threshold); //3.0f original

  /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
  float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
  bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
  road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);

  /*
   * kinect capture
   * capturing from run()
   */
  boost::function<void (const CloudConstPtr&)> f = boost::bind(&mKinectCapture::cloud_cb_, this, _1); //  // make callback function from member function
  boost::signals2::connection c = grabber_.registerCallback(f); //  // connect callback function for desired signal. In this case its a point cloud with color values
  grabber_.start(); //  // start receiving point clouds


//  //  // Set up a 3D viewer
//  viewer->setBackgroundColor(0, 0, 0);
//  double scale = 2.0;
//  float x = 0.0; //0
//  float y = 0.5; //0.5
//  float z = 1.0 ; //1
//  int viewport = 0;
//  //Adds 3D axes describing a coordinate system to screen at x, y, z.
//  viewer->addCoordinateSystem(scale, x, y, z, viewport);  //    viewer->addCoordinateSystem(1.0); //default
//  viewer->initCameraParameters();
//  //viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0) //TODO
//  // run and keyboard callback
//  viewer->registerKeyboardCallback(&mKinectCapture::keyboardCallback, *this, 0);

}// constructor

void mKinectCapture::pcd2img()
{
  pcl::PCLImage image;
  pcl::io::PointCloudImageExtractorFromRGBField<PointT> pcie;
  pcie.extract(*prev_ground_image, image);

  finroc::data_ports::tPortDataPointer<rrlib::coviroa::tImage> output_img_ptr = output_img.GetUnusedBuffer();

  //rrlib::coviroa::tImage output_tImage;
  output_img_ptr->Resize(image.width, image.height, rrlib::coviroa::eIMAGE_FORMAT_RGB24);
  memcpy(output_img_ptr->GetImagePtr(), &image.data[0], output_img_ptr->GetImageSize());

  output_img.Publish(output_img_ptr);

}//pcd2img

//----------------------------------------------------------------------
// mKinectCapture destructor
//----------------------------------------------------------------------
mKinectCapture::~mKinectCapture()
{
  grabber_.stop();
}

//----------------------------------------------------------------------
// mKinectCapture OnStaticParameterChange
//----------------------------------------------------------------------
void mKinectCapture::OnStaticParameterChange()
{
//  if (this->static_parameter_1.HasChanged())
//  {
//    //As this static parameter has changed, do something with its value!
//  }
}

//----------------------------------------------------------------------
// mKinectCapture OnParameterChange
//----------------------------------------------------------------------
void mKinectCapture::OnParameterChange()
{
  //If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mKinectCapture Update
//----------------------------------------------------------------------
void mKinectCapture::Update()
{
  if (this->InputChanged())
  {
    //At least one of your input ports has changed. Do something useful with its data.
    //However, using the .HasChanged() method on each port you can check in more detail.
  }

  //Do something each cycle independent from changing ports.
  if (cloud_)
  {
    CloudConstPtr cloud = getLatestCloud();

    saveCloud(cloud);
    images_idx++;

//    viewer->removeText3D("cloud");
//    image_viewer->removeLayer("line");  //const std::string &layer_id
    processCloud(cloud);

    gridmap();


    //fingui display
    pcd2img();

//    image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
//
//    if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
//    {
//      pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_ground_image);
//      viewer->addPointCloud(prev_ground_image, rgb, "cloud"); //works too
//      //viewer->addPointCloud(prev_ground_image, "cloud");
//    }//if
//
//    if (prev_normal_cloud->points.size() > 1000 && display_normals)
//    {
//      viewer->removePointCloud("normals");
//      viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
//    }// if display_normals
//    else
//    {
//      if (!display_normals)
//      {
//        viewer->removePointCloud("normals");
//      }
//    }
//
//    // Show the groundplane normal
//    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
//
//    // Adjust for camera tilt
//    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//
//    // Show the groundplane normal
//    pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
//    pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
//                      prev_ground_centroid[1] + prev_ground_normal[1],
//                      prev_ground_centroid[2] + prev_ground_normal[2]);
//    pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
//                      prev_ground_centroid[1] + tilt_road_normal[1],
//                      prev_ground_centroid[2] + tilt_road_normal[2]);
//
//    viewer->removeShape("ground_norm");
//    viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
//    viewer->removeShape("expected_ground_norm");
//    viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");
//
//    image_viewer->spinOnce(1);
//    viewer->spinOnce(1);

  }// if  run_capture();
  //this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}
