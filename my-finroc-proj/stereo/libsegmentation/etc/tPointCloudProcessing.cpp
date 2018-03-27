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

//----------------------------------------------------------------------
// tPointCloudProcessing constructors
//----------------------------------------------------------------------
PointCloudProcessing::PointCloudProcessing(const std::vector<std::string> left_images, const int frames_number_total) :
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
  viewer_test(new pcl::visualization::PCLVisualizer("3D Viewer test"))
  ,
  image_viewer_test(new pcl::visualization::ImageViewer("Image Viewer test")),
  road_comparator(new GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
/*If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!*/
{
  trigger = true;
  continuous = false;
  display_normals = false;

  this->left_images = left_images;
  files_idx = 0;
  images_idx = 0;
  this -> frames_number_total = frames_number_total;

  /*// Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->registerKeyboardCallback(&PointCloudProcessing::keyboardCallback, *this, 0);


  /*// Set up a 3D viewer*/
  viewer_test->setBackgroundColor(0, 0, 0);
  viewer_test->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_test->registerKeyboardCallback(&PointCloudProcessing::keyboardCallback, *this, 0);

  /*Set up the normal estimation based on kinematic capability and the terrain for GroundPlaneComparator*/
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(40.0f); //20.0f
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
  bool use_depth_dependent_smoothing = false;
  ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

  /*! Set the tolerance in radians for difference in normal direction between neighboring points,
   * to be considered part of the same plane.*/
  float angular_threshold = pcl::deg2rad(20.0f); //the tolerance in radians
  road_comparator->setAngularThreshold(angular_threshold); //3.0f original

  /*
    // Set up the ground plane comparator -- If the camera was pointing straight out, the normal would be:
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)

    Adjust for camera tilt for traversability analysis
    Set the expected ground plane normal with respect to the stereo camera.
     * Pixels labeled as ground must be within ground_angular_threshold radians of this normal to be labeled as ground.
     *
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);

    slope threshold and Set the tolerance in radians for difference in normal direction between a point and the expected ground normal.
    float   ground_angular_threshold = pcl::deg2rad(20.0f);
    road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original


        Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
    road_comparator->setAngularThreshold(angular_threshold); //3.0f original

    step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);
  */

}

//----------------------------------------------------------------------
// tPointCloudProcessing destructor
//----------------------------------------------------------------------
PointCloudProcessing::~PointCloudProcessing()
{}

////----------------------------------------------------------------------
//// tPointCloudProcessing SomeExampleMethod
////----------------------------------------------------------------------
//void tPointCloudProcessing::SomeExampleMethod()
//{
//  /*This is an example for a method. Replace it by your own methods!*/
//}

void
PointCloudProcessing::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
    case ' ':
      trigger = true;
      break;
    case 'c':
      continuous = !continuous;
      break;
    case 'n':
      display_normals = !display_normals;
      break;
    }
  }
}

void
PointCloudProcessing::processCloud(const CloudConstPtr& cloud)
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


  /*! segmentation results */
  for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
  {

    if (region_indices[i].indices.size() > 100)  //100 original //min_inliers - green -- for each segment 1000 original
    {

      for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
      {
        ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
        label_image->points[region_indices[i].indices[j]].r = 0;
        label_image->points[region_indices[i].indices[j]].g = 255 ;
        label_image->points[region_indices[i].indices[j]].b = 0;
      }// for

    } //if region
  }// for i


  /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);


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

void PointCloudProcessing::run()
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
    if (cloud_mutex.try_lock())  //undefined ref to pthread with new gcc 4.8
    {
      if (!viewer->updatePointCloud(prev_ground_image, "cloud")) //&& !viewer_test->updatePointCloud(prev_ground_image_test, "cloud_test")
      {
        viewer->addPointCloud(prev_ground_image, "cloud");
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

      viewer->removePointCloud("normals");
      if (prev_normal_cloud->points.size() > 1000 && display_normals)
      {
        viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
      }

      cloud_mutex.unlock();
    }//if

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
