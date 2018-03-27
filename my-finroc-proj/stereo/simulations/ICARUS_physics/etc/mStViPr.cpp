//
// You received this file as part of Finroc
// A Framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/simulation/mStViPr.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2013-04-26
 *
 * * this is sense and control module for stereo vision processing (StViPr)
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/simulation_physics/mStViPr.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "plugins/data_ports/tPortDataPointer.h"
#include "rrlib/coviroa/opencv_utils.h" //flann error !!!  //coviroa to opencv_aras changed

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <ctype.h>
#include <iostream>     // std::cout
#include <algorithm>    // std::for_each
#include <vector>
#include <stdio.h>      /* printf */
#include <math.h>       /* round, floor, ceil, trunc */

/* atan2 example */
#include <stdio.h>      /* printf */
#include <math.h>       /* atan2 */
#define PI 3.14159265

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace finroc::core;
using namespace rrlib::coviroa;
using namespace cv;
using namespace std;


//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace simulation_physics
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStViPr> cCREATE_ACTION_FOR_M_ST_VI_PR("StViPr");
const int scanner_radius_max_threshold = 19;
const int bound_limit = 40; //80 //40 //20
const double cell_resolution = 0.1; //0.1 //1
const double mapping_scale = 0.1; //1; //0.1;
/*! scaling */
const double stereo_max_range = 8; //7.9 - 8.1
const double scanner_max_range = 20; //19; //21;
const double mapping_ratio_Z = scanner_max_range / stereo_max_range;
const double mapping_ratio_X = 0.3 / 3;//1; //0.3 / 12;  //0.3 / 6;  //3 cells is 2.2 X in image //0.3 / 2.2;  //3 cells is 2.2 X in image
const int threshNumPointsInCell_stereo = 3;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
void
mStViPr::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
    case ' ':
      trigger = true;
      break;
//    case '1':
//    smooth_strong -= 10;
//    PCL_INFO ("smooth_strong: %d\n", smooth_strong);
//    stereo.setSmoothStrong (smooth_strong);
//    break;
//    case '2':
//    smooth_strong += 10;
//    PCL_INFO ("smooth_strong: %d\n", smooth_strong);
//    stereo.setSmoothStrong (smooth_strong);
//    break;
//    case '3':
//    smooth_weak -= 10;
//    PCL_INFO ("smooth_weak: %d\n", smooth_weak);
//    stereo.setSmoothWeak (smooth_weak);
//    break;
//    case '4':
//    smooth_weak += 10;
//    PCL_INFO ("smooth_weak: %d\n", smooth_weak);
//    stereo.setSmoothWeak (smooth_weak);
//    break;
//    case 'c':
//      continuous = !continuous;
//      break;
    case 'n':
      display_normals = !display_normals;
      break;
    case 'o':
      detect_obstacles = !detect_obstacles;
      break;
    }
  }
}

PointCloud<RGB>::Ptr
mStViPr::png2pcd(const Mat rect_png_image)
{

  //BGR image
  Mat image;
  cvtColor(rect_png_image, image, CV_GRAY2BGR);

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  double* pixel = new double [4];
  memset(pixel, 0, sizeof(double) * 4);
  PointCloud<RGB> cloud;

  // cloud initialization
  cloud.width = image.cols;
  cloud.height = image.rows; // This indicates that the point cloud is organized
  cloud.is_dense = true;
  cloud.points.resize(cloud.width * cloud.height);

  // png to pcd loop
  for (int y = 0; y < image.rows; y++)
  {
    for (int x = 0; x < image.cols; x++)
    {
      pixel[0] = (double) image.at<Vec3b>(y, x)[0];
      RGB color;
      color.r = 0;
      color.g = 0;
      color.b = 0;
      color.a = 0;
      color.rgb = 0.0f;
      color.rgba = 0;

      int rgb;
      int rgba;

      color.r = static_cast<uint8_t>(pixel[0]);
      color.g = static_cast<uint8_t>(pixel[0]);
      color.b = static_cast<uint8_t>(pixel[0]);

      rgb = (static_cast<int>(color.r)) << 16 |
            (static_cast<int>(color.g)) << 8 |
            (static_cast<int>(color.b));

      rgba = rgb;
      color.rgb = static_cast<float>(rgb);
      color.rgba = static_cast<uint32_t>(rgba);

      cloud(x, y) = color;

    } //for x
  }// for y

  // free memory
  delete[] pixel;

  // output pcd --> conversion from PointCloud<RGB> to PointCloud<RGB>::Ptr
  PointCloud<RGB>::Ptr cloud_output(new PointCloud<RGB>);
  *cloud_output = cloud;

  return cloud_output;
}// png2pcd

void
mStViPr::stereo_rectify(const Mat input_png_images_left, const Mat input_png_images_right)
{

  //variable initialization
  const char* intrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/simulation/etc/calib_intrinsics.yml"; //(char*) input_intrinsic_filename.c_str();
  const char* extrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/simulation/etc/calib_extrinsics.yml"; //(char*) input_extrinsic_filename.c_str();

  //gray input
  Mat img1, img2;
  cvtColor(input_png_images_left, img1, CV_BGR2GRAY);
  cvtColor(input_png_images_right, img2, CV_BGR2GRAY);


  // Mat img = imread(goodImageList[i*2+k]
  Mat img[2];
  img1.copyTo(img[0]);
  img2.copyTo(img[1]);
  Size imageSize = img1.size();


  // matrices
  Mat cameraMatrix[2], distCoeffs[2];
  cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
  cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
  Mat R, T, E, F;


  // save intrinsic parameters
  FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
  if (fs.isOpened())
  {
    Mat M1, D1, M2, D2;
    fs["M1"] >> cameraMatrix[0];
    fs["D1"] >> distCoeffs[0];
    fs["M2"] >> cameraMatrix[1];
    fs["D2"] >> distCoeffs[1];
    fs.release();
  }
  else
    cout << "Error: can not read the intrinsic parameters\n";


  //initialize the variables
  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];

  fs.open(extrinsic_filename, CV_STORAGE_READ);
  if (fs.isOpened())
  {
    fs["R"] >> R;
    fs["T"] >> T;
    fs.release();
  }
  else
    cout << "Error: can not read the intrinsic parameters\n";


  //stereo rectify
  stereoRectify(cameraMatrix[0], distCoeffs[0],
                cameraMatrix[1], distCoeffs[1],
                imageSize, R, T, R1, R2, P1, P2, Q,
                CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);


  //StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true);
  bool useCalibrated = true;
  bool showRectified = true;

  // COMPUTE AND DISPLAY RECTIFICATION
  if (!showRectified)
    return;

  Mat rmap[2][2];

  //Precompute maps for cv::remap()
  initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
  initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

  Mat canvas;
  double sf;
  int w, h;

  // OpenCV can handle left-right or up-down camera arrangements
  bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
  if (!isVerticalStereo)
  {
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);
  }
  else
  {
    sf = 300. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h * 2, w, CV_8UC3);
  }
  Mat img_rect[2];
  for (int k = 0; k < 2; k++)
  {
    Mat rimg, cimg;
    remap(img[k], rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
    cvtColor(rimg, cimg, CV_GRAY2BGR);
    Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
    resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
    if (useCalibrated)  //Bouguet's method - not hartley
    {
      Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
                cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
      rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
    }
    rimg.copyTo(img_rect[k]);
  }

  if (!isVerticalStereo)
    for (int j = 0; j < canvas.rows; j += 16)
      line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
  else
    for (int j = 0; j < canvas.cols; j += 16)
      line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);

//  //display the rectifid images
//  imshow("rectified_true", canvas);
//  waitKey(1);

  // outputing rect images
  img_rect[0].copyTo(left_rect_png_img);
  img_rect[1].copyTo(right_rect_png_img);

}// stereo_rectify_test

void
mStViPr::processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
{

  stereo.compute(*left_image, *right_image);
  stereo.medianFilter(10);  // better but slower //optimal

  // stereo images camera calibration parameters  --> simvis3d --> 1st --> the best
  float u_c = 130.00152015686035f; //1.3000152015686035e+02; //old sample -->379.85181427001953f; // the main one 3.7985181427001953e+02; // pcl //opencv - Cx = 3.8749671936035156e+02;
  float v_c = 118.91034603118896f; //1.1891034603118896e+02 //old sample --> 305.85922241210938f; // pcl //opencv - Cy = 3.1611055755615234e+02;
  float focal = 163.33793783988659f;  //1.6333793783988659e+02  --> old sample ==>//920.38355542932538f; //9.7814703939910510e+02;  //f
  float baseline = 0.291699576f; //47.645607158602786÷163.33793783988659 //47.645607158602786/163.33793783988659; //-4.7645607158602786e+01; old sample -->//0.359294689f; //real one using calculator
  CloudPtr cloud = out_cloud;
  pcl::PointCloud<pcl::RGB>::Ptr texture = left_image;
  stereo.getPointCloud(u_c, v_c, focal, baseline, cloud, texture);

}// processPair

/*----> the main terrain classification -------*/
void
mStViPr::processCloud(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  // Compute the normals
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(cloud);
  ne.compute(*normal_cloud);

  // Set up the ground plane comparator
  road_comparator->setInputCloud(cloud);
  road_comparator->setInputNormals(normal_cloud);

  // Run segmentation
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> region_indices;
  road_segmentation.setInputCloud(cloud);
  road_segmentation.segment(labels, region_indices);

  // Draw the segmentation result
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_image(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(new pcl::PointCloud<pcl::PointXYZRGB>);
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

  // green for traverable terrain
  for (int i = 0; i < region_indices.size(); i++)
  {
    if (region_indices[i].indices.size() > 1000)  //1000 original for min_inlier points green area
    {

      for (int j = 0; j < region_indices[i].indices.size(); j++)
      {
        pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                cloud->points[region_indices[i].indices[j]].y,
                                cloud->points[region_indices[i].indices[j]].z);
        ground_cloud->points.push_back(ground_pt);
        ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
        label_image->points[region_indices[i].indices[j]].r = 0;
        label_image->points[region_indices[i].indices[j]].g = 255;
        label_image->points[region_indices[i].indices[j]].b = 0;
      }// for j

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
      } //if

      model.values[0] = plane_params[0];
      model.values[1] = plane_params[1];
      model.values[2] = plane_params[2];
      model.values[3] = plane_params[3];
      model_coefficients.push_back(model);
      inlier_indices.push_back(region_indices[i]);
      centroids.push_back(clust_centroid);
      covariances.push_back(clust_cov);
    } // if
  } // for i

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
  mps.setMinInliers(500);  //min_inlier for yellow segments
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
  for (int i = 0; i < region_indices.size(); i++)
  {
    if (region_indices[i].indices.size() > 1000) //min_inliers yellow regions -- 1000 original
    {
      for (int j = 0; j < region_indices[i].indices.size(); j++)
      {
        // Check to see if it has already been labeled
        if (ground_image->points[region_indices[i].indices[j]].g == ground_image->points[region_indices[i].indices[j]].b)
        {
          pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                  cloud->points[region_indices[i].indices[j]].y,
                                  cloud->points[region_indices[i].indices[j]].z);
          ground_cloud->points.push_back(ground_pt);
          ground_image->points[region_indices[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].r + 255) / 2);
          ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
          label_image->points[region_indices[i].indices[j]].r = 128;
          label_image->points[region_indices[i].indices[j]].g = 128; //yellow
          label_image->points[region_indices[i].indices[j]].b = 0;
        }

      }
    }
  }

  // Segment Obstacles (enabled by default)
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
        if ((euclidean_label_indices[i].indices.size() > 100)) //min_inlier for obstacles -- 200 original
        {
          pcl::PointCloud<PointT> cluster;
          pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
          clusters.push_back(cluster);

          Eigen::Vector4f cluster_centroid;
          Eigen::Matrix3f cluster_cov;
          pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);

          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

          if ((ptp_dist > 0.5) && (ptp_dist < 3.0)) //min and max point to plane distance
          {

            for (int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
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
  for (int i = 0; i < cloud->points.size(); i++)
  {
    if (!pcl::isFinite(cloud->points[i]))
    {
      ground_image->points[i].b = static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
      label_image->points[i].r = 0;
      label_image->points[i].g = 0;
      label_image->points[i].b = 255;
    }
  }

  // Update info for the visualization thread
  {
    cloud_mutex.lock();
    prev_cloud = cloud;
    prev_normal_cloud = normal_cloud;
    prev_ground_cloud = ground_cloud;
    prev_ground_image = ground_image;
//    prev_ground_image = label_image;  //aras test
    prev_label_image = label_image;
    prev_ground_normal = ground_plane_params;
    prev_ground_centroid = ground_centroid;
    cloud_mutex.unlock();
  }
}

///* ----- terrain classification, modeling, traversability analysis and mapping ------*/
//void mStViPr::processCloud_new(const CloudConstPtr& cloud)
//{
//  // Compute the normals
//  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
//  ne.setInputCloud(cloud);
//  ne.compute(*normal_cloud);
//
//  // Set up the groundplane comparator
//  road_comparator->setInputCloud(cloud);
//  road_comparator->setInputNormals(normal_cloud);
//
//  // Run segmentation
//  pcl::PointCloud<pcl::Label> labels;
//  std::vector<pcl::PointIndices> region_indices;
//  road_segmentation.setInputCloud(cloud);
//  road_segmentation.segment(labels, region_indices);
//
//  // Draw the segmentation result
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  CloudPtr ground_image(new Cloud);
//  CloudPtr label_image(new Cloud);
//  *ground_image = *cloud;
//  *label_image = *cloud;
//
//
//
//  //// initializing fitting model //in this example //640*480 ==> image
//  const int num_cells_horizontal = 10; //number of cells???
//  const int num_cells_vertical = 10;
//  int img_width = ground_image->width;
//  int img_height = ground_image->height;
//  int cell_width = img_width / num_cells_horizontal; //64
//  int cell_height = img_height / num_cells_vertical; //48 //not needed
//  int total_number_cells = num_cells_horizontal * num_cells_vertical; //width * height  //100
//  int num_points_in_cell = cell_height * cell_width;
//  int thresh_green = 0.5 * num_points_in_cell; //025 //0.5 //0.75 //1.00
//  int thresh_yellow = 0.25 * num_points_in_cell;
//  int thresh_10percent = 0.1 * num_points_in_cell;
//  // clearing the counter array memory
//  int counter_green [num_cells_vertical][num_cells_horizontal]; //traversable
//  int counter_yellow [num_cells_vertical][num_cells_horizontal]; //semi
//  int counter_red [num_cells_vertical][num_cells_horizontal]; //obstacle
//  for (int i = 0; i < num_cells_vertical; i++) //i - row - y - vertival/verices - height
//  {
//    for (int j = 0; j < num_cells_horizontal; j++) //j - col - x horizintal/horizon - width
//    {
//      counter_green[i][j] = 0;
//      counter_yellow[i][j] = 0;
//      counter_red[i][j] = 0;
//    }//for j
//  }// for i
//
//
//  // 2- Compute plane info --> irrelevant to mapping but reauired for yellow and red cells
//  Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
//  Eigen::Vector4f vp = Eigen::Vector4f::Zero();
//  Eigen::Matrix3f clust_cov;
//  pcl::ModelCoefficients model;
//  model.values.resize(4);
//  ////
//  std::vector<pcl::ModelCoefficients> model_coefficients;
//  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
//  std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
//  std::vector<pcl::PointIndices> inlier_indices;
//
//
//
//  // counting the green points in all cells
//  for (int i = 0; i < region_indices.size() ; i++) //col //region_indices.size()
//  {
//    if (region_indices[i].indices.size() > 1000)
//    {
//      // green area and traversable
//      for (int j = 0; j < region_indices[i].indices.size() ; j++) //row  //region_indices[i].indices.size()
//      {
//        // x,y //row, col ==> location conversion from indices to row,col -- x,y -- width,height -- i,j
//        int current_point = region_indices[i].indices[j];
//        int img_col = current_point % img_width; //x col width j
//        int img_row = abs(current_point / img_width); //y row height i
////          ground_image->at(img_col, img_row).g = static_cast<uint8_t>((cloud->at(img_col, img_row).g + 255) / 2); //all grean area and points
//
//        // cpunting the points in cells and dividing into cells
//        int cell_col = abs(img_col / cell_width); //from 1 to 10 vertical //array starts at 0 not 1
//        int cell_row = abs(img_row / cell_height); //from 1 to 10 horizontal
//        int cell_counter = ++counter_green[cell_row][cell_col];
//      }//for j
//
//
//
//      // 2- Compute plane info --> irrelevant to mapping but required for yellow and red
//      pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);
//      Eigen::Vector4f plane_params;
//      ////
//      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//      pcl::eigen33(clust_cov, eigen_value, eigen_vector);
//      plane_params[0] = eigen_vector[0];
//      plane_params[1] = eigen_vector[1];
//      plane_params[2] = eigen_vector[2];
//      plane_params[3] = 0;
//      plane_params[3] = -1 * plane_params.dot(clust_centroid);
//      ////
//      vp -= clust_centroid;
//      float cos_theta = vp.dot(plane_params);
//      if (cos_theta < 0)
//      {
//        plane_params *= -1;
//        plane_params[3] = 0;
//        plane_params[3] = -1 * plane_params.dot(clust_centroid);
//      }
//      ////
//      model.values[0] = plane_params[0];
//      model.values[1] = plane_params[1];
//      model.values[2] = plane_params[2];
//      model.values[3] = plane_params[3];
//      model_coefficients.push_back(model);
//      inlier_indices.push_back(region_indices[i]);
//      centroids.push_back(clust_centroid);
//      covariances.push_back(clust_cov);
//
//    } // if region_indices
//
//  } //for regin_indices
//
//
//  // Refinement ==> yellow points
//  std::vector<bool> grow_labels;
//  std::vector<int> label_to_model;
//  grow_labels.resize(region_indices.size(), false);
//  label_to_model.resize(region_indices.size(), 0);
//
//  for (size_t i = 0; i < model_coefficients.size(); i++)
//  {
//    int model_label = (labels)[inlier_indices[i].indices[0]].label;
//    label_to_model[model_label] = static_cast<int>(i);
//    grow_labels[model_label] = true;
//  }
//  ////
//  boost::shared_ptr<pcl::PointCloud<pcl::Label> > labels_ptr(new pcl::PointCloud<pcl::Label>());
//  *labels_ptr = labels;
//  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
//  pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
//  refinement_compare->setInputCloud(cloud);
//  refinement_compare->setDistanceThreshold(0.15f);
//  refinement_compare->setLabels(labels_ptr);
//  refinement_compare->setModelCoefficients(model_coefficients);
//  refinement_compare->setRefineLabels(grow_labels);
//  refinement_compare->setLabelToModel(label_to_model);
//  mps.setRefinementComparator(refinement_compare);
//  mps.setMinInliers(500);
//  mps.setAngularThreshold(pcl::deg2rad(3.0));
//  mps.setDistanceThreshold(0.02);
//  mps.setInputCloud(cloud);
//  mps.setInputNormals(normal_cloud);
//  mps.refine(model_coefficients,
//             inlier_indices,
//             centroids,
//             covariances,
//             labels_ptr,
//             region_indices);
//
//  // yellow regions that have been extended ==> semi-traversable
//  Cloud extended_ground_cloud;
//  for (int i = 0; i < region_indices.size(); i++)
//  {
//    if (region_indices[i].indices.size() > 1000)
//    {
//      for (int j = 0; j < region_indices[i].indices.size(); j++)
//      {
//        // Check to see if it has already been labeled ==> ground plane and obstacle info
//        if (ground_image->points[region_indices[i].indices[j]].g == ground_image->points[region_indices[i].indices[j]].b)
//        {
//          // x,y //row, col ==> location conversion from indices to row,col -- x,y -- width,height -- i,j
//          int current_point = region_indices[i].indices[j];
//          int img_col = current_point % img_width; //x col width j
//          int img_row = abs(current_point / img_width); //y row height i
////            ground_image->at(img_col, img_row).r = static_cast<uint8_t>((cloud->at(img_col, img_row).r + 255) / 2);
////            ground_image->at(img_col, img_row).g = static_cast<uint8_t>((cloud->at(img_col, img_row).g + 255) / 2); //all grean area and points
//
//          // counting the points in cells and dividing into cells
//          int cell_col = abs(img_col / cell_width); //from 1 to 10 vertical //array starts at 0 not 1
//          int cell_row = abs(img_row / cell_height); //from 1 to 10 horizontal
//          int cell_counter = ++counter_yellow[cell_row][cell_col];
//
//          // irrelevant to yellow cells but needed for ground plane and red cells
//          pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
//                                  cloud->points[region_indices[i].indices[j]].y,
//                                  cloud->points[region_indices[i].indices[j]].z);
//          ground_cloud->points.push_back(ground_pt);
//        } // if
//      } // for j
//    }// if
//  } //for int i - regions
//
//  // ground plane info and obstacle info
//  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
//  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
//  //
//  if (ground_cloud->points.size() > 0)
//  {
//    ground_centroid = centroids[0];
//    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
//  } //if
//
//  // red cells ==> segment obstacles
//  if (detect_obstacles)
//  {
//    Cloud::CloudVectorType clusters;
//    if (ground_cloud->points.size() > 0)
//    {
//      std::vector<bool> plane_labels;
//      plane_labels.resize(region_indices.size(), false);
//      for (size_t i = 0; i < region_indices.size(); i++)
//      {
//        if (region_indices[i].indices.size() > mps.getMinInliers())
//        {
//          plane_labels[i] = true;
//        }
//      }
//
//      pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
//
//      euclidean_cluster_comparator_->setInputCloud(cloud);
//      euclidean_cluster_comparator_->setLabels(labels_ptr);
//      euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
//      euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);
//
//      pcl::PointCloud<pcl::Label> euclidean_labels;
//      std::vector<pcl::PointIndices> euclidean_label_indices;
//      pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
//      euclidean_segmentation.setInputCloud(cloud);
//      euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);
//
//      for (size_t i = 0; i < euclidean_label_indices.size(); i++)
//      {
//        if ((euclidean_label_indices[i].indices.size() > 200))
//        {
//          Cloud cluster;
//          pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
//          clusters.push_back(cluster);
//
//          Eigen::Vector4f cluster_centroid;
//          Eigen::Matrix3f cluster_cov;
//          pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          if ((ptp_dist > 0.5) && (ptp_dist < 3.0))
//          {
//
//            for (int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
//            {
//              // x,y //row, col ==> location conversion from indices to row,col -- x,y -- width,height -- i,j
////        int current_point = region_indices[i].indices[j]; //old
//              int current_point = euclidean_label_indices[i].indices[j];
//              int img_col = current_point % img_width; //x col width j
//              int img_row = abs(current_point / img_width); //y row height i
//              //ground_image->at(img_col, img_row).r = static_cast<uint8_t>((cloud->at(img_col, img_row).r + 255) / 2);
//              //ground_image->at(img_col, img_row).g = static_cast<uint8_t>((cloud->at(img_col, img_row).g + 255) / 2); //all grean area and points
//              //ground_image->points[euclidean_label_indices[i].indices[j]].r = 255;
//              //ground_image->at(img_col, img_row).r = 255; //main //working!!
//
//              // counting the points in cells and dividing into cells
//              int cell_col = abs(img_col / cell_width); //from 1 to 10 vertical //array starts at 0 not 1
//              int cell_row = abs(img_row / cell_height); //from 1 to 10 horizontal
//              int cell_counter = ++counter_red[cell_row][cell_col];
//
//            }
//
//          } //if
//
//        }// if
//      } //for
//
//    } //if ground_cloud
//  } // if obstacles
//
//
//
//  // analyzing all the points in a cell for traversability - traversability analysis
//  for (int i = 0; i < num_cells_vertical; i++)  //row y height i
//  {
//    for (int j = 0; j < num_cells_horizontal; j++)  //col x width j
//    {
//
//      int cell_starting_point_row = ((i) * cell_height); //margin==0
//      int cell_end_point_row = ((i + 1) * cell_height) ; //margin
//      int cell_starting_point_col = ((j) * cell_width); //margin
//      int cell_end_point_col = ((j + 1) * cell_width); //margin
//      for (int row = cell_starting_point_row; row < cell_end_point_row; row++)
//        for (int col = cell_starting_point_col; col < cell_end_point_col; col++)
//        {
//          bool is_red = false;
//
//          //red cells
//          if (counter_red[i][j] > thresh_10percent)
//          {
//            ground_image->at(col, row).r = 255;
//            is_red = true;
//          }// if red
//
//          // green cells
//          if (!is_red && counter_green[i][j] > thresh_green)
//          {
//            ground_image->at(col, row).g = 255;
//            is_red = true;
//          }//if
//
//          // yellow cells - semi
//          if (!is_red &&  counter_green[i][j] < thresh_green && counter_green[i][j] > thresh_yellow
//              ||
//              !is_red && counter_yellow[i][j] > thresh_10percent)  //  //cells with from 0.25 to 0.5 //blue
//          {
//            ground_image->at(col, row).b = 255; //tocheck --> supposed to be yellow like traffic lights but it s hard to tell it appart from green sometimes
////          ground_image->at(col, row).g = 255;
////          ground_image->at(col, row).r = 255;
//          } // if yellow or blue
//        }// for 2nd one --> col
//
//    }//for j
//  } //for i
//
//  // drawing the mapping model on image //model fitting on image ==> to know about the location of cells
//  for (int i = 0; i < num_cells_vertical; i++) //height
//    for (int j = 0; j < img_width; j++) //width
//    {
//      ground_image->at(j, i * cell_height).rgb = 0; //fitting the model on terrain
//    }//for j
//  for (int i = 0; i < img_height; i++) //height
//    for (int j = 0; j < num_cells_horizontal; j++) //width
//    {
//      ground_image->at(j * cell_width, i).rgb = 0; //fitting the model on terrain
//    }//for j
//
//
//  // Update info for the visualization thread
//  {
//    cloud_mutex.lock();
//    prev_cloud = cloud;
//    prev_normal_cloud = normal_cloud;
//    prev_ground_image = ground_image;
//    prev_ground_normal = ground_plane_params; //ground plan info
//    prev_ground_centroid = ground_centroid; // ground plane info
//    cloud_mutex.unlock();
//  }
//
//}// processClooud


///* ----- test ----> terrain classification, modeling, traversability analysis and mapping ------*/
//void mStViPr::processCloud_new_test(const CloudConstPtr& cloud)
//{
//  // Compute the normals
//  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
//  ne.setInputCloud(cloud);
//  ne.compute(*normal_cloud);
//
//  // Set up the groundplane comparator
//  road_comparator->setInputCloud(cloud);
//  road_comparator->setInputNormals(normal_cloud);
//
//  // Run segmentation
//  pcl::PointCloud<pcl::Label> labels;
//  std::vector<pcl::PointIndices> region_indices;
//  road_segmentation.setInputCloud(cloud);
//  road_segmentation.segment(labels, region_indices);
//
//  // Draw the segmentation result
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  CloudPtr ground_image(new Cloud);
//  CloudPtr label_image(new Cloud);
//  *ground_image = *cloud;
//  *label_image = *cloud;
//
//
//
//  //// initializing fitting model //in this example //640*480 ==> image
//  const int num_cells_horizontal = 5; //10 //number of cells???
//  const int num_cells_vertical = 5; //10
//  int img_width = ground_image->width;
//  int img_height = ground_image->height;
//  int cell_width = img_width / num_cells_horizontal; //64
//  int cell_height = img_height / num_cells_vertical; //48 //not needed
//  int total_number_cells = num_cells_horizontal * num_cells_vertical; //width * height  //100
//  int num_points_in_cell = cell_height * cell_width;
//  int thresh_green = 0.5 * num_points_in_cell; //025 //0.5 //0.75 //1.00
//  int thresh_yellow = 0.25 * num_points_in_cell;
//  int thresh_10percent = 0.1 * num_points_in_cell;
//  int thresh_90percent = 0.9 * num_points_in_cell;
//
//  // clearing the counter array memory
//  int counter_green [num_cells_vertical][num_cells_horizontal]; //traversable
//  int counter_yellow [num_cells_vertical][num_cells_horizontal]; //semi
//  int counter_red [num_cells_vertical][num_cells_horizontal]; //obstacle
//  for (int i = 0; i < num_cells_vertical; i++) //i - row - y - vertival/verices - height
//  {
//    for (int j = 0; j < num_cells_horizontal; j++) //j - col - x horizintal/horizon - width
//    {
//      counter_green[i][j] = 0;
//      counter_yellow[i][j] = 0;
//      counter_red[i][j] = 0;
//    }//for j
//  }// for i
//
//
//  // 2- Compute plane info --> irrelevant to mapping but reauired for yellow and red cells
//  Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
//  Eigen::Vector4f vp = Eigen::Vector4f::Zero();
//  Eigen::Matrix3f clust_cov;
//  pcl::ModelCoefficients model;
//  model.values.resize(4);
//  ////
//  std::vector<pcl::ModelCoefficients> model_coefficients;
//  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
//  std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
//  std::vector<pcl::PointIndices> inlier_indices;
//
//
//
//  /*
//   * counting the green points in all cells
//   */
//  for (int i = 0; i < region_indices.size() ; i++) //col //region_indices.size()
//  {
//    if (region_indices[i].indices.size() > 1000)
//    {
//      /*! green area and traversable*/
//      for (int j = 0; j < region_indices[i].indices.size() ; j++) //row  //region_indices[i].indices.size()
//      {
//        /*! row, col ==> location conversion from indices to row,col -- x,y -- width,height -- i,j*/
//        int current_point = region_indices[i].indices[j];
//        int img_col = current_point % img_width; //x col width j
//        int img_row = abs(current_point / img_width); //y row height i
//
//        // filtered
//        ground_image->at(img_col, img_row).r = 0;
//        ground_image->at(img_col, img_row).g = 255;
//        ground_image->at(img_col, img_row).b = 0;
//
//        // original
//        label_image->at(img_col, img_row).r = 0;
//        label_image->at(img_col, img_row).g = 255;
//        label_image->at(img_col, img_row).b = 0;
//
//
//        /*! counting the points in cells and dividing into cells*/
//        int cell_col = abs(img_col / cell_width); //from 1 to 10 vertical //array starts at 0 not 1
//        int cell_row = abs(img_row / cell_height); //from 1 to 10 horizontal
//        int cell_counter = ++counter_green[cell_row][cell_col];
//      }//for j
//
//
//
//      // 2- Compute plane info --> irrelevant to mapping but required for yellow and red
//      pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);
//      Eigen::Vector4f plane_params;
//      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//      pcl::eigen33(clust_cov, eigen_value, eigen_vector);
//      plane_params[0] = eigen_vector[0];
//      plane_params[1] = eigen_vector[1];
//      plane_params[2] = eigen_vector[2];
//      plane_params[3] = 0;
//      plane_params[3] = -1 * plane_params.dot(clust_centroid);
//      vp -= clust_centroid;
//      float cos_theta = vp.dot(plane_params);
//      if (cos_theta < 0)
//      {
//        plane_params *= -1;
//        plane_params[3] = 0;
//        plane_params[3] = -1 * plane_params.dot(clust_centroid);
//      }
//      ////
//      model.values[0] = plane_params[0];
//      model.values[1] = plane_params[1];
//      model.values[2] = plane_params[2];
//      model.values[3] = plane_params[3];
//      model_coefficients.push_back(model);
//      inlier_indices.push_back(region_indices[i]);
//      centroids.push_back(clust_centroid);
//      covariances.push_back(clust_cov);
//
//    } // if region_indices
//
//  } //for regin_indices
//
//
//  /*
//   * Refinement ==> yellow points
//   */
//  std::vector<bool> grow_labels;
//  std::vector<int> label_to_model;
//  grow_labels.resize(region_indices.size(), false);
//  label_to_model.resize(region_indices.size(), 0);
//
//  for (size_t i = 0; i < model_coefficients.size(); i++)
//  {
//    int model_label = (labels)[inlier_indices[i].indices[0]].label;
//    label_to_model[model_label] = static_cast<int>(i);
//    grow_labels[model_label] = true;
//  }
//  ////
//  boost::shared_ptr<pcl::PointCloud<pcl::Label> > labels_ptr(new pcl::PointCloud<pcl::Label>());
//  *labels_ptr = labels;
//  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
//  pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
//  refinement_compare->setInputCloud(cloud);
//  refinement_compare->setDistanceThreshold(0.15f);
//  refinement_compare->setLabels(labels_ptr);
//  refinement_compare->setModelCoefficients(model_coefficients);
//  refinement_compare->setRefineLabels(grow_labels);
//  refinement_compare->setLabelToModel(label_to_model);
//  mps.setRefinementComparator(refinement_compare);
//  mps.setMinInliers(500);
//  mps.setAngularThreshold(pcl::deg2rad(3.0));
//  mps.setDistanceThreshold(0.02);
//  mps.setInputCloud(cloud);
//  mps.setInputNormals(normal_cloud);
//  mps.refine(model_coefficients,
//             inlier_indices,
//             centroids,
//             covariances,
//             labels_ptr,
//             region_indices);
//
//  /*! yellow regions that have been extended ==> semi-traversable*/
//  Cloud extended_ground_cloud;
//  for (int i = 0; i < region_indices.size(); i++)
//  {
//    if (region_indices[i].indices.size() > 1000)
//    {
//      for (int j = 0; j < region_indices[i].indices.size(); j++)
//      {
//        /*! Check to see if it has already been labeled ==> ground plane and obstacle info*/
//        if (ground_image->points[region_indices[i].indices[j]].g == ground_image->points[region_indices[i].indices[j]].b)
//        {
//          // x,y //row, col ==> location conversion from indices to row,col -- x,y -- width,height -- i,j
//          int current_point = region_indices[i].indices[j];
//          int img_col = current_point % img_width; //x col width j
//          int img_row = abs(current_point / img_width); //y row height i
//
//          // filtered
//          ground_image->at(img_col, img_row).r = 128; //main //working!!
//          ground_image->at(img_col, img_row).g = 128; //main //working!!
//          ground_image->at(img_col, img_row).b = 0; //main //working!!
//
//          // original
//          label_image->at(img_col, img_row).r = 128; //main //working!!
//          label_image->at(img_col, img_row).g = 128; //main //working!!
//          label_image->at(img_col, img_row).b = 0; //main //working!!
//
//
//          // counting the points in cells and dividing into cells
//          int cell_col = abs(img_col / cell_width); //from 1 to 10 vertical //array starts at 0 not 1
//          int cell_row = abs(img_row / cell_height); //from 1 to 10 horizontal
//          int cell_counter = ++counter_yellow[cell_row][cell_col];
//
//          // irrelevant to yellow cells but needed for ground plane and red cells
//          pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
//                                  cloud->points[region_indices[i].indices[j]].y,
//                                  cloud->points[region_indices[i].indices[j]].z);
//          ground_cloud->points.push_back(ground_pt);
//        } // if
//      } // for j
//    }// if
//  } //for int i - regions
//
//  /*
//   * ground plane info and obstacle info
//   */
//  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
//  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
//  if (ground_cloud->points.size() > 0)
//  {
//    ground_centroid = centroids[0];
//    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
//  } //if
//
//  /*
//   * red cells ==> segment obstacles
//   */
//  if (detect_obstacles)
//  {
//    Cloud::CloudVectorType clusters;
//    if (ground_cloud->points.size() > 0)
//    {
//      std::vector<bool> plane_labels;
//      plane_labels.resize(region_indices.size(), false);
//      for (size_t i = 0; i < region_indices.size(); i++)
//      {
//        if (region_indices[i].indices.size() > mps.getMinInliers())
//        {
//          plane_labels[i] = true;
//        }
//      }
//
//      pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
//
//      euclidean_cluster_comparator_->setInputCloud(cloud);
//      euclidean_cluster_comparator_->setLabels(labels_ptr);
//      euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
//      euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);
//
//      pcl::PointCloud<pcl::Label> euclidean_labels;
//      std::vector<pcl::PointIndices> euclidean_label_indices;
//      pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
//      euclidean_segmentation.setInputCloud(cloud);
//      euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);
//
//      for (size_t i = 0; i < euclidean_label_indices.size(); i++)
//      {
//        if ((euclidean_label_indices[i].indices.size() > 200))
//        {
//          Cloud cluster;
//          pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
//          clusters.push_back(cluster);
//
//          Eigen::Vector4f cluster_centroid;
//          Eigen::Matrix3f cluster_cov;
//          pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);
//
//          pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//          double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          if ((ptp_dist > 0.5) && (ptp_dist < 3.0))
//          {
//
//            for (int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
//            {
//              /*
//               * row, col ==> location conversion from indices to row,col -- x,y -- width,height -- i,j
//               */
//              //int current_point = region_indices[i].indices[j]; //old
//              int current_point = euclidean_label_indices[i].indices[j];
//              int img_col = current_point % img_width; //x col width j
//              int img_row = abs(current_point / img_width); //y row height i
//
//              /*! counting the points in cells and dividing into cells */
//              int cell_col = abs(img_col / cell_width); //from 1 to 10 vertical //array starts at 0 not 1
//              int cell_row = abs(img_row / cell_height); //from 1 to 10 horizontal
//              int cell_counter = ++counter_red[cell_row][cell_col];
//
//              label_image->at(img_col, img_row).r = 255;
//              label_image->at(img_col, img_row).g = 0;
//              label_image->at(img_col, img_row).b = 0;
//
//            }
//
//          } //if
//
//        }// if
//      } //for
//
//    } //if ground_cloud
//  } // if obstacles
//
//
//
//  /*
//   * analyzing all the points in a cell for traversability - traversability analysis
//   */
//  for (int i = 0; i < num_cells_vertical; i++)  //row y height i
//  {
//    for (int j = 0; j < num_cells_horizontal; j++)  //col x width j
//    {
//
//      int cell_starting_point_row = ((i) * cell_height); //margin==0
//      int cell_end_point_row = ((i + 1) * cell_height) ; //margin
//      int cell_starting_point_col = ((j) * cell_width); //margin
//      int cell_end_point_col = ((j + 1) * cell_width); //margin
//      for (int row = cell_starting_point_row; row < cell_end_point_row; row++)
//        for (int col = cell_starting_point_col; col < cell_end_point_col; col++)
//        {
//          bool is_red = false;
//
//          /*! red cells*/
//          if (counter_red[i][j] > thresh_90percent)  //thresh_10percent)
//          {
//            ground_image->at(col, row).r = 255;
//            ground_image->at(col, row).g = 0;
//            ground_image->at(col, row).b = 0;
////            label_image->at(col, row).r = 255;
////            label_image->at(col, row).g = 0;
////            label_image->at(col, row).b = 0;
//            is_red = true;
//          }// if red
//
//        }// for 2nd one --> col
//
//    }//for j
//  } //for i
//
//
//  /*! note the NAN points in the image as well*/
//  for (int i = 0; i < cloud->points.size(); i++)
//  {
//    if (!pcl::isFinite(cloud->points[i]))
//    {
//      ground_image->points[i].r = 0;
//      ground_image->points[i].g = 0;
//      ground_image->points[i].b = 255; //original terrain classification
//
//      label_image->points[i].r = 0;
//      label_image->points[i].g = 0;
//      label_image->points[i].b = 255; //filtered
//    }
//  }
//
//
////  /*
////   * drawing the mapping model on image -- grid model fitting on image ==> to know about the location of cells
////   */
////  for (int i = 0; i < num_cells_vertical; i++) //height
////    for (int j = 0; j < img_width; j++) //width
////    {
////      ground_image->at(j, i * cell_height).rgb = 0; //fitting the model on terrain
////    }//for j
////  for (int i = 0; i < img_height; i++) //height
////    for (int j = 0; j < num_cells_horizontal; j++) //width
////    {
////      ground_image->at(j * cell_width, i).rgb = 0; //fitting the model on terrain
////    }//for j
//
//
//  /*
//   * Update info for the visualization thread
//   */
//  {
//    cloud_mutex.lock();
//    prev_cloud = cloud;
//    prev_normal_cloud = normal_cloud;
//    prev_ground_image = ground_image;
//    prev_label_image = label_image;
//    prev_ground_normal = ground_plane_params; //ground plan info
//    prev_ground_centroid = ground_centroid; // ground plane info
//    cloud_mutex.unlock();
//  }
//
//}// processClooud
//

void
mStViPr::saveCloud(const CloudConstPtr& cloud)
{
  std::stringstream ss;
  ss << dir_name_ << "/" << file_name_ << "_" << images_idx << ".pcd";
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
  images_idx ++;
}

void
mStViPr::run()
{

  stereo_rectify(left_rect_png_img, right_rect_png_img);

  pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
  pcl::PointCloud<pcl::RGB>::Ptr right_cloud(new pcl::PointCloud<pcl::RGB>);
  CloudPtr out_cloud(new Cloud);

  //png2pcd -- mat img input
  left_cloud = png2pcd(left_rect_png_img);
  right_cloud = png2pcd(right_rect_png_img);

  // process the disparity map and point cloud
  processStereoPair(left_cloud, right_cloud, out_cloud);
  processCloud(out_cloud); //main for terrain lassification without filtering
//  processCloud_new(out_cloud); //modelling the terrain classification
//  processCloud_new_test(out_cloud); //main for filtering the obtacle noise

  ///displaying disparity map
  pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
  stereo.getVisualMap(vmap);
  image_viewer_disparity->addRGBImage<pcl::RGB> (vmap);
  image_viewer_disparity_2->addRGBImage<pcl::RGB> (vmap);  //debugging!!


  // Draw visualizations
  if (cloud_mutex.try_lock())
  {
    if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
      viewer->addPointCloud(prev_ground_image, "cloud");

    if (prev_normal_cloud->points.size() > 1000 && display_normals)
    {
      viewer->removePointCloud("normals");
      viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
    }

    if (prev_cloud->points.size() > 1000)
    {
      image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
      image_viewer_original->addRGBImage<PointT>(prev_label_image, "rgb_image", 0.3);
    }

    // Show the ground plane normal
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);

    // Adjust for camera tilt
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;

    // Show the ground plane normal
    pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
    pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                      prev_ground_centroid[1] + prev_ground_normal[1],
                      prev_ground_centroid[2] + prev_ground_normal[2]);
    pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                      prev_ground_centroid[1] + tilt_road_normal[1],
                      prev_ground_centroid[2] + tilt_road_normal[2]);

    viewer->removeShape("ground_norm");
    viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
    viewer->removeShape("expected_ground_norm");
    viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");

    cloud_mutex.unlock();
  } //if cloud_mutex

  viewer->spinOnce(1);
  image_viewer->spinOnce(1);
  image_viewer_original->spinOnce(1);
  image_viewer_disparity->spinOnce(1);
  image_viewer_disparity_2->spinOnce(1);
//  char key = waitKey(1); //opencv windows and display

}// run

void
mStViPr::display_images()
{
  namedWindow("left_rect_png_img", 0);
  imshow("left_rect_png_img", left_rect_png_img);
  namedWindow("right_rect_png_img", 0);
  imshow("right_rect_png_img", right_rect_png_img);
  waitKey(1); //to diplay the windows
}

void
mStViPr::saving_frames_right_left_folders(const Mat imgl, const Mat imgr)
{
  char key = waitKey(1);
  if (key == ' ')
  {
    char filename[100];
    sprintf(filename, "/home/aras/stereo_images/simvis3d-camera/frames_img_left/left%.2d.png", images_idx_saveImage);
    imwrite(filename, imgl);
    cout << "Saved " << filename << endl;
    sprintf(filename, "/home/aras/stereo_images/simvis3d-camera/frames_img_right/right%.2d.png", images_idx_saveImage);
    imwrite(filename, imgr);
    cout << "Saved " << filename << endl;
    images_idx_saveImage ++;  //not working
  }// if key
}

void
mStViPr::chessboard_detection()
{
  //1-variable initialization and declaration
  enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
  enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

  //chessboard
  Size boardSize, imageSize;
  boardSize.width = 9;
  boardSize.height = 6;
  vector<vector<Point2f>> imagePoints;
  Pattern pattern = CHESSBOARD;
  vector<Point2f> pointbuf;

  //2-chess board detection module -------------------------------------> right camera
  Mat view, view_chessboard, viewGray;
  right_rect_png_img.copyTo(view);
  imageSize = view.size();

  //view.copyTo(viewGray); //gray scale camera like mine
  cvtColor(view, viewGray, CV_BGR2GRAY); //color camera like tobias's
  bool found = false;
  found = findChessboardCorners(viewGray, boardSize, pointbuf,
                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //stereo_calib

  // improve the found corners' coordinate accuracy
  if (pattern == CHESSBOARD && found)
    cornerSubPix(viewGray, pointbuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01)); //0.1 default
  if (found)   //&& (clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC) //!capture.isOpened() ||  //mode == CAPTURING &&
  {
    imagePoints.push_back(pointbuf);
  }
  //////////
  view.copyTo(view_chessboard);
  //cvtColor(view, view, CV_GRAY2BGR);
  if (found)
    drawChessboardCorners(view, boardSize, Mat(pointbuf), found);

  ///////save the image with the found chessboard  //aras
  namedWindow("ImageView_right_0", 1);
  imshow("ImageView_right_0", view);
  ///////
  Mat frame_found;
  if (found)
    view_chessboard.copyTo(frame_found);

  //3-chessboard detection ----------------------------------------------->left camera
  Mat viewl, viewGrayl, viewl_chessboard;
  left_rect_png_img.copyTo(viewl);
  imageSize = viewl.size();

  //  viewl.copyTo(viewGrayl); //grayscale camera like mine
  cvtColor(viewl, viewGrayl, CV_BGR2GRAY); //colorful camera like tobias's
  bool found_1 = false;
  found_1 = findChessboardCorners(viewGrayl, boardSize, pointbuf,
                                  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);  //CV_CALIB_CB_FAST_CHECK --> default


  // improve the found corners' coordinate accuracy
  if (pattern == CHESSBOARD && found_1)
    cornerSubPix(viewGrayl, pointbuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));

  if (found_1)  //&& (clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC)//mode == CAPTURING &&   //!capture_1.isOpened() ||
  {
    imagePoints.push_back(pointbuf);
  }//if

  viewl.copyTo(viewl_chessboard);
  //cvtColor(viewl, viewl, CV_GRAY2BGR);

  if (found_1)
    drawChessboardCorners(viewl, boardSize, Mat(pointbuf), found_1);

  ///////save the image with the found chessboard  //aras
  namedWindow("ImageView_left_1", 1);
  imshow("ImageView_left_1", viewl);
  ///////
  Mat frame_found_1;
  if (found_1)
    viewl_chessboard.copyTo(frame_found_1);

  ///////----------------------------------------------->saving  images
  char filename[200];
  char key = waitKey(1);
  if (found && found_1 && key == ' ')
  {
    sprintf(filename, "/home/aras/stereo_images/simvis3d-camera/calib/right%.2d.jpg", n_r++);
    imwrite(filename, frame_found);
    cout << "Saved " << filename << endl;
    ////
    sprintf(filename, "/home/aras/stereo_images/simvis3d-camera/calib/left%.2d.jpg", n_l++);
    imwrite(filename, frame_found_1);
    cout << "Saved " << filename << endl;
  }//if
}// cheesboard detection

void
mStViPr::gridmap_stereo()
{

  /* --------------> robot pose <---------------------*/
  /*---------------> front laser scanner <----------------------*/
  // reading the port
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data = si_distance_data_front_scanner.GetPointer();

  // access robot pose from source ... init absolute_sensor_pose
  rrlib::math::tPose3D absolute_sensor_pose(distance_data->RobotPose());

  // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
  absolute_sensor_pose.ApplyRelativePoseTransformation(distance_data->SensorPose());
  absolute_sensor_pose.Scale(0.001); // for displaying the minimum distance sample we need unit meter

  //saving the transformation matrix
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose.Position());
  rrlib::math::tVec3f pose_front_scanner = absolute_sensor_pose.Position();

  /*---------------> rear laser scanner <----------------------*/
  // reading the port
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data_rear_scanner = si_distance_data_rear_scanner.GetPointer();

  // access robot pose from source ... absolute_sensor_pose
  rrlib::math::tPose3D absolute_sensor_pose_rear_scanner(distance_data_rear_scanner->RobotPose());

  // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
  absolute_sensor_pose_rear_scanner.ApplyRelativePoseTransformation(distance_data_rear_scanner->SensorPose());
  absolute_sensor_pose_rear_scanner.Scale(0.001); // for displaying the minimum distance sample we need unit meter

  //saving the transformation matrix
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_rear_scanner.Position());
  rrlib::math::tVec3f pose_rear_scanner = absolute_sensor_pose_rear_scanner.Position();


  /* --------------> stereo <---------------------*/
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> map_pointer_stereo = so_map_grid_stereo.GetUnusedBuffer();
  rrlib::mapping::tMapGridCartesian2D<double>& map_stereo = *map_pointer_stereo;
  map_stereo.Clear(0);

  // 20 by 20 is the size of the grid and 0.1 for cells
  map_stereo.SetBounds(bounds_stereo_env);
  map_stereo.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1));  //0.1 cell resolution is one meter in simulation //LUGV 3m - 3 cell size - 0.3 in grid map

  /*! reset*/
  if (ci_reset_grdimap_stereo_env.Get() == 1)
  {
    map_stereo_env.Clear(0);
  }

  //sensor pose
  // access robot pose from source ... absolute_sensor_pose
  rrlib::math::tPose3D absolute_sensor_pose_stereo(distance_data->RobotPose()); //front scanner
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_stereo.Roll());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_stereo.Pitch());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_stereo.Yaw());


  // access robot relative sensor pose from source ... apply this offset to obtain sensor pose in world coordinates
  absolute_sensor_pose_stereo.ApplyRelativePoseTransformation(distance_data->SensorPose());
  absolute_sensor_pose_stereo.Scale(0.001); // for displaying the minimum distance sample we need unit meter

  //saving the transformation matrix
  const rrlib::math::tMat4x4d robot2sensorTransformationMatrix_stereo = absolute_sensor_pose_stereo.GetTransformationMatrix();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose_stereo.Position());


  rrlib::math::tVec3d pose_stereo = absolute_sensor_pose_stereo.Position();
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "pose_stereo.X: ", pose_stereo.X());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "pose_stereo.Y: ", pose_stereo.Y());

  rrlib::math::tMat3x3d rotation_90_to_right = rrlib::math::tPose3D(0, 0, 0, 0, 0, -1.57).GetRotationMatrix(); // -1.57//-90 degree to right
  rrlib::math::tMat4x4d rotation_90_to_right_mat4x4 = rrlib::math::tMat4x4d(rotation_90_to_right[0][0], rotation_90_to_right[0][1], rotation_90_to_right[0][2], 0,
      rotation_90_to_right[1][0], rotation_90_to_right[1][1], rotation_90_to_right[1][2], 0,
      rotation_90_to_right[2][0], rotation_90_to_right[2][1], rotation_90_to_right[2][2], 0,
      0, 0, 0, 1);

  const rrlib::math::tMat3x3d rotation_robot2world = absolute_sensor_pose_stereo.GetRotationMatrix();
  rrlib::math::tMat4x4d transformation_robot2world_mat4x4 = rrlib::math::tMat4x4d(rotation_robot2world[0][0], rotation_robot2world[0][1], rotation_robot2world[0][2], absolute_sensor_pose_stereo.X() / 10, //absolute_sensor_pose_stereo.X()
      rotation_robot2world[1][0], rotation_robot2world[1][1], rotation_robot2world[1][2], absolute_sensor_pose_stereo.Y() / 10, //absolute_sensor_pose_stereo.X()
      rotation_robot2world[2][0], rotation_robot2world[2][1], rotation_robot2world[2][2], 0,
      0, 0, 0, 1);


  /*filling out the grid cells for rear scanner*/
  int counterNumPointsInEachCell_stereo[map_stereo.GetNumberOfCells()];
  for (int i = 0; i < map_stereo.GetNumberOfCells(); i++)
  {
    counterNumPointsInEachCell_stereo[i] = 0;
  }// for

  /*filling out the grid cells for rear scanner*/
  int counterNumPointsInEachCell_stereo_env[map_stereo_env.GetNumberOfCells()];
  for (int i = 0; i < map_stereo_env.GetNumberOfCells(); i++)
  {
    counterNumPointsInEachCell_stereo_env[i] = 0;
  }// for


  /*
   * --------------------- environment mapping ----------------
   */
  // drawing the mapping model on image //model fitting on image ==> to know about the location of cells
  for (int i = 0; i < prev_ground_image->height; i++) //height y row  vertical
  {
    for (int j = 0; j < prev_ground_image->width; j++) //width x column  horizon horizontal
    {

      /*
       * ! if obstacle --> red
       * */
      if (prev_ground_image->at(j, i).r == 255 && prev_ground_image->at(j, i).g == 0 && prev_ground_image->at(j, i).b == 0) //red obstacle
      {

        /*
         * thresholding and filtering the outliers
         */
        /*! using the bounding box impirical */
        if (prev_ground_image->at(j, i).z > 2 && prev_ground_image->at(j, i).z < 9
            &&
            prev_ground_image->at(j, i).x > -4 && prev_ground_image->at(j, i).x < 10)
        {

          //temp map in RCS
          double gridmap_x = prev_ground_image->at(j, i).x; //image coordinate //minus right and plus left
          double gridmap_y = prev_ground_image->at(j, i).z;
          double gridmap_z = (prev_ground_image->at(j, i).y * -1) + 15; //* -1; //for filtering the noise TODO
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y);
          map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x , gridmap_y)) = 4; //mapping_ratio_Z
          map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x , gridmap_z)) = 4; //showing the height of obstacles



          //temp map in RCS -- scaled down
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "the red area for obstacle =====> x: ", prev_ground_image->at(j, i).x, "---- y: ", prev_ground_image->at(j, i).y, "------- z: ", prev_ground_image->at(j, i).z);  //prev_ground_image->at(j, i * cell_height).rgb = 0; //fitting the model on terrain
          gridmap_x = prev_ground_image->at(j, i).x * mapping_ratio_Z * mapping_scale * mapping_ratio_X; //image coordinate //minus right and plus left
          gridmap_y = prev_ground_image->at(j, i).z * mapping_ratio_Z * mapping_scale;
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y);

          //thresholding the points, counting the number of points in each cell and filling it out for the front scanner
          counterNumPointsInEachCell_stereo[map_stereo.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y))] ++;

          //thresholding temp map
          if (counterNumPointsInEachCell_stereo[map_stereo.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y))] > threshNumPointsInCell_stereo) //if more than 2 points in oone cell
          {
            map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x , gridmap_y)) = 4; //mapping_ratio_Z
          }//counter


          // environment map in WCS
          /* transformation applying on the points*/
          absolute_sensor_pose_stereo.Set(transformation_robot2world_mat4x4 * rotation_90_to_right_mat4x4 * rrlib::math::tPose3D(gridmap_x, gridmap_y, 0, 0, 0, 0).GetTransformationMatrix());
          rrlib::math::tVec2f position = absolute_sensor_pose_stereo.Position();
          double gridmap_x_env = position.X();
          double gridmap_y_env = position.Y();

          //thresholding the points, counting the number of points in each cell and filling it out for the front scanner
          counterNumPointsInEachCell_stereo_env[map_stereo_env.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env))] ++;

          //thresholding
          if (counterNumPointsInEachCell_stereo_env[map_stereo_env.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env))] > threshNumPointsInCell_stereo
              &&
              counterNumPointsInEachCell_stereo[map_stereo.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y))] > threshNumPointsInCell_stereo) //if more than 2 points in oone cell
          {
            map_stereo_env.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env)) = 4; //red or obnstacle
          }//counter

        }// if filtering
      }// if

      /*
       * area yellow and green
       */
      if ((prev_ground_image->at(j, i).r == 128 && prev_ground_image->at(j, i).g == 128) //yellow
          ||
          prev_ground_image->at(j, i).g == 255)  //green
      {
        /*
         * thresholding and filtering the outliers
         */
        /*! using the bounding box impirical */
        if (prev_ground_image->at(j, i).x < 5 && prev_ground_image->at(j, i).x > -2 && prev_ground_image->at(j, i).z > 2 && prev_ground_image->at(j, i).z < 5)
        {
          // real one in map for RCS
          double gridmap_x = prev_ground_image->at(j, i).x;
          double gridmap_y = prev_ground_image->at(j, i).z;
//          map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 2;  //mapping_ratio_Z

          if (map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) != 4) //if already marked as red or obstacle
          {
            map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 2; //temporary //red for obstacle
          }// if

          //scaled one for WCS
          gridmap_x = prev_ground_image->at(j, i).x * mapping_ratio_Z * mapping_scale * mapping_ratio_X;
          gridmap_y = prev_ground_image->at(j, i).z * mapping_ratio_Z * mapping_scale;
          map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 2;  //mapping_ratio_Z

          /* transformation applying on the points for mapping the environment */
          absolute_sensor_pose_stereo.Set(transformation_robot2world_mat4x4 * rotation_90_to_right_mat4x4 * rrlib::math::tPose3D(gridmap_x, gridmap_y, 0, 0, 0, 0).GetTransformationMatrix());
          rrlib::math::tVec2f position = absolute_sensor_pose_stereo.Position();
          double gridmap_x_env = position.X();
          double gridmap_y_env = position.Y();

          if (map_stereo_env.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env)) != 4) //if already marked as red or obstacle
          {
            map_stereo_env.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env)) = 2; //temporary //red for obstacle
          }// if


        }// if thresh and filtering the outliers
      }// if yellow

    }//for j
  } // for i

  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "width: ", prev_ground_image->width, "----- height: ", prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ALL IMAGE PIXELS: ", prev_ground_image->width * prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "NUMBER OF CELLS: ", map_stereo.GetNumberOfCells());

  /*---------------> 4- pose of robot and environment <----------------------*/
  map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(0, 0)) = 1; //origin and location of stereo camera -- left one
  map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(0, -0.1)) = 1; //origin and location of stereo camera -- left one
  map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(0, -0.2)) = 1; //origin and location of stereo camera -- left one
  map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(0, -0.3)) = 1; //origin and location of stereo camera -- left one
  map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(0, -0.4)) = 1; //origin and location of stereo camera -- left one


  /*--------------------- drawing robot on environment map canvas ----------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas_map_env_stereo_pointer = si_canvas_map_env_stereo.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas_map_env_stereo_pointer = so_canvas_map_env_stereo.GetUnusedBuffer();
  rrlib::canvas::tCanvas2D *out_canvas_map_env_stereo = output_canvas_map_env_stereo_pointer.Get();
  memcpy(out_canvas_map_env_stereo, input_canvas_map_env_stereo_pointer, sizeof(*input_canvas_map_env_stereo_pointer));
  out_canvas_map_env_stereo->DrawArrow(pose_rear_scanner.X() * mapping_scale, pose_rear_scanner.Y() * mapping_scale, pose_front_scanner.X() * mapping_scale,
                                       pose_front_scanner.Y() * mapping_scale, false);
  so_canvas_map_env_stereo.Publish(output_canvas_map_env_stereo_pointer);


  //publishing the port
  so_map_grid_stereo.Publish(map_pointer_stereo);
  so_map_grid_stereo_env.Publish(map_stereo_env);

}// gridmap_stereo

//----------------------------------------------------------------------
// mStViPr constructor
//----------------------------------------------------------------------
mStViPr::mStViPr(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name)
  , input_iplimage_left(NULL),  //tImage2cvMat
  input_iplimage_right(NULL), //tImage2cvMat
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer filtered")),
  image_viewer_original(new pcl::visualization::ImageViewer("Image Viewer original terrain classification")),
  image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
  image_viewer_disparity_2(new visualization::ImageViewer("ImageViewer empty-debugging!!!")),  //debugging only
  prev_cloud(new Cloud),
  prev_normal_cloud(new pcl::PointCloud<pcl::Normal>),
  prev_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  prev_ground_image(new Cloud),
  prev_label_image(new Cloud),
  road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>)
  , road_segmentation(road_comparator)
  , writer_()
  , file_name_("frames_pcd")
  , dir_name_("frames_pcd")
  , format_(4)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{

  fprintf(stderr, "mStViPr constructor started.....................................................................................\n");  //::ctor >>

  input_iplimage_left = cvCreateImageHeader(cvSize(0, 0), IPL_DEPTH_32F, 3);
  input_iplimage_right = cvCreateImageHeader(cvSize(0, 0), IPL_DEPTH_32F, 3);

  //stereo vision processing
  trigger = true;
  continuous = true;
  display_normals = false;
  detect_obstacles = true;  //enabled

  //left and right images
  this->left_images = left_images;
  this->right_images = right_images;
  images_idx = 0; //images_idx ++;
  images_idx_saveImage = 0;
  n_r = 1;
  n_l = 1;
  this->img_pairs_num = img_pairs_num;
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  // Set up a 3D viewer
  viewer->setBackgroundColor(0, 0, 0); //    viewer->addCoordinateSystem(1.0); //default
  double scale = 2.0;
  float x = 0.0; //0
  float y = 0.5; //0.5
  float z = 1.0 ; //1
  int viewport = 0;

  // Adds 3D axes describing a coordinate system to screen at x, y, z.
  viewer->addCoordinateSystem(scale, x, y, z, viewport);
  viewer->initCameraParameters();
  viewer->registerKeyboardCallback(&mStViPr::keyboardCallback, *this, 0);

  // AdaptiveCostSOStereoMatching
  stereo.setMaxDisparity(60);
  stereo.setXOffset(0);
  stereo.setRadius(5);
  stereo.setSmoothWeak(25);
  stereo.setSmoothStrong(100);
  stereo.setGammaC(25);
  stereo.setGammaS(5);
  stereo.setPreProcessing(true);

//  // Set up the normal estimation
//  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
//  ne.setMaxDepthChangeFactor(0.03f);
//  ne.setNormalSmoothingSize(40.0f); //20.0f
//
//  // Set up the ground plane comparator  // If the camera was pointing straight out, the normal would be:
//  Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
//
//  // Adjust for camera tilt:
//  Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//  road_comparator->setExpectedGroundNormal(tilt_road_normal);
//  road_comparator->setGroundAngularThreshold(pcl::deg2rad(20.0f)); //10.0f original
//  road_comparator->setAngularThreshold(pcl::deg2rad(10.0f)); //3.0f original

  /*---------------mapping stereo -------------------*/
  /*--------------------- environment mapping ----------------*/
  bounds_stereo_env.upper_bounds[0] = rrlib::math::tVec2d(bound_limit, 0);
  bounds_stereo_env.lower_bounds[0] = rrlib::math::tVec2d(-bound_limit, 0);
  bounds_stereo_env.upper_bounds[1] = rrlib::math::tVec2d(0, bound_limit);
  bounds_stereo_env.lower_bounds[1] = rrlib::math::tVec2d(0, -bound_limit);
  map_stereo_env.SetBounds(bounds_stereo_env);
  map_stereo_env.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); // resolution: 1 meter, 1 meter
//  if (ci_reset_grdimap_stereo_env.Get() == 1){
//    map_stereo_env.Clear(0);
//  }

  fprintf(stderr, "mStViPr constructor finished.....................................................................................\n");  //::ctor >>

}// constructor mStViPr

//----------------------------------------------------------------------
// mStViPr Destructor /* --------------------------private------------------------*/
//----------------------------------------------------------------------
mStViPr::~mStViPr() {}

//----------------------------------------------------------------------
// mStViPr EvaluateStaticParameters
//----------------------------------------------------------------------
void mStViPr::OnStaticParameterChange()
{}

//----------------------------------------------------------------------
// mStViPr EvaluateParameters
//----------------------------------------------------------------------
void mStViPr::OnParameterChange()
{}

//----------------------------------------------------------------------
// mStViPr Sense
//----------------------------------------------------------------------
void mStViPr::Sense()  //sensing the images
{

  // Grabing images from tFrameGrabber
  data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> input_images = si_image.GetPointer();

  // when enabled
  if (input_images->size() > 0)
  {
    rrlib::coviroa::AccessImageAsIplImage(input_images-> at(0), *input_iplimage_left);
    rrlib::coviroa::AccessImageAsIplImage(input_images-> at(1), *input_iplimage_right);

    // tImage2IplImage --> IplImage2Mat
    Mat img1(input_iplimage_left);
    Mat img2(input_iplimage_right);
    img1.copyTo(left_rect_png_img);
    img2.copyTo(right_rect_png_img);

    /* ----terrain classification---------*/
    run();

    /*------mapping-----------*/
    gridmap_stereo();

  }//if

}//mStViPr sense

//----------------------------------------------------------------------
// mStViPr Control
//----------------------------------------------------------------------
void mStViPr::Control()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
} //simulation
} //icarus
} //finroc
