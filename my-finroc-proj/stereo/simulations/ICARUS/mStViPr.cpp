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
#include "projects/icarus/simulation/mStViPr.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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
namespace simulation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStViPr> cCREATE_ACTION_FOR_M_ST_VI_PR("StViPr");

const int scanner_radius_max_threshold = 19;
const int bound_limit = 100; //40; //80 //40 //20
const double cell_resolution = 0.2; //0.1; //0.5 //0.1; //0.1 //1
const double mapping_scale = 1; //0.1; //1; //0.1;
/*! scaling */
const double stereo_max_range = 8; //7.9 - 8.1
const double scanner_max_range = 20; //19; //21;
const double mapping_ratio_Z = scanner_max_range / stereo_max_range;
const double mapping_ratio_X = 0.3 / 3;//1; //0.3 / 12;  //0.3 / 6;  //3 cells is 2.2 X in image //0.3 / 2.2;  //3 cells is 2.2 X in image
const int threshNumPointsInCell_stereo = 3;
#define PI 3.14159265


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
    case '1':
      smooth_strong -= 10;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '2':
      smooth_strong += 10;
      PCL_INFO("smooth_strong: %d\n", smooth_strong);
      stereo.setSmoothStrong(smooth_strong);
      break;
    case '3':
      smooth_weak -= 10;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    case '4':
      smooth_weak += 10;
      PCL_INFO("smooth_weak: %d\n", smooth_weak);
      stereo.setSmoothWeak(smooth_weak);
      break;
    case 'n':
      display_normals = !display_normals;
      break;
    }
  }
}

void
mStViPr::stereo_rectify(const Mat input_images_left, const Mat input_images_right)
{

  Mat img[2];
  input_images_left.copyTo(img[0]);
  input_images_right.copyTo(img[1]);

  Size imageSize = img[0].size();
  Mat canvas;
  double sf;
  int w, h;
  // OpenCV can handle left-right or up-down camera arrangements
  bool isVerticalStereo = false; //fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
  if (!isVerticalStereo)
  {
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);
  }


  //Mat img_rect[2];
  for (int k = 0; k < 2; k++)
  {
    //cvtColor(rimg, cimg, CV_GRAY2BGR);
    Mat canvasPart = canvas(Rect(w * k, 0, w, h)) ;
    resize(img[k], canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
  }


  if (!isVerticalStereo)
    for (int j = 0; j < canvas.rows; j += 16)
      line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

  imshow("rectified_true", canvas);


  //img_rect[0].copyTo(left_img_rect);
  //img_rect[1].copyTo(right_img_rect);
  img[0].copyTo(left_img_rect);
  img[1].copyTo(right_img_rect);
  namedWindow("left_rect", 0);
  imshow("left_rect", left_img_rect);
  namedWindow("right_rect", 0);
  imshow("right_rect", right_img_rect);

}// stereo_rectify_test

PointCloud<RGB>::Ptr
mStViPr::img2pcd(const Mat image_rect)
{

  Mat image;
  image_rect.copyTo(image);

  double pixel;
  PointCloud<RGB> cloud;

  // cloud initialization
  cloud.width = image.cols;
  cloud.height = image.rows; // This indicates that the point cloud is organized
  cloud.is_dense = true;
  cloud.points.resize(cloud.width * cloud.height);

  // sky removal
  vector<int> valid_indices;

  // png to pcd loop
  for (int y = 0; y < image.rows; y++)
  {
    for (int x = 0; x < image.cols; x++)
    {
      pixel = (double) image.at<Vec3b>(y, x)[0];

      RGB color;
      color.r = 0;
      color.g = 0;
      color.b = 0;
      color.a = 0;
      color.rgb = 0.0f;
      color.rgba = 0;

      //case 1: component ==1
      color.r = static_cast<uint8_t>(pixel);
      color.g = static_cast<uint8_t>(pixel);
      color.b = static_cast<uint8_t>(pixel);

      int rgb;
      rgb = (static_cast<int>(color.r)) << 16 |
            (static_cast<int>(color.g)) << 8 |
            (static_cast<int>(color.b));

      //color.rgb = static_cast<double>(pixel_test);
      color.rgb = static_cast<float>(rgb);
      color.rgba = static_cast<uint32_t>(rgb);



      // Point<RGB> (x,y) = RGB
      //cloud (x, image.rows - y - 1) = color;
      cloud(x, y) = color;

      valid_indices.push_back(y + x);
    } //for x
  }// for y


  // output pcd
  PointCloud<RGB>::Ptr cloud_output(new PointCloud<RGB>);
  *cloud_output = cloud;

  return cloud_output;
}// img2pcd

void
mStViPr::processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
{

  stereo.compute(*left_image, *right_image);
  stereo.medianFilter(10);  // better but slower //optimal

  // stereo images camera calibration parameters  --> simvis3d --> 1st --> the best
  float u_c = 130.00152015686035f; //1.3000152015686035e+02; //old sample -->379.85181427001953f; // the main one 3.7985181427001953e+02; // pcl //opencv - Cx = 3.8749671936035156e+02;
  float v_c = 118.91034603118896f; //1.1891034603118896e+02 //old sample --> 305.85922241210938f; // pcl //opencv - Cy = 3.1611055755615234e+02;
  float focal = 163.33793783988659f;  //1.6333793783988659e+02  --> old sample ==>//920.38355542932538f; //9.7814703939910510e+02;  //f
  //float baseline = 0.291699576f; //47.645607158602786÷163.33793783988659 //47.645607158602786/163.33793783988659; //-4.7645607158602786e+01; old sample -->//0.359294689f; //real one using calculator
  float baseline = 0.18f; //47.645607158602786÷163.33793783988659 //47.645607158602786/163.33793783988659; //-4.7645607158602786e+01; old sample -->//0.359294689f; //real one using calculator
  CloudPtr cloud = out_cloud;
  pcl::PointCloud<pcl::RGB>::Ptr texture = left_image;
  stereo.getPointCloud(u_c, v_c, focal, baseline, cloud, texture);
}// processPair

void
mStViPr::processCloud(const CloudConstPtr& cloud)
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
  /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
  /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
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
  /*checking the number of point in the cloud for verification*/
  /*checking the number of point in the cloud for verification*/
  /*checking the number of point in the cloud for verification*/
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

  /*filtering*/
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(dominant_plane_y_min, dominant_plane_y_max);
  CloudPtr cloud_filtered(new Cloud);
  pass.filter(*cloud_filtered);
  cout << "size: " << cloud_filtered->points.size() << endl;
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
    //viewer->addText3D(ss.str(), position, textScale, r, g, b,  "cloud");
    //viewer->addText(ss.str(), 400, 300, textScale, r, g, b, "cloud");
    //viewer_test->addText3D(ss.str(), position, textScale, r, g, b, "cloud_test");

    unsigned int x_min = 0, y_min = 0, x_max = cloud->width, y_max = cloud->height; //image.width=800, image.height=600
    double opacity = 1.0;
    image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    image_viewer_original->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    x_min = cloud->width;
    y_min = 0;
    x_max = 0;
    y_max = cloud->height;
    image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    image_viewer_original->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
  }



  /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
  if (ground_cloud->points.size() > 0)
  {
    ground_centroid = centroids[0];
    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
  }// if


//  /*! other traversable regions in dominant plane limits - green*/
//  if (frame_isGood)
//  {
//    /*! other traversable regions in dominant plane limits - green*/
//    for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
//    {
//
//      /*looking for other the largest region*/
////      if (region_indices[i].indices.size() > threshold_min_dominant_plane_cloud &&
////          region_indices[i].indices.size() <= largest_1 &&
////          region_indices[i].indices.size() > largest_1 / 2) //100 original //min_inliers - green -- for each segment 1000 original
//      if (region_indices[i].indices.size() > 100)
//      {
//
//            /*visualizing the result by colorizing the point cloud */ /*! traversable regions - green*/
//            for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
//            {
//              ground_image->points[region_indices[i].indices[j]].r = 0;
//              ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
//              ground_image->points[region_indices[i].indices[j]].b = 0;
//              label_image->points[region_indices[i].indices[j]].r = 0;
//              label_image->points[region_indices[i].indices[j]].g = 255 ;
//              label_image->points[region_indices[i].indices[j]].b = 0;
//              pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
//                                      cloud->points[region_indices[i].indices[j]].y,
//                                      cloud->points[region_indices[i].indices[j]].z);
//              ground_cloud->points.push_back(ground_pt);
//
//            }// for
//
////        /*! Compute plane info*/
////        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
////        Eigen::Matrix3f clust_cov;
////        pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);
////
////        //cout << "cluster_centroid: " << clust_centroid << endl;
////        cout << "cluster_centroid_y: " << clust_centroid[1] << endl;
////
////
////        pcl::PointXYZ centroid_pt(clust_centroid[0], clust_centroid[1], clust_centroid[2]);
////        //double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
////        double ptp_dist =  pcl::pointToPlaneDistance(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
////
////        //if (plane_params[3] < dominant_plane_y_max && plane_params[3] > dominant_plane_y_min)
////        if (clust_centroid[1] <= dominant_plane_y_max && clust_centroid[1] >= dominant_plane_y_min && ptp_dist < 1)
////        {
////
////          cout << "cluster_centroid meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
////
////          /*visualizing the result by colorizing the point cloud */ /*! traversable regions - green*/
////          for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
////          {
////            ground_image->points[region_indices[i].indices[j]].r = 0;
////            ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
////            ground_image->points[region_indices[i].indices[j]].b = 0;
////            label_image->points[region_indices[i].indices[j]].r = 0;
////            label_image->points[region_indices[i].indices[j]].g = 255 ;
////            label_image->points[region_indices[i].indices[j]].b = 0;
////            pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
////                                    cloud->points[region_indices[i].indices[j]].y,
////                                    cloud->points[region_indices[i].indices[j]].z);
////            ground_cloud->points.push_back(ground_pt);
////
////          }// for
////        }//if
////        else
////        {
////          cout << "cluster_centroid NOT meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
////        }//else
//
//
//      } //if region
//    }// for i
//  }// frame


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
    unsigned min_inliers = 500; //500 first time //this shows how stable and reliable a plane is //max w and height of the image
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

    //Note the regions that have been extended
    for (unsigned int i = 0; i < region_indices.size(); i++)
    {
      if (region_indices[i].indices.size() >= min_inliers)
      {
        for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
        {
          if (label_image->points[region_indices[i].indices[j]].g != 255)  //region_indices_test[i].indices.size() >= 1000
          {
            //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
            //ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].r = 255; //static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].r + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].g = 255; //static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].b = 0;
            label_image->points[region_indices[i].indices[j]].r = 255;
            label_image->points[region_indices[i].indices[j]].g = 255;
            label_image->points[region_indices[i].indices[j]].b = 0;
          }//if
        }//for
      }//if
    }// for

  }//if for plane refinement



  /*Segment Obstacles using OrganizedConnectedComponentSegmentation and EuclideanClusterComparator*/
  if (frame_isGood && ground_cloud->points.size())
  {
    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_compare(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
    euclidean_cluster_compare->setInputCloud(cloud);
    euclidean_cluster_compare->setInputNormals(normal_cloud);

    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    euclidean_cluster_compare->setDistanceThreshold(distance_threshold, depth_dependent);

    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
    euclidean_cluster_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle capability

    boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
    *labels_ptr = labels;
    euclidean_cluster_compare->setLabels(labels_ptr);

    std::vector<bool> plane_labels;
    plane_labels.resize(region_indices.size(), false);
    unsigned min_inliers_mps = 500;
    for (size_t i = 0; i < region_indices.size(); i++)
    {
      if (region_indices[i].indices.size() > min_inliers_mps)
      {
        plane_labels[i] = true;
      }//if
    }// for
    euclidean_cluster_compare->setExcludeLabels(plane_labels);


    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(euclidean_cluster_compare);
    organized_cc_segmentation.setInputCloud(cloud);
    pcl::PointCloud<pcl::Label> labels_test;
    std::vector<pcl::PointIndices> inlier_indices_test;
    organized_cc_segmentation.segment(labels_test, inlier_indices_test);

    vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices

    /*colorizing the point cloud*/
    for (unsigned int i = 0; i < region_indices_test.size(); i++)
    {
      if (region_indices_test[i].indices.size() >= 500)
      {

        /*! Compute plane info*/
        Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
        Eigen::Matrix3f cluster_covariance;
        pcl::computeMeanAndCovarianceMatrix(*ground_image, region_indices_test[i].indices, cluster_covariance, cluster_centroid);

        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
        Eigen::Vector3f plane_params;
        plane_params[0] = eigen_vector[0];
        plane_params[1] = eigen_vector[1];
        plane_params[2] = eigen_vector[2];

        //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
        Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
        Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
        float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
        float thresold_obstacle = cos(50.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces

        pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
        double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

        for (unsigned int j = 0; j < region_indices_test[i].indices.size(); j++)
        {

          if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
          {
            //ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
            ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
            ground_image->points[region_indices_test[i].indices[j]].g = 0;
            ground_image->points[region_indices_test[i].indices[j]].b = 0;
            label_image->points[region_indices_test[i].indices[j]].r = 255;
            label_image->points[region_indices_test[i].indices[j]].g = 0;
            label_image->points[region_indices_test[i].indices[j]].b = 0;
          }// if ptp
        }//for j indices
      }// if for min inliers to 1000
    }// for i the mainn one for regions

  }// if obstacle for euclidean + cc


  // note the NAN points in the image as well
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if (!pcl::isFinite(cloud->points[i]))
    {
      ground_image->points[i].r = 0;
      ground_image->points[i].g = 0;
      ground_image->points[i].b = 255; //static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
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
}

void mStViPr::run()
{
  // tImage2IplImage --> IplImage2Mat
  Mat img1(input_iplimage_left);
  Mat img2(input_iplimage_right);

  stereo_rectify(img1, img2);
  //waitKey(1);

  pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
  pcl::PointCloud<pcl::RGB>::Ptr right_cloud(new pcl::PointCloud<pcl::RGB>);
  CloudPtr out_cloud(new Cloud);

  /*png2pcd -- mat img input*/
  left_cloud = img2pcd(left_img_rect);
  right_cloud = img2pcd(right_img_rect);

  /*process the disparity map and point cloud*/
  processStereoPair(left_cloud, right_cloud, out_cloud);

  ///displaying disparity map
  pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
  stereo.getVisualMap(vmap);
  image_viewer_disparity->addRGBImage<pcl::RGB> (vmap);
  image_viewer_disparity->spinOnce(1);


  if (out_cloud->points.size() > 1000)
  {
    image_viewer->removeLayer("line");  //const std::string &layer_id
    image_viewer_original->removeLayer("line");  //const std::string &layer_id
    processCloud(out_cloud);

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
      else if (!display_normals)
      {
        viewer->removePointCloud("normals");
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
  }// if any cloud

}// run

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

//  /*measring and drawing the vehicle */
//  rrlib::math::tPose3D fl = distance_data->SensorPose();
//  rrlib::math::tPose3D absolute_corner_fl(distance_data->RobotPose());
//  absolute_corner_fl.ApplyRelativePoseTransformation(rrlib::math::tPose3D(fl.X(), fl.Y()+24 , fl.Z())); //, fl.Roll(), fl.Pitch(), fl.Yaw()
//  absolute_corner_fl.Scale(0.001); // for displaying the minimum distance sample we need unit meter
//  rrlib::math::tVec3d pose_front_corner_left = absolute_corner_fl.Position();
//
//
////  rrlib::math::tPose3D fl = distance_data->SensorPose();
//  rrlib::math::tPose3D absolute_corner_rl(distance_data->RobotPose());
//  absolute_corner_rl.ApplyRelativePoseTransformation(rrlib::math::tPose3D(fl.X()-3.3, fl.Y()+24 , fl.Z())); //, fl.Roll(), fl.Pitch(), fl.Yaw()
//  absolute_corner_rl.Scale(0.001); // for displaying the minimum distance sample we need unit meter
//  rrlib::math::tVec3d pose_rear_corner_left = absolute_corner_rl.Position();
//
//  //  rrlib::math::tPose3D absolute_corner_fl(rrlib::math::tPose3D(fl.X(), fl.Y()+1 , fl.Z()));
//  //  rrlib::math::tPose3D absolute_corner_fr(rrlib::math::tPose3D(fl.X(), fl.Y()-1 , fl.Z()));
//  //  rrlib::math::tPose3D absolute_corner_rl(rrlib::math::tPose3D(fl.X()-3.3, fl.Y()+1 , fl.Z()));
//  //  rrlib::math::tPose3D absolute_corner_rr(rrlib::math::tPose3D(fl.X()-3.3, fl.Y()-1 , fl.Z()));


  //saving the transformation matrix
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "front scanner sensor pose in WCS: ", absolute_sensor_pose.Position());
  rrlib::math::tVec3f pose_front_scanner = absolute_sensor_pose.Position();

  /*---------------> rear laser scanner <----------------------*/
  // reading the port
  data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> distance_data_rear_scanner = si_distance_data_rear_scanner.GetPointer();

  // access robot pose from source ... absolute_sensor_pose
//  rrlib::math::tPose3D absolute_sensor_pose_rear_scanner(distance_data_rear_scanner->RobotPose());
  rrlib::math::tPose3D absolute_sensor_pose_rear_scanner(distance_data->RobotPose()); //test

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


  //sector mapping stuff
  vector<double>* cartesian_flc_sectors_distance = new vector<double> [5];
  vector<double>* cartesian_frc_sectors_distance = new vector<double> [5];

  vector<double>* polar_frp_sectors_distance = new vector<double> [10];
  vector<double>* polar_flp_sectors_distance = new vector<double> [10];



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
      if (prev_label_image->at(j, i).r == 255 && prev_label_image->at(j, i).g == 0 && prev_label_image->at(j, i).b == 0) //red obstacle
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
          //double gridmap_z = (prev_ground_image->at(j, i).y * -1) + 15; //* -1; //for filtering the noise TODO
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x);
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y);
          map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x , gridmap_y)) = 3;

          /* filling the sectomaps for cartesian */
          for (int i = 0; i < 5 ; i++)
          {
            if (gridmap_x > (-0.2 * (i + 1)) && gridmap_x < (-0.2 * i))
            {
              cartesian_flc_sectors_distance[i].push_back(gridmap_y);
            }

            if (gridmap_x < (0.2 * (i + 1)) && gridmap_x > (0.2 * i))
            {
              cartesian_frc_sectors_distance[i].push_back(gridmap_y);
            }
          }

          /*
           * polar sectors filling
           */

          double gridmap_x_trans = gridmap_x - 1.0;
          double gridmap_y_trans = gridmap_y;
          double param = (gridmap_y_trans) / (gridmap_x_trans);
          double result;
          result = atan(param) * 180 / PI;  //degree

          double gridmap_x_trans_left = gridmap_x + 1.0;
          double gridmap_y_trans_left = gridmap_y;
          double param_left = (gridmap_y_trans_left) / (gridmap_x_trans_left);
          double result_left;
          result_left = atan(param_left) * 180 / PI;  //degree

          for (int i = 0; i < 10; i++)
          {
            if (gridmap_x_trans >= 0)
            {
              if (result > (i * 9) && result < ((i + 1) * 9))
              {
                double length = sqrt((gridmap_x_trans * gridmap_x_trans) + (gridmap_y_trans * gridmap_y_trans));
                polar_frp_sectors_distance[i].push_back(abs(length));
                FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x_trans: ", gridmap_x_trans, " gridmap_y_trans: " , gridmap_y_trans);
              }
            }

            if (gridmap_x_trans_left <= 0)
            {
              if (result_left > ((i * 9) + 90) && result_left < (((i + 1) * 9) + 90))
              {
                double length = sqrt((gridmap_x_trans_left * gridmap_x_trans_left) + (gridmap_y_trans_left * gridmap_y_trans_left));
                polar_flp_sectors_distance[i].push_back(length);
                FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x_trans: ", gridmap_x_trans_left, " gridmap_y_trans: " , gridmap_y_trans_left);
              }
            }

          }


          //thresholding the points, counting the number of points in each cell and filling it out for the front scanner
          counterNumPointsInEachCell_stereo[map_stereo.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y))] ++;

//          //thresholding temp map
//          if (counterNumPointsInEachCell_stereo[map_stereo.GetCellIDByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y))] > threshNumPointsInCell_stereo) //if more than 2 points in oone cell
//          {
//            map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x , gridmap_y)) = 3; //mapping_ratio_Z
//          }//counter


          //temp map in RCS -- scaled down
          FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "the red area for obstacle =====> x: ", prev_ground_image->at(j, i).x, "---- y: ", prev_ground_image->at(j, i).y, "------- z: ", prev_ground_image->at(j, i).z);  //prev_ground_image->at(j, i * cell_height).rgb = 0; //fitting the model on terrain
          double gridmap_x_scaled = prev_ground_image->at(j, i).x * mapping_ratio_Z * mapping_scale * mapping_ratio_X; //image coordinate //minus right and plus left
          double gridmap_y_scaled = prev_ground_image->at(j, i).z * mapping_ratio_Z * mapping_scale;
          //FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x);
          //FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y);

          // environment map in WCS
          /* transformation applying on the points*/
          absolute_sensor_pose_stereo.Set(transformation_robot2world_mat4x4 * rotation_90_to_right_mat4x4 * rrlib::math::tPose3D(gridmap_x_scaled, gridmap_y_scaled, 0, 0, 0, 0).GetTransformationMatrix());
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
            map_stereo_env.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env)) = 3; //red or obnstacle
          }//counter



        }// if filtering
      }// if red or obstacle

      /*
       * area green
       */
      //(prev_label_image->at(j, i).r == 255 && prev_label_image->at(j, i).g == 255) //yellow
      if (prev_label_image->at(j, i).r == 0 && prev_label_image->at(j, i).g == 255 && prev_label_image->at(j, i).b == 0)  //green
      {

        // real one in map for RCS
        double gridmap_x = prev_ground_image->at(j, i).x;
        double gridmap_y = prev_ground_image->at(j, i).z;

        if (map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) != 3) //if already marked as red or obstacle
        {
          //map_stereo.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 1; //temporary //red for obstacle
        }// if


        /*
         * thresholding and filtering the outliers
         */
        /*! using the bounding box impirical */
        if (prev_ground_image->at(j, i).x < 5 && prev_ground_image->at(j, i).x > -2
            &&
            prev_ground_image->at(j, i).z > 2 && prev_ground_image->at(j, i).z < 5)
        {

          //scaled one for WCS
          double gridmap_x_scaled = prev_ground_image->at(j, i).x * mapping_ratio_Z * mapping_scale * mapping_ratio_X;
          double gridmap_y_scaled = prev_ground_image->at(j, i).z * mapping_ratio_Z * mapping_scale;

          /* transformation applying on the points for mapping the environment */
          absolute_sensor_pose_stereo.Set(transformation_robot2world_mat4x4 * rotation_90_to_right_mat4x4 * rrlib::math::tPose3D(gridmap_x_scaled, gridmap_y_scaled, 0, 0, 0, 0).GetTransformationMatrix());
          rrlib::math::tVec2f position = absolute_sensor_pose_stereo.Position();
          double gridmap_x_env = position.X();
          double gridmap_y_env = position.Y();

          if (map_stereo_env.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env)) != 3) //if already not marked as red or obstacle
          {
            map_stereo_env.GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x_env, gridmap_y_env)) = 1; //temporary //red for obstacle
          }// if

        }// if thresh and filtering the outliers

      }// if green

    }//for j
  } // for i

  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "width: ", prev_ground_image->width, "----- height: ", prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ALL IMAGE PIXELS: ", prev_ground_image->width * prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "NUMBER OF CELLS: ", map_stereo.GetNumberOfCells());

  /*--------------------- drawing robot on environment map canvas ----------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas_map_env_stereo_pointer = si_canvas_map_env_stereo.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas_map_env_stereo_pointer = so_canvas_map_env_stereo.GetUnusedBuffer();
  rrlib::canvas::tCanvas2D *out_canvas_map_env_stereo = output_canvas_map_env_stereo_pointer.Get();
  memcpy(out_canvas_map_env_stereo, input_canvas_map_env_stereo_pointer, sizeof(*input_canvas_map_env_stereo_pointer));
  out_canvas_map_env_stereo->DrawArrow(pose_rear_scanner.X() * mapping_scale, pose_rear_scanner.Y() * mapping_scale, pose_front_scanner.X() * mapping_scale,
                                       pose_front_scanner.Y() * mapping_scale, false);



  /*measring and drawing the vehicle */

  //  out_canvas_map_env_stereo->DrawArrow(pose_rear_corner_left.X() * mapping_scale, pose_rear_corner_left.Y() * mapping_scale, pose_front_corner_left.X() * mapping_scale,
  //                                      pose_front_corner_left.Y() * mapping_scale, false);

  //  out_canvas_map_env_stereo->DrawPoint(pose_rear_corner_left.X(), pose_rear_corner_left.Y());
  //  out_canvas_map_env_stereo->DrawPoint(pose_front_corner_left.X(), pose_front_corner_left.Y());
  //  //  void DrawBox(T bottom_left_x, T bottom_left_y, T width, T height = -1);
  //  out_canvas_map_env_stereo->DrawBox(pose_rear_corner_left.X(), pose_rear_corner_left.Y(), 3.3, -1);


  //temp map
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas_map_temp_stereo_pointer = si_canvas_map_temp_stereo.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas_map_temp_stereo_pointer = so_canvas_map_temp_stereo.GetUnusedBuffer();
  rrlib::canvas::tCanvas2D *out_canvas_map_temp_stereo = output_canvas_map_temp_stereo_pointer.Get();
  memcpy(out_canvas_map_temp_stereo, input_canvas_map_temp_stereo_pointer, sizeof(*input_canvas_map_temp_stereo_pointer));


  //sector mapping stuff
  double* dist  = new double[5];
  double* dist_frc =  new double[5];

  //std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].be, cartesian_flc_sectors_distance[0] + cartesian_flc_sectors_distance[0].size()) << '\n';
  for (int i = 0; i < 5; i++)
  {
    if (cartesian_flc_sectors_distance[i].size() > 0)
    {
      //std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].begin(), cartesian_flc_sectors_distance[0].end()) << '\n';
      //dist[i] = *std::max_element(cartesian_flc_sectors_distance[i].begin(), cartesian_flc_sectors_distance[i].end()); //green
      dist[i] = *std::min_element(cartesian_flc_sectors_distance[i].begin(), cartesian_flc_sectors_distance[i].end()); //red
      //std::sort(cartesian_flc_sectors_distance[i].begin(), cartesian_flc_sectors_distance[i].end());  //from small to big descending order
      //dist[i] = cartesian_flc_sectors_distance[i].at(0); //region_indices_size.size() - 1 largest
      out_canvas_map_temp_stereo->DrawArrow((-0.2 * (i + 1)) + 0.1 , 0.0, (-0.2 * (i + 1)) + 0.1, dist[i]    , false);
      dist_sectors[i] = dist[i];
    }//if
    else
    {
      dist_sectors[i] = 10;
    }

    if (cartesian_frc_sectors_distance[i].size() > 0)
    {
      //std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].begin(), cartesian_flc_sectors_distance[0].end()) << '\n';
      //dist_frc[i] = *std::max_element(cartesian_frc_sectors_distance[i].begin(), cartesian_frc_sectors_distance[i].end()); //grean
      dist_frc[i] = *std::min_element(cartesian_frc_sectors_distance[i].begin(), cartesian_frc_sectors_distance[i].end()); //red
      //std::sort(cartesian_frc_sectors_distance[i].begin(), cartesian_frc_sectors_distance[i].end());  //from small to big descending order
      //dist_frc[i] = cartesian_frc_sectors_distance[i].at(0); //region_indices_size.size() - 1 largest
      out_canvas_map_temp_stereo->DrawArrow((0.2 * i) + 0.1 , 0.0, (0.2 * i) + 0.1   , dist_frc[i], false);
      dist_frc_sectors[i] = dist_frc[i];
    }//if
    else
    {
      dist_frc_sectors[i] = 10;
    }
  }


  delete [] dist;
  delete [] dist_frc;
  delete [] cartesian_flc_sectors_distance;
  delete [] cartesian_frc_sectors_distance;

  //sector mapping stuff
  double* dist_polar  = new double[10];
  double* dist_frc_polar =  new double[10];

  //std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].be, cartesian_flc_sectors_distance[0] + cartesian_flc_sectors_distance[0].size()) << '\n';
  for (unsigned int i = 0; i < 10; i++)
  {
    if (polar_flp_sectors_distance[i].size() > 0)
    {
      dist_polar[i] = *std::min_element(polar_flp_sectors_distance[i].begin(), polar_flp_sectors_distance[i].end()); //red
      out_canvas_map_temp_stereo->DrawArrow(-1.0, 0.0, (dist_polar[i] * cos((90 + ((i + 1) * 9))* PI / 180.0)) - 1, dist_polar[i] * sin((90 + ((i + 1) * 9))* PI / 180.0) , false);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "FLP ----> dist_polar[i]: ", dist_polar[i], " cos( (90+(i * 9))* PI / 180.0): ", cos((90 + (i * 9))* PI / 180.0), " sin( (90+(i * 9))* PI / 180.0): ", sin((90 + (i * 9))* PI / 180.0));
      dist_polar_sectors[i] = dist_polar[i];
    }//if
    else
    {
      dist_polar_sectors[i] = 10;
    }

    if (polar_frp_sectors_distance[i].size() > 0)
    {
      //std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].begin(), cartesian_flc_sectors_distance[0].end()) << '\n';
      //dist_frc[i] = *std::max_element(cartesian_frc_sectors_distance[i].begin(), cartesian_frc_sectors_distance[i].end()); //grean
      dist_frc_polar[i] = *std::min_element(polar_frp_sectors_distance[i].begin(), polar_frp_sectors_distance[i].end()); //red
      out_canvas_map_temp_stereo->DrawArrow(1.0, 0.0, (dist_frc_polar[i] * cos(i * 9 * PI / 180.0)) + 1, dist_frc_polar[i] * sin(i * 9 * PI / 180.0) , false);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "FRP -----> dist_frc_polar[i]: ", dist_frc_polar[i], " cos( i * 9 * PI / 180.0): ", cos(i * 9 * PI / 180.0), " sin( i * 9 * PI / 180.0): ", sin(i * 9 * PI / 180.0));
      dist_polar_frc_sectors[i] = dist_frc_polar[i];
    }//if
    else
    {
      dist_polar_frc_sectors[i] = 10;
    }
  }

  delete [] dist_polar;
  delete [] dist_frc_polar;
  delete [] polar_frp_sectors_distance;
  delete [] polar_flp_sectors_distance;

  out_canvas_map_temp_stereo->SetFill(true);
  out_canvas_map_temp_stereo->SetEdgeColor(0, 0, 0);
  out_canvas_map_temp_stereo->SetFillColor(200, 200, 255);
  out_canvas_map_temp_stereo->DrawBox(-1.0, -3.3, 2.0, 3.3);
  out_canvas_map_temp_stereo->DrawArrow(0.0, -2.0, 0.0, -1.0, false);


  //publishing the port
  so_map_grid_stereo.Publish(map_pointer_stereo);
  so_map_grid_stereo_env.Publish(map_stereo_env);
  so_canvas_map_env_stereo.Publish(output_canvas_map_env_stereo_pointer);
  so_canvas_map_temp_stereo.Publish(output_canvas_map_temp_stereo_pointer);

}// gridmap_stereo

void mStViPr::sectormap()
{
  /*
   * sector mapping
   */
  /* --------------> stereo <---------------------*/
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridPolar2D<double>> sectormap_pointer_flp = so_sectormap_flp.GetUnusedBuffer();
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridPolar2D<double>> sectormap_pointer_frp = so_sectormap_frp.GetUnusedBuffer();
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> sectormap_pointer_fc = so_sectormap_fc.GetUnusedBuffer();
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> sectormap_pointer_flc = so_sectormap_fcl.GetUnusedBuffer();


  //    rrlib::mapping::tMapGridPolar2D<double>::tBounds bounds;
  //    bounds.upper_bounds[0] = tVec(rrlib::math::tAngleDeg(90), 0); // first dimension: 90 degress till
  //    bounds.lower_bounds[0] = tVec(rrlib::math::tAngleDeg(-90), 0);  // -90 degrees
  //    bounds.upper_bounds[1] = tVec(0, 30); // second dimension: 20 meters to
  //    bounds.lower_bounds[1] = tVec(0, 0); // 0 meters
  //    map.SetBounds(bounds);
  //    map.SetResolution(tVec(rrlib::math::tAngleDeg(18), 1)); // resolution: 18 degrees, 1 meter

  //front right polar FRP
  rrlib::mapping::tMapGridPolar2D<double>::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(0, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar += rrlib::mapping::tPolar2D::tCoordinate(90, 1);
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  sectormap_pointer_frp->SetBounds(bounds_polar);
  //  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(10, 5); //test to make sure about overwriting the legth of sectors
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(9, 1);
  sectormap_pointer_frp->SetResolution(coordinate_polar);



  //front left polar FLP
  rrlib::mapping::tMapGridPolar2D<double>::tBounds bounds_polar_flp;
  rrlib::mapping::tMapGridPolar2D<double>::tCoordinate coordinate_polar_flp(90, 0);
  bounds_polar_flp.lower_bounds[0] = coordinate_polar_flp;
  bounds_polar_flp.lower_bounds[1] = coordinate_polar_flp;
  coordinate_polar_flp += rrlib::mapping::tPolar2D::tCoordinate(180, 1);
  bounds_polar_flp.upper_bounds[0] = coordinate_polar_flp;
  bounds_polar_flp.upper_bounds[1] = coordinate_polar_flp;
  sectormap_pointer_flp->SetBounds(bounds_polar_flp);
  coordinate_polar_flp = rrlib::mapping::tPolar2D::tCoordinate(9, 1);
  sectormap_pointer_flp->SetResolution(coordinate_polar_flp);


  for (unsigned int i = 0; i < 10; i++)
  {
    sectormap_pointer_flp->GetCellByCellID(i) = 0;
    sectormap_pointer_frp->GetCellByCellID(i) = 0;


    if (dist_polar_sectors[i] < 5)
    {
      sectormap_pointer_flp->GetCellByCellID(i) = 3; //red
    }//if
    if (dist_polar_frc_sectors[i] < 5)
    {
      sectormap_pointer_frp->GetCellByCellID(i) = 3; //red
    }//if
  }



  //front cartesian ==> left to right
  rrlib::mapping::tCartesian2D::tCoordinate coordinate_frc(0, 0);
  rrlib::mapping::tCartesian2D::tBounds bounds_frc;
  bounds_frc.lower_bounds[0] = coordinate_frc;
  bounds_frc.lower_bounds[1] = coordinate_frc;
  coordinate_frc += rrlib::mapping::tCartesian2D::tCoordinate(1, 1);
  bounds_frc.upper_bounds[0] = coordinate_frc;
  bounds_frc.upper_bounds[1] = coordinate_frc;
  sectormap_pointer_fc->SetBounds(bounds_frc);
  coordinate_frc = rrlib::mapping::tCartesian2D::tCoordinate(0.2, 1);
  sectormap_pointer_fc->SetResolution(coordinate_frc);

  //front cartesian left ==> left to right
  rrlib::mapping::tCartesian2D::tCoordinate coordinate_flc(-1, 0);
  rrlib::mapping::tCartesian2D::tBounds bounds_flc;
  bounds_flc.lower_bounds[0] = coordinate_flc;
  bounds_flc.lower_bounds[1] = coordinate_flc;
  coordinate_flc += rrlib::mapping::tCartesian2D::tCoordinate(1, 1);
  bounds_flc.upper_bounds[0] = coordinate_flc;
  bounds_flc.upper_bounds[1] = coordinate_flc;
  sectormap_pointer_flc->SetBounds(bounds_flc);
  coordinate_flc = rrlib::mapping::tCartesian2D::tCoordinate(0.2, 1);
  sectormap_pointer_flc->SetResolution(coordinate_flc);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "sectormap_pointer_flc->GetCoordinateByCellID(0): ", sectormap_pointer_flc->GetCoordinateByCellID(0));


  for (unsigned int i = 0; i < 5; i++)
  {
    sectormap_pointer_flc->GetCellByCellID(i) = 0; //red
    sectormap_pointer_fc->GetCellByCellID(i) = 0; //red

    if (dist_sectors[4 - i] < 5)
    {
//         sectormap_pointer_flc->GetCellByCellID(4-i) = 3; //red
      sectormap_pointer_flc->GetCellByCellID(i) = 3; //red
    }//if
    if (dist_frc_sectors[i] < 5)
    {
      sectormap_pointer_fc->GetCellByCellID(i) = 3; //red
    }//if
  }


  //test
//     sectormap_pointer_flc->GetCellByCellID(4) = 3; //red
//     sectormap_pointer_flc->GetCellByCellID(3) = 2; //red
//     sectormap_pointer_flc->GetCellByCellID(2) = 1; //red
//     sectormap_pointer_flc->GetCellByCellID(1) = 0; //red
//     sectormap_pointer_flc->GetCellByCellID(0) = -1; //red



  so_sectormap_flp.Publish(sectormap_pointer_flp);
  so_sectormap_frp.Publish(sectormap_pointer_frp);
  so_sectormap_fc.Publish(sectormap_pointer_fc);
  so_sectormap_fcl.Publish(sectormap_pointer_flc);

}

//----------------------------------------------------------------------
// mStViPr constructor
//----------------------------------------------------------------------
mStViPr::mStViPr(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name),  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  input_iplimage_left(NULL),  //tImage2cvMat
  input_iplimage_right(NULL), //tImage2cvMat
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
  image_viewer_original(new pcl::visualization::ImageViewer("Image Viewer Mask")),
  image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
  image_viewer_debugging(new visualization::ImageViewer("Image Viewer Debugging")),  //debugging only
  road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
{

  fprintf(stderr, "mStViPr constructor started.....................................................................................\n");  //::ctor >>

  input_iplimage_left = cvCreateImageHeader(cvSize(0, 0), IPL_DEPTH_32F, 3);
  input_iplimage_right = cvCreateImageHeader(cvSize(0, 0), IPL_DEPTH_32F, 3);

  display_normals = false;

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->registerKeyboardCallback(&mStViPr::keyboardCallback, *this, 0);

  /*! susgested for AdaptiveCostSOStereoMatching same as gray*/
  stereo.setMaxDisparity(60); //original

  /*
   * setter for horizontal offset, i.e. number of pixels to shift the disparity range over the target image
   */
  stereo.setXOffset(0); //original
  stereo.setRadius(5); //original

  smooth_weak = 20;
  smooth_strong = 100;
  stereo.setSmoothWeak(smooth_weak); //20 original
  stereo.setSmoothStrong(smooth_strong); //original
  stereo.setGammaC(25); //original
  stereo.setGammaS(10); //10 original was original

  //stereo.setRatioFilter(20);
  //stereo.setPeakFilter(0);
  //stereo.setLeftRightCheck(true);
  //stereo.setLeftRightCheckThreshold(1);
  stereo.setPreProcessing(true);


  /*Set up the normal estimation based on kinematic capability and the terrain for GroundPlaneComparator*/
  /*
   * COVARIANCE_MATRIX - creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood.
  AVERAGE_3D_GRADIENT - creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients.
  AVERAGE_DEPTH_CHANGE - creates only a single integral image and computes the normals from the average depth changes.
   */
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);

  /*The depth change threshold for computing object borders.
  max_depth_change_factor  the depth change threshold for computing object borders based on depth changes
   * */
  ne.setMaxDepthChangeFactor(0.03f);

  /*normal_smoothing_size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
   * */
  ne.setNormalSmoothingSize(40.0f); //20.0f

  /*BORDER_POLICY_IGNORE  BORDER_POLICY_MIRROR*/
  ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

  /*  decides whether the smoothing is depth dependent*/
  bool use_depth_dependent_smoothing = false;
  ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

  // Set up the ground plane comparator -- If the camera was pointing straight out, the normal would be:
  Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)

  /*Adjust for camera tilt for traversability analysis*/
  /*Set the expected ground plane normal with respect to the stereo camera.
   * Pixels labeled as ground must be within ground_angular_threshold radians of this normal to be labeled as ground.
   *     */
  Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
  road_comparator->setExpectedGroundNormal(tilt_road_normal);

  /*slope threshold and Set the tolerance in radians for difference in normal direction between a point and the expected ground normal.*/
  float   ground_angular_threshold = pcl::deg2rad(20.0f);  //10.0f original
  road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original


  /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
  float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians //3.0f original
  road_comparator->setAngularThreshold(angular_threshold); //3.0f original

  /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
  float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
  bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
  road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);


  /*---------------mapping stereo -------------------*/
  /*--------------------- environment mapping ----------------*/
  bounds_stereo_env.upper_bounds[0] = rrlib::math::tVec2d(bound_limit, 0);
  bounds_stereo_env.lower_bounds[0] = rrlib::math::tVec2d(-bound_limit, 0);
  bounds_stereo_env.upper_bounds[1] = rrlib::math::tVec2d(0, bound_limit);
  bounds_stereo_env.lower_bounds[1] = rrlib::math::tVec2d(0, -bound_limit);
  map_stereo_env.SetBounds(bounds_stereo_env);
  map_stereo_env.SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); // resolution: 1 meter, 1 meter

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

    /* ----terrain classification---------*/
    run();

    /*------mapping-----------*/
    gridmap_stereo();
    sectormap();

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
