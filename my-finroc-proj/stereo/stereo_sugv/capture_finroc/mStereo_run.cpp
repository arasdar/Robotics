/*
 * tStereoProc_run.cpp
 *
 *  Created on: May 12, 2014
 *      Author: aras
 */

#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.h"

using namespace finroc::icarus::sensor_processing::stereo_sugv::capture;

void
mStereo::run()
{

  /*
    for (;;){
        stereoCapture(); //images_idx
    //    images_idx ++;
    }
  */

  /*
    while (!viewer->wasStopped())
    {}
  */

  stereoCapture();
  images_idx++;

  stereoRect();

  /*! displaying disparity map*/
  stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
  stereo.medianFilter(10);  // better but slower //optimal

  /*! visualizing in opencv*/
  Mat vmap_opencv(left_img_rect.rows, left_img_rect.cols, CV_8UC3);
  stereo.getVisualMap(vmap_opencv);
  imshow("vmap_opencv", vmap_opencv);
  waitKey(1);

  float u_c = 652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  float v_c = 492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
  float focal = 1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  float baseline = 4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator

  pcl::PointCloud<PointT>::Ptr out_cloud(new pcl::PointCloud<PointT>);
  stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud);

  /*
  //    if (!viewer->updatePointCloud(out_cloud, "3D reconstructed cloud"))
  //      viewer->addPointCloud(out_cloud, "3D reconstructed cloud");
  */

  processCloud(out_cloud); //dominant plane detection

  /*
      //Draw visualizations
      //if (cloud_mutex.try_lock())  //undefined ref to pthread with new gcc 4.8
      //{         cloud_mutex.unlock();      }//if
  */

  if (!viewer->updatePointCloud(prev_ground_image, "cloud")) //&& !viewer_test->updatePointCloud(prev_ground_image_test, "cloud_test")
  {

    viewer->addPointCloud(prev_ground_image, "cloud");
    //viewer_test->addPointCloud(prev_ground_image_test, "cloud_test");
  }

  if (prev_cloud->points.size() > 1000)
  {
    image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
  }// if

  // showing normals on point clouds
  bool display_normals = false;
  if (prev_normal_cloud->points.size() > 1000 && display_normals)
  {
    viewer->removePointCloud("normals");
    viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
  }

  /*
  //    comparing the normals of real ground and expected ground plane for normal
  //    if (prev_ground_cloud->points.size() > 0) {        }// if
  */

  /*
      // Show the ground plane normal
      Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
  */

  // Show the ground plane normal
  pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
  pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                    prev_ground_centroid[1] + prev_ground_normal[1],
                    prev_ground_centroid[2] + prev_ground_normal[2]);
  pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                    prev_ground_centroid[1] + tilt_road_normal[1],
                    prev_ground_centroid[2] + tilt_road_normal[2]);

  // comparing the normals of real ground and expected ground plane for normal
  viewer->removeShape("ground_norm");
  viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
  viewer->removeShape("expected_ground_norm");
  viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");

  viewer->spinOnce(1);
  image_viewer->spinOnce(1);

}// run






