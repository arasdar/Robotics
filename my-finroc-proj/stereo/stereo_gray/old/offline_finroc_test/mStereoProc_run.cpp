/*
 * mStereoProc_run_all.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;

void
mStereoGrayOffline::run()
{

  /*! Proceed or nor to the next image*/
  if (trigger || continuous || bwd)
  {

    if (img_pairs_num == images_idx - 1)
    {
      cout << "no more images................   == > img_pairs_num == images_idx: " << images_idx << std::endl;
      cout << "img_pairs_num: " << img_pairs_num << endl;

      trigger = false;
      continuous = false;
      bwd = false;

      /*
            return EXIT_SUCCESS;
            exit(EXIT_SUCCESS);
      */

    } //if
    else
    {

      if (bwd)
      {
        if (images_idx > 0)
        {
          images_idx--;
        }
        bwd = false;
      }

      if (continuous || trigger)
      {
        images_idx++;
        trigger = false;
      }


      stereo_rectify(left_images[images_idx], right_images[images_idx]);

      /*process the disparity map and point cloud*/
      stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
      stereo.medianFilter(4);  // better but slower //optimal

      /*stereo images camera calibration parameters for ptgrey grayscale camera*/
      /*
            float u_c = P2_calib.at<double>(0, 2); //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
            float v_c = P2_calib.at<double>(1, 2); //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
            float focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
            float baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator
      */

      float u_c = P2_calib.at<double>(0, 2) - pt_top_left_crop.x; //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
      float v_c = P2_calib.at<double>(1, 2) - pt_top_left_crop.y; //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
      float focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
      float baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator

      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "X_c: ", u_c, " Y_c:", v_c, " focal: ", focal, " baseline: ", baseline, " point_crop: ", pt_top_left_crop);

      CloudPtr out_cloud(new Cloud);
      stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud);

      /* displaying the points selected by gui - mouse and */
      Cloud::Ptr clicked_points_3d(new Cloud);
      cb_args.clicked_points_3d = clicked_points_3d;
      cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
      cb_args.viewerPtr_2 = pcl::visualization::PCLVisualizer::Ptr(viewer_disparity);
      cb_args.viewerPtr_3 = pcl::visualization::PCLVisualizer::Ptr(viewer_original);
      cb_args.viewerPtr_4 = pcl::visualization::PCLVisualizer::Ptr(viewer_disp_proc);
      cb_args.viewerPtr_5 = pcl::visualization::PCLVisualizer::Ptr(viewer_disparity_processed);


      /*removing and clean up*/
      cb_args.clicked_points_3d->clear();
      viewer->removeAllPointClouds();  //ok
      viewer_disparity->removeAllPointClouds();
      viewer_disparity_processed->removeAllPointClouds();
      viewer_original ->removeAllPointClouds();
      viewer_disp_proc->removeAllPointClouds();
      for (int i = 0; i < 100; i++)
      {
        viewer->removeText3D(text_id_str[i]);
        viewer_disparity->removeText3D(text_id_str[i]);
        viewer_disparity_processed->removeText3D(text_id_str[i]);
        viewer_original->removeText3D(text_id_str[i]);
        viewer_disp_proc->removeText3D(text_id_str[i]);
      }//for
      text_id = 0;

      /*! displaying disparity map*/
      /*! visualizing in opencv*/
      /*
            Mat vmap_opencv(left_img_rect.rows, left_img_rect.cols, CV_8UC3);
            stereo.getVisualMap(vmap_opencv);
            imshow("vmap_opencv", vmap_opencv);
            waitKey(1);
      */

      /*visualizing in pcl*/
      pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
      stereo.getVisualMap(vmap);
      image_viewer_disparity->addRGBImage<RGB> (vmap);

      CloudPtr out_cloud_disp(new Cloud);
      stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud_disp, vmap);
      if (!viewer_disparity->updatePointCloud(out_cloud_disp, "cloud disparity"))
      {
        viewer_disparity->addPointCloud(out_cloud_disp, "cloud disparity");
      }


      viewer->removeText3D("cloud");
      image_viewer->removeLayer("line");  //const std::string &layer_id

      //processCloud(out_cloud, out_cloud_disp);
      processCloud_test(out_cloud, out_cloud_disp);
      processSmallSegments();
      //filterCloudOutlierRemoval();
      //filterCloudRadius();

      cout << "left_images[img_index]: " << left_images[images_idx] << std::endl;
      cout << "right_images[img_index]: " << right_images[images_idx] << std::endl;
      cout << "images_idx: " << images_idx << endl;
      cout << "img_pairs_num: " << img_pairs_num << endl;
      cout << "press space to continue to the next image ................. " << endl;


      /*! Camera angle with respect the expected nominal ground*/
      Eigen::Vector4f expected_ground_normal(tilt_road_normal[0], tilt_road_normal[1], tilt_road_normal[2], 0.0);
      double cos_theta = prev_ground_normal.dot(expected_ground_normal);
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "cos_theta: ", cos_theta);

      const double PI = 3.14159265;       //#define PI 3.14159265
      double result = acos(cos_theta) * 180.0 / PI;
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "the angle: ", result);

    }//else

  }// if trigger

  drawVisualization();

}// run





