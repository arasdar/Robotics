
#include "projects/stereo_traversability_experiments/openTraverse/pointCloudProcessing/tStereoProcessing.h"

#include "projects/stereo_traversability_experiments/openTraverse/mauro/tUPD.h"
#include "projects/stereo_traversability_experiments/openTraverse/alex/tHRCSSegmentation.h"
#include "projects/stereo_traversability_experiments/openTraverse/aras/tSSTA.h"
#include "projects/stereo_traversability_experiments/openTraverse/igor/worker.h"

#include <pcl/console/parse.h>

using namespace finroc::stereo_traversability_experiments::openTraverse::pointCloudProcessing;

void tStereoProcessing::run(int argc, char** argv)
{

  while (!viewer->wasStopped())
  {

    /*! Proceed or nor to the next image*/
    if (trigger || continuous || bwd)
    {

      run_proceed_callbacks();

      CloudPtr cloud(new Cloud);
      PCDReader pcd;
      pcd.read(left_images[images_idx], *cloud);
      *prev_cloud = *cloud;

      if (console::find_argument(argc, argv, "--mauro") >= 0)
      {

        // UPD --------- mauro's approach
        finroc::stereo_traversability_experiments::openTraverse::mauro::tUPD trav_upd_test;
        *trav_upd_test.image_cloud = *cloud;
        trav_upd_test.upd();
        *prev_cloud_filtered = *trav_upd_test.image_cloud;
        *prev_cloud_trav = *trav_upd_test.colored_cloud;
      }

      /* HRCS Segmentation ------------ alex's approach       */
      if (console::find_argument(argc, argv, "--alex") >= 0)
      {
        finroc::stereo_traversability_experiments::openTraverse::alex::tHRCSSegmentation trav_hrcs;
        trav_hrcs.processCloud(cloud);
        *prev_cloud_filtered = *trav_hrcs.prev_label_image;
        *prev_cloud_trav = *trav_hrcs.prev_ground_image;
      }


      if (console::find_argument(argc, argv, "--aras") >= 0)
      {
        // SSTA ------------- my approach
        finroc::stereo_traversability_experiments::openTraverse::aras::tSSTA trav_ssta;
        *trav_ssta.prev_cloud = *cloud;
        trav_ssta.processCloud_normals(cloud);
        trav_ssta.processCloud_segm();
        *prev_cloud_filtered = *trav_ssta.prev_cloud_segm;
        trav_ssta.processCloud_trav();
        *prev_cloud_trav = *trav_ssta.prev_cloud_trav_stepAnalysis;
      }

      if (console::find_argument(argc, argv, "--igor") >= 0)
      {
        //kinect-based traversability analyzer and map generator
        stereo_traversability_experiments::openTraverse::igor::Worker trav_worker;
        int maxAngle = 30; //30 deg
        float cellWidth = 0.1f, //10 cm
              maxTravHeight = 0.3f, //30 cm
              radiusNormals = 0.1f,
              robotHeight = 1.0f;
        trav_worker.processCloudToMap(
          maxAngle,
          cellWidth,
          maxTravHeight,
          radiusNormals,
          robotHeight,
          cloud);
        *prev_cloud_filtered = *cloud;
        *prev_cloud_trav = *cloud;
      }

      run_initialize();

    }// if trigger

    run_visualize();

  } // while

}
