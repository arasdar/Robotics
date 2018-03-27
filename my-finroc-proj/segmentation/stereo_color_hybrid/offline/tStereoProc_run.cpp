
#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::run()
{

  while (!viewer->wasStopped())
  {

    /*! Proceed or nor to the next image*/
    if (trigger || continuous || bwd)
    {

      run_proceed_callbacks();

      stereo_reconst(); //generating point cloud

      // processing point cloud
      processCloud_normals(prev_cloud);
      processCloud_invalid_points();
      processCloud_segm();
      processCloud_segm_appear();
      processCloud_trav();

      run_initialize();

//      run_saveCloud(prev_cloud);

    }// if trigger

    run_visualize();

  } // while

}// run
