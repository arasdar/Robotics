
#include "projects/stereo_traversability_experiments/daniel/segmentation_with_normals/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_normals;

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
      processCloud_segm();

      run_initialize();

    }// if trigger

    run_visualize();

  } // while

}// run
