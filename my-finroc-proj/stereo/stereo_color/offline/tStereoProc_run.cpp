
#include "projects/stereo_traversability_experiments/aras/stereo_color/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::stereo_color::offline;

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
      processCloud_trav();

      run_initialize();

//      run_saveCloud(prev_cloud);

    }// if trigger

    run_visualize();

  } // while

}// run
