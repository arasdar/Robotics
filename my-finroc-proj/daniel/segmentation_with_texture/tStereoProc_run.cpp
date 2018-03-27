
#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_texture;

void tStereoProcessing::run()
{

  while (!viewer_proc_segm_normal->wasStopped())
  {

    /*! Proceed or nor to the next image*/
    if (trigger || continuous || bwd)
    {

      run_proceed_callbacks();

      stereo_reconst(); //generating point cloud

      // segment with texture
      processCloud_normals(prev_cloud);
      processImage_texture();
      processCloud_segm();

      run_initialize();

    }// if trigger

    run_visualize();

  } // while

}// run
