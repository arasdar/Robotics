

#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void
tStereoProcessing::processCloud_segm_appear()
{
  int algo = 0; // 0 = SLIC; 1 = Felzenszwalb
  if (algo == 0)
  {
    processCloud_segm_slic(left_images[images_idx]);
  }
}
