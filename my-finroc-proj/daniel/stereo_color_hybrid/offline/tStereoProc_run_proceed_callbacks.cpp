

#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::run_proceed_callbacks()
{


  if (img_pairs_num == images_idx - 1)
  {
    cout << "no more images................   == > img_pairs_num == images_idx: " << images_idx << std::endl;
    cout << "img_pairs_num: " << img_pairs_num << endl;

    trigger = false;
    continuous = false;
    bwd = false;
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

  }//else

}



void tStereoProcessing::run_proceed_callbacks_test()
{
  char key = waitKey(0);
  if (key = ' ')
  {
    images_idx++;
  }
}
