

//#include "projects/icarus/sensor_processing/pointCloudProcessing/tStereoProcessing.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

//using namespace finroc::icarus::sensor_processing::pointCloudProcessing;
using namespace finroc::mauro::test;

void UPD::run_proceed_callbacks()
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
