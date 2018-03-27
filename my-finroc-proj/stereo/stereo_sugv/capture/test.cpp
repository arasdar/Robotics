/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include "projects/icarus/sensor_processing/stereo_sugv/capture/tStereoProcessing.h"

using namespace finroc::icarus::sensor_processing::stereo_sugv::capture;

int
main(int argc, char** argv)
{

  /*capture online, Process and display*/
  tStereoProcessing stereo_vision_processing;
  stereo_vision_processing.run();

  return 0;
}// main
