/** \brief This is an application for processing point clouds to classify terrain based on traversability estimation.
  *
  * \author Aras Dargazany
  */

#include "projects/icarus/sensor_processing/libsegmentation/tests/tPointCloudProcessing.h"

int
main(int argc, char** argv)
{

  if (argc < 2)
  {
    PCL_INFO("usage: aras_icarus_sensorProcessing_libsegmentation_test pcd_frames_directory \n");
    PCL_INFO("note: frames must be in PCD format. \n");
    PCL_INFO("for example : \n"
             "aras_icarus_sensorProcessing_libsegmentation_test ~/stereo_images/kinect_sugv/frames\n "
             "aras_icarus_sensorProcessing_libsegmentation_test ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/acso/frames_pcd_left\n "
             "aras_icarus_sensorProcessing_libsegmentation_test  ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/acso_test/frames_pcd_left \n");
    return -1;

  }

  /*// variable initial*/
  int img_number_left = 0;

  /*// Get list of stereo files*/
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());
  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  /*// showing the input images*/
  cout <<  "frames_number: " << img_number_left << endl;

  /*// Process and display*/
  finroc::icarus::sensor_processing::libsegmentation::PointCloudProcessing point_cloud_processing(left_images , img_number_left);
  point_cloud_processing.run();

  return 0;
}
