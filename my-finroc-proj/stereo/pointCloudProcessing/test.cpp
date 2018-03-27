/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include "projects/stereo_traversability_experiments/aras/pointCloudProcessing/tStereoProcessing.h"

//using namespace finroc::icarus::sensor_processing::pointCloudProcessing;
using namespace finroc::stereo_traversability_experiments::aras::pointCloudProcessing;
int
main(int argc, char** argv)
{

  if (argc < 2)
  {
    PCL_INFO("usage: aras_stereoTravExp_aras_pointCloudProcessing left_pcd_directory\n");
    PCL_INFO("note: pcd in frames folder is PCD format.\n");
    PCL_INFO("for example : \n"
             "aras_stereoTravExp_aras_pointCloudProcessing /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/frames_pcd_acso/frames_binary\n "
             "aras_stereoTravExp_aras_pointCloudProcessing ~/Desktop/frames_pcd_acso-pcd/\n"
             "aras_stereoTravExp_aras_pointCloudProcessing ~/mine/openTraverse/mauro/data/dataset_main-pcd/\n"
            );
    return -1;
  }


  /*variable initial*/
  unsigned img_number_left = 0;
  unsigned img_pairs_num = 0;

  /*Get list of stereo files from left folder*/
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");
  PCL_INFO("Press 'n' to enable normals\n");

  /*showing the input images*/
  cout << "img_number_left: " << img_number_left << std::endl;
  img_pairs_num = img_number_left;

  /*Process and display*/
  tStereoProcessing point_cloud_processing(left_images, img_pairs_num);
  point_cloud_processing.run();

  return 0;
}// main
