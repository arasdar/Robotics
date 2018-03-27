/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include "projects/stereo_traversability_experiments/daniel/segmentation_with_normals/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_normals;

int
main(int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO("usage: daniel_stereoTravExp_segmentation_normals left_image_directory right_image_directory intrinsic_parameter_filename extrinsic_parameter_filename\n");
    PCL_INFO("note: images in both left and right folders can be in different format.\n");
    PCL_INFO("for example : \n"
             "daniel_stereoTravExp_segmentation_normals /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/left/ /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/right/ /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/calib_1-ok/intrinsics.yml /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/calib_1-ok/extrinsics.yml\n ");
    return -1;
  }


  /*variable initial*/
  int img_number_left = 0, img_number_right = 0 ;
  int img_pairs_num = 0;

  /*Get list of stereo files from left folder*/
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  /*reading right images from folder*/
  std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr(argv[2]); itr != end_itr; ++itr)
  {
    right_images.push_back(itr->path().string());
    img_number_right++;
  }
  sort(right_images.begin(), right_images.end());
  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  /*showing the input images*/
  cout << "img_number_left: " << img_number_left << std::endl;
  cout << "img_number_right: " << img_number_right << std::endl;
  if (img_number_left == img_number_right)
    img_pairs_num = img_number_left;

  /*calibration parameters*/
  string input_intrinsic_filename = argv[3];
  string input_extrinsic_filename = argv[4];

  /*Process and display*/
  tStereoProcessing stereo_vision_processing(left_images, right_images, img_pairs_num, input_intrinsic_filename, input_extrinsic_filename);
  stereo_vision_processing.run();

  return 0;
}// main
