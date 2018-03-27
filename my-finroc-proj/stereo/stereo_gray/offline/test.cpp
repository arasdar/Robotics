/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include "projects/stereo_traversability_experiments/aras/stereo_gray/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::stereo_gray::offline;

int
main(int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO("usage: aras_stereoTravExp_aras_stereoGray_offline left_image_directory right_image_directory intrinsic_parameter_filename extrinsic_parameter_filename\n");
    PCL_INFO("note: images in both left and right folders can be in different format.\n");
    PCL_INFO("for example : \n"
             "\n stereo ravon gray -- the test outside labor IN FOREST AND CONSTRUCTION SITE-- GOOD\n"
             "aras_stereoTravExp_aras_stereoGray_offline /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/left/ /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/right/ /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/calib_1-ok/intrinsics.yml /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/calib_1-ok/extrinsics.yml\n "

             "\n stereo ravon gray -- the test outside labor IN FOREST AND CONSTRUCTION SITE complete-- OK \n"
             "aras_stereoTravExp_aras_stereoGray_offline ~/mine/stereo_images/stereo_ravon_gray/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/left/ ~/mine/stereo_images/stereo_ravon_gray/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/right/ ~/mine/stereo_images/stereo_ravon_gray/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/intrinsics.yml ~/mine/stereo_images/stereo_ravon_gray/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/extrinsics.yml\n "
             "aras_stereoTravExp_aras_stereoGray_offline ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/left ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/right/ ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/intrinsics.yml ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/extrinsics.yml\n "
             "aras_stereoTravExp_aras_stereoGray_offline ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/left_ramp ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/right_ramp ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/intrinsics.yml ~/mine/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/extrinsics.yml\n"

             "\n stereo ravon gray -- the test outside labor in winter -- BAD CALIBRATION \n"
             "aras_stereoTravExp_aras_stereoGray_offline /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-02-26_Stereo_Images_Dataset_Rough_Terrain/left /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-02-26_Stereo_Images_Dataset_Rough_Terrain/right /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-02-26_Stereo_Images_Dataset_Rough_Terrain/calib/intrinsics.yml /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-02-26_Stereo_Images_Dataset_Rough_Terrain/calib/extrinsics.yml\n "

             "\n stereo ravon gray -- marche-en-farme --- catasrophoie - 1,2,3,4 (all) with after calib -- BAD\n"
             "aras_stereoTravExp_aras_stereoGray_offline /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-07-29_myStereo-baseline-45cm_3rd_test-on-theodor-Marche/1/left /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-07-29_myStereo-baseline-45cm_3rd_test-on-theodor-Marche/1/right /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-07-29_myStereo-baseline-45cm_3rd_test-on-theodor-Marche/calib_after-collecting-data/calib_2-ok_better/intrinsics.yml /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-07-29_myStereo-baseline-45cm_3rd_test-on-theodor-Marche/calib_after-collecting-data/calib_2-ok_better/extrinsics.yml\n "

            );
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
