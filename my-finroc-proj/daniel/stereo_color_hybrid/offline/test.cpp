/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

int
main(int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO("usage: daniel_stereoTravExp_daniel_stereoColor_hybrid_offline left_image_directory right_image_directory intrinsic_parameter_filename extrinsic_parameter_filename\n");
    PCL_INFO("note: images in both left and right folders can be in different format.\n");
    PCL_INFO("for example : \n"

             ///mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/frames_pcd_acso/006-catas/selection/left_002.pcd

             "\n ================> stereo lugv color -- Marche-en-Farme catasrophiy -- 000: MAIN\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "~//mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/left_rect/000/3 "
             "~//mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/right_rect/000/3 "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/extrinsics.yml\n "

             "\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/left_rect/000/4 "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/right_rect/000/4 "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/extrinsics.yml\n "

             "\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/left_rect/000/5 "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/right_rect/000/5 "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/extrinsics.yml\n "

             "\n ================> stereo lugv color -- Marche-en-Farme catasrophiy -- 006: catasrophyyyyyyyyyyyy\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/left_rect/006-catas "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/right_rect/006-catas "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/extrinsics.yml\n "

             "\n ================> stereo lugv color -- Marche-en-Farme catasrophiy -- 006: catasrophyyyyyyyyyyyy main \n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/home/d_poh/Pictures/color_detritus/left_images/1280X960 "
             "/home/d_poh/Pictures/color_detritus/right_images/1280X960 "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/calibrate_with_8x6_and_10cm/intrinsics.yml "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/calibrate_with_8x6_and_10cm/extrinsics.yml\n "


             "\n ================> stereo lugv color -- Marche-en-Farme catasrophiy -- 006: catasrophyyyyyyyyyyyy main selection\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/left_rect/006-catas/selection "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2014-09-08_MeF_DS/right_rect/006-catas/selection "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/calibrate_with_8x6_and_10cm/intrinsics.yml "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/calibrate_with_8x6_and_10cm/extrinsics.yml\n "


             "\n ================> stereo lugv color -- outside in road in RMA: GOOD\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/ICARUS/stereo_lugv_color-haris/2014_07_Haris_Stereo_System/3dv-dataset_RMA_outside/left_rect/ "
             "/mnt/public_work/Testdaten/ICARUS/stereo_lugv_color-haris/2014_07_Haris_Stereo_System/3dv-dataset_RMA_outside/right_rect "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/extrinsics.yml\n "

             "\n ==============> stereo lugv color haris -- inside office in RMA: OK \n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/ICARUS/stereo_lugv_color-haris/2014-02-13_SV_System_RMA/3dv-dataset_3/left_rect/ "
             "/mnt/public_work/Testdaten/ICARUS/stereo_lugv_color-haris/2014-02-13_SV_System_RMA/3dv-dataset_3/right_rect "
             "~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/intrinsics.yml "
             "~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/extrinsics.yml\n "

             "\n ================> stereo_lugv_haris -- two main databases for outside and rough offroad in lawn and grass: GREAT \n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_1/left_rect/ "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_1/right_rect "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_1/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_1/calib_1-ok/extrinsics.yml\n "

             "\n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/left_rect/ "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/right_rect "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_lugv-haris/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_2/calib_1-ok/extrinsics.yml\n "

             "\n ================> stereo ravon color -- tests calibrate_with_8x6_and_10cm - need some changes for stereo reconst -- > SEG FAULT FOR rectification (stereo reconst)!!! ????? \n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/test/left "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/test/right "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/calibrate_with_8x6_and_10cm/intrinsics.yml "
             "/mnt/public_work/Testdaten/ICARUS/stereo_ravon_color/calibrate_with_8x6_and_10cm/extrinsics.yml\n "

             "\n ===========================> stereo webcam color -- OK \n"
             "daniel_stereoTravExp_daniel_stereoColor_hybrid_offline "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_webcam/webcam_2013-04-15_labor-mit-Xtion/1st/left "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_webcam/webcam_2013-04-15_labor-mit-Xtion/1st/right "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_webcam/webcam_2013-04-15_labor-mit-Xtion/calib/intrinsics.yml "
             "/mnt/public_work/Testdaten/StereoImages_and_PointCloudData/stereo_webcam/webcam_2013-04-15_labor-mit-Xtion/calib/extrinsics.yml\n "

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
