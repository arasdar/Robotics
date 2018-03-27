
//#include "UPD.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

int  main(int argc, char** argv)
{

  if (argc < 2)
  {
    PCL_INFO("usage: aras_stereoTravExp_openTraverse_mauro_test left_pcd_directory\n");
    PCL_INFO("note: pcd in frames folder is PCD format.\n");
    PCL_INFO("for example : \n"
//            "aras_stereoTravExp_openTraverse_mauro_test -m /mnt/public_work/Testdaten/ICARUS/stereo_ravon_gray/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/frames_pcd_acso/frames_binary\n "
             "aras_stereoTravExp_openTraverse_mauro_test -m ~/Desktop/frames_pcd_acso/pcd_list.txt\n"
             "aras_stereoTravExp_openTraverse_mauro_test -m ~/mine/openTraverse/mauro/data/dataset_main/pcd_list.txt\n"
            );
    return -1;
  }

  finroc::mauro::test::UPD upd_test;
  upd_test.run(argc, argv);

}
