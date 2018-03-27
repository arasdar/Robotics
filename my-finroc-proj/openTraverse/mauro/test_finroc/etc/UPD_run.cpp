
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

using namespace finroc::mauro::test;

//void UPD::run_test(const vector<string> left_images, const unsigned img_pairs_num)
void UPD::run_test()
{

  while (!viewer_test->wasStopped())
  {

    /*! Proceed or nor to the next image*/
    if (trigger || continuous || bwd)
    {

      run_proceed_callbacks();

//      //stereo_reconst(); //generating point cloud
      CloudPtr cloud(new Cloud);
      PCDReader pcd;
//      pcd.read(left_images[images_idx], *cloud);
//      prev_cloud = cloud;

//      // processing point cloud
//      processCloud_normals();
//      processCloud_segm();
//      processCloud_trav();
//
//      run_initialize();

    }// if trigger

//    run_visualize();

  } // while

}// run
