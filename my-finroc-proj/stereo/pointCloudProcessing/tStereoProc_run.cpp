
#include "projects/stereo_traversability_experiments/aras/pointCloudProcessing/tStereoProcessing.h"

#include <pcl/io/pcd_io.h>

#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"
#include <pcl/filters/voxel_grid.h>

//using namespace finroc::icarus::sensor_processing::pointCloudProcessing;
using namespace finroc::stereo_traversability_experiments::aras::pointCloudProcessing;

void tStereoProcessing::run()
{

  while (!viewer->wasStopped())
  {

    /*! Proceed or nor to the next image*/
    if (trigger || continuous || bwd)
    {

      run_proceed_callbacks();

      //stereo_reconst(); //generating point cloud
      CloudPtr cloud(new Cloud);
      PCDReader pcd;
      pcd.read(left_images[images_idx], *cloud);
      prev_cloud = cloud;


//      // processing point cloud  - my proposed approach
//      processCloud_normals();
//      processCloud_segm();
//      processCloud_trav();

      pcl::VoxelGrid<PointT>* ds(new VoxelGrid<PointT>);   //create downsampling filter
      CloudPtr image_cloud_ds(new Cloud);
      ds->setInputCloud(cloud);
      ds->setLeafSize(0.2f, 0.2f, 0.2f);
      ds->filter(*image_cloud_ds);
      cout << "Orginal cloud size" << cloud->size() <<
           " Downsampled cloud size = " << image_cloud_ds->size() << endl;
      *cloud = *image_cloud_ds;
//        image_cloud_ds->clear();

      // UPD --------- mauro's approach
      finroc::mauro::test::UPD trav_upd_test;
      trav_upd_test.image_cloud = cloud;
      trav_upd_test.upd();
      prev_cloud = trav_upd_test.image_cloud;
      prev_cloud_segm = trav_upd_test.colored_cloud;

      run_initialize();

    }// if trigger

    run_visualize();

  } // while

}// run
