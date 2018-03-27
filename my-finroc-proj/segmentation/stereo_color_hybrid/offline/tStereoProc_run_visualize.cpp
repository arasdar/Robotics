
#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void
tStereoProcessing::run_visualize()
{

  viewer->removeText3D("cloud");
  //image_viewer->removeLayer("line");  //const std::string &layer_id

  //visualizing generated point cloud and extracted normals on it optionally
  if (!viewer->updatePointCloud(prev_cloud, "cloud"))
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_cloud);
    viewer->addPointCloud(prev_cloud, rgb, "cloud"); //works too
  }//if

  viewer->removePointCloud("normals");
  if (display_normals)
  {
    viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_cloud, prev_normal_cloud, 10, 0.15f, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
  }
  image_viewer->addRGBImage<PointT>(prev_cloud, "rgb_image", 0.3);

  if (!viewer_disparity->updatePointCloud(prev_cloud_disp, "cloud disparity"))
  {
    viewer_disparity->addPointCloud(prev_cloud_disp, "cloud disparity");
  }
  image_viewer_disparity->addRGBImage<RGB>(prev_img_disp);

  if (!viewer_disparity_processed->updatePointCloud(prev_cloud_disp, "cloud disp proc"))
  {
    viewer_disparity_processed->addPointCloud(prev_cloud_disp, "cloud disp proc");
  }
  image_viewer_disparity_processed->addRGBImage<PointT>(prev_cloud_disp, "cloud disp proc");


  if (!viewer_proc_invalid_points->updatePointCloud(prev_cloud_invalid_points, "cloud_invalid_points"))
  {
    viewer_proc_invalid_points->addPointCloud(prev_cloud_invalid_points, "cloud_invalid_points");
  }
  image_viewer_proc_invalid_points->addRGBImage<PointT>(prev_cloud_invalid_points, "cloud invalid points");

  // visualizing segmented regions on the point cloud
  if (!viewer_proc_segm->updatePointCloud(prev_cloud_segm, "cloud_segm")) //previous cloud for segmentation results
  {
    viewer_proc_segm->addPointCloud(prev_cloud_segm, "cloud_segm");
  }
  image_viewer_proc_segm->addRGBImage<PointT>(prev_cloud_segm);

  // visualizing segmented regions on the point cloud
  if (!viewer_proc_segm_appear->updatePointCloud(prev_cloud_segm_appear, "cloud_segm_appear")) //previous cloud for segmentation results
  {
    viewer_proc_segm_appear->addPointCloud(prev_cloud_segm_appear, "cloud_segm_appear");
  }
  image_viewer_proc_segm_appear->addRGBImage<PointT>(prev_cloud_segm_appear);



  if (!viewer_proc_trav->updatePointCloud(prev_cloud_trav_dominGroundPlane, "cloud_trav"))
  {
    viewer_proc_trav->addPointCloud(prev_cloud_trav_dominGroundPlane, "cloud_trav");
  }
  image_viewer_proc_trav->addRGBImage<PointT>(prev_cloud_trav_dominGroundPlane);

  if (!viewer_proc_trav_slopeAnalysis->updatePointCloud(prev_cloud_trav_slopeAnalysis, "cloud trav slope"))
  {
    viewer_proc_trav_slopeAnalysis->addPointCloud(prev_cloud_trav_slopeAnalysis, "cloud trav slope");
  }
  image_viewer_proc_trav_slopeAnalysis->addRGBImage<PointT>(prev_cloud_trav_slopeAnalysis, "cloud trav slope");

  if (!viewer_proc_trav_slopeAnalysis_appear->updatePointCloud(prev_cloud_trav_slopeAnalysis_appear, "cloud trav slope appear"))
  {
    viewer_proc_trav_slopeAnalysis_appear->addPointCloud(prev_cloud_trav_slopeAnalysis_appear, "cloud trav slope appear");
  }
  image_viewer_proc_trav_slopeAnalysis_appear->addRGBImage<PointT>(prev_cloud_trav_slopeAnalysis_appear, "cloud trav slope appear");

  /*
  if (!viewer_proc_trav_slopeMapping_discrete->updatePointCloud(prev_cloud_trav_slopeMapping_discrete, "cloud trav slope mapped discrete"))
  {
    viewer_proc_trav_slopeMapping_discrete->addPointCloud(prev_cloud_trav_slopeMapping_discrete, "cloud trav slope mapped discrete");
  }
  image_viewer_proc_trav_slopeMapping_discrete->addRGBImage<PointT>(prev_cloud_trav_slopeMapping_discrete, "cloud trav slope mapped discrete");

  if (!viewer_proc_trav_stepMapping_discrete->updatePointCloud(prev_cloud_trav_stepMapping_discrete, "cloud trav step mapped discrete"))
  {
    viewer_proc_trav_stepMapping_discrete->addPointCloud(prev_cloud_trav_stepMapping_discrete, "cloud trav step mapped discrete");
  }
  image_viewer_proc_trav_stepMapping_discrete->addRGBImage<PointT>(prev_cloud_trav_stepMapping_discrete, "cloud trav step mapped discrete");
  */

  if (!viewer_proc_trav_stepAnalysis->updatePointCloud(prev_cloud_trav_stepAnalysis, "cloud trav step"))
  {
    viewer_proc_trav_stepAnalysis->addPointCloud(prev_cloud_trav_stepAnalysis, "cloud trav step");
  }
  image_viewer_proc_trav_stepAnalysis->addRGBImage<PointT>(prev_cloud_trav_stepAnalysis, "cloud trav step");

  if (!viewer_proc_trav_stepAnalysis_appear->updatePointCloud(prev_cloud_trav_stepAnalysis_appear, "cloud trav step appear"))
  {
    viewer_proc_trav_stepAnalysis_appear->addPointCloud(prev_cloud_trav_stepAnalysis_appear, "cloud trav step appear");
  }
  image_viewer_proc_trav_stepAnalysis_appear->addRGBImage<PointT>(prev_cloud_trav_stepAnalysis_appear, "cloud trav step appear");

  /*
  if (!viewer_proc_trav_slopeFusion_discrete->updatePointCloud(prev_cloud_trav_slopeFusion_discrete, "cloud trav slope fused discrete"))
  {
    viewer_proc_trav_slopeFusion_discrete->addPointCloud(prev_cloud_trav_slopeFusion_discrete, "cloud trav slope fused discrete");
  }
  image_viewer_proc_trav_slopeFusion_discrete->addRGBImage<PointT>(prev_cloud_trav_slopeFusion_discrete, "cloud trav slope fused discrete");

  if (!viewer_proc_trav_stepFusion_discrete->updatePointCloud(prev_cloud_trav_stepFusion_discrete, "cloud trav step fused discrete"))
  {
    viewer_proc_trav_stepFusion_discrete->addPointCloud(prev_cloud_trav_stepFusion_discrete, "cloud trav step fused discrete");
  }
  image_viewer_proc_trav_stepFusion_discrete->addRGBImage<PointT>(prev_cloud_trav_stepFusion_discrete, "cloud trav step fused discrete");

  */
  if (!viewer_proc_trav_geomFusion_discrete->updatePointCloud(prev_cloud_trav_geomFusion_discrete, "cloud trav geom fused discrete"))
  {
    viewer_proc_trav_geomFusion_discrete->addPointCloud(prev_cloud_trav_geomFusion_discrete, "cloud trav geom fused discrete");
  }
  image_viewer_proc_trav_geomFusion_discrete->addRGBImage<PointT>(prev_cloud_trav_geomFusion_discrete, "cloud trav geom fused discrete");


  if (!viewer_proc_trav_geomFusionMapping_discrete->updatePointCloud(prev_cloud_trav_fusedMapping_discrete, "cloud trav geom fused and mapped discrete"))
  {
    viewer_proc_trav_geomFusionMapping_discrete->addPointCloud(prev_cloud_trav_fusedMapping_discrete, "cloud trav geom fused and mapped discrete");
  }
  image_viewer_proc_trav_geomFusionMapping_discrete->addRGBImage<PointT>(prev_cloud_trav_fusedMapping_discrete, "cloud trav geom fused and mapped discrete");


  if (!viewer_proc_trav_geomFusionMapping_discrete_uh->updatePointCloud(prev_cloud_trav_fusedMapping_discrete_uh, "cloud trav geom fused and mapped unknown handled discrete"))
  {
    viewer_proc_trav_geomFusionMapping_discrete_uh->addPointCloud(prev_cloud_trav_fusedMapping_discrete_uh, "cloud trav geom fused and mapped unknown handled discrete");
  }
  image_viewer_proc_trav_geomFusionMapping_discrete_uh->addRGBImage<PointT>(prev_cloud_trav_fusedMapping_discrete_uh, "cloud trav geom fused and mapped unknown handled discrete");

  /*
  if (!viewer_proc_trav_finalFusion_discrete->updatePointCloud(prev_cloud_trav_finalFusion_discrete, "cloud trav final fused discrete"))
  {
    viewer_proc_trav_finalFusion_discrete->addPointCloud(prev_cloud_trav_finalFusion_discrete, "cloud trav final fused discrete");
  }
  image_viewer_proc_trav_finalFusion_discrete->addRGBImage<PointT>(prev_cloud_trav_finalFusion_discrete, "cloud trav final fused discrete");

  if (!viewer_proc_trav_final->updatePointCloud(prev_cloud_trav_final, "cloud trav final"))
  {
    viewer_proc_trav_final->addPointCloud(prev_cloud_trav_final, "cloud trav final");
  }
  image_viewer_proc_trav_final->addRGBImage<PointT>(prev_cloud_trav_final, "cloud trav final");

  if (!viewer_proc_trav_final_appear->updatePointCloud(prev_cloud_trav_final_appear, "cloud trav final appear"))
  {
    viewer_proc_trav_final_appear->addPointCloud(prev_cloud_trav_final_appear, "cloud trav final appear");
  }
  image_viewer_proc_trav_final_appear->addRGBImage<PointT>(prev_cloud_trav_final_appear, "cloud trav final appear");
  */

  viewer->spinOnce(1);
  image_viewer->spinOnce(1);

  viewer_disparity->spinOnce(1);
  image_viewer_disparity->spinOnce(1);

  viewer_disparity_processed->spinOnce(1);
  image_viewer_disparity_processed->spinOnce(1);

  viewer_proc_invalid_points->spinOnce(1);
  image_viewer_proc_invalid_points->spinOnce(1);

  viewer_proc_segm->spinOnce(1);
  image_viewer_proc_segm->spinOnce(1);

  viewer_proc_segm_appear->spinOnce(1);
  image_viewer_proc_segm_appear->spinOnce(1);

  viewer_proc_trav->spinOnce(1);
  image_viewer_proc_trav->spinOnce(1);

  viewer_proc_trav_stepAnalysis->spinOnce(1);
  image_viewer_proc_trav_stepAnalysis->spinOnce(1);

  viewer_proc_trav_stepAnalysis_appear->spinOnce(1);
  image_viewer_proc_trav_stepAnalysis_appear->spinOnce(1);


  viewer_proc_trav_slopeAnalysis->spinOnce(1);
  image_viewer_proc_trav_slopeAnalysis->spinOnce(1);

  viewer_proc_trav_slopeAnalysis_appear->spinOnce(1);
  image_viewer_proc_trav_slopeAnalysis_appear->spinOnce(1);

  //viewer_proc_trav_slopeMapping_discrete->spinOnce(1);
  //image_viewer_proc_trav_slopeMapping_discrete->spinOnce(1);

  //viewer_proc_trav_stepMapping_discrete->spinOnce(1);
  //image_viewer_proc_trav_stepMapping_discrete->spinOnce(1);

  //viewer_proc_trav_slopeFusion_discrete->spinOnce(1);
  //image_viewer_proc_trav_slopeFusion_discrete->spinOnce(1);

  //viewer_proc_trav_stepFusion_discrete->spinOnce(1);
  //image_viewer_proc_trav_stepFusion_discrete->spinOnce(1);

  viewer_proc_trav_geomFusion_discrete->spinOnce(1);
  image_viewer_proc_trav_geomFusion_discrete->spinOnce(1);


  viewer_proc_trav_geomFusionMapping_discrete->spinOnce(1);
  image_viewer_proc_trav_geomFusionMapping_discrete->spinOnce(1);


  viewer_proc_trav_geomFusionMapping_discrete_uh->spinOnce(1);
  image_viewer_proc_trav_geomFusionMapping_discrete_uh->spinOnce(1);


  //viewer_proc_trav_finalFusion_discrete->spinOnce(1);
  //image_viewer_proc_trav_finalFusion_discrete->spinOnce(1);


  //viewer_proc_trav_final->spinOnce(1);
  //image_viewer_proc_trav_final->spinOnce(1);

  //viewer_proc_trav_final_appear->spinOnce(1);
  //image_viewer_proc_trav_final_appear->spinOnce(1);


  waitKey(1);
}// run





