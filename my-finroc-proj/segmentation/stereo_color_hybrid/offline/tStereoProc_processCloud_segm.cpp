

#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void
tStereoProcessing::processCloud_segm()
{

  /*! Initializing process cloud - segmentation*/
  float roughness = deg2rad(1.0f); //the tolerance in radians
  comp->setAngularThreshold(roughness); //3.0f original

  /*Set up the ground plane comparator*/
  comp->setInputCloud(prev_cloud);
  comp->setInputNormals(prev_normal_cloud);

  /*Run segmentation*/
  pcl::PointCloud<pcl::Label> segment_labels;
  std::vector<pcl::PointIndices> segments;
  segm.setInputCloud(prev_cloud);
  segm.segment(segment_labels, segments);

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;

  /*! Visualizing the segments on the pcd*/
  for (unsigned i = 0; i < segments.size(); ++i)
  {
    if (segments[i].indices.size() > thresh_segments)
    {
      for (unsigned j = 0; j < segments[i].indices.size(); ++j)
      {
        cloud->points[segments[i].indices[j]].g = 255;
      } // for j
    } //if
  }// for i


  prev_cloud_segm = cloud;
  prev_cloud_segments = segments;
  prev_cloud_segment_labels = segment_labels;
  drawContoursAroundSegments(cloud, prev_cloud_segment_labels, prev_cloud_segments);
}
