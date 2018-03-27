

#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_texture;

void
tStereoProcessing::processCloud_segm()
{

  /*! Initializing process cloud - segmentation*/
  float roughness = deg2rad(5.0f); //the tolerance in radians
  comp_normal->setAngularThreshold(roughness); //3.0f original
  comp_texture->setAngularThreshold(deg2rad(10.0)); //3.0f original

  /*Set up the ground plane comparator*/
  comp_normal->setInputCloud(prev_cloud);
  comp_normal->setInputNormals(prev_normal_cloud);
  comp_texture->setInputCloud(prev_cloud);
  comp_texture->setInputTextures(prev_texture_cloud);


  /*Run segmentation*/
  pcl::PointCloud<pcl::Label> segment_labels_normal;
  std::vector<pcl::PointIndices> segments_normal;
  segm_normal.setInputCloud(prev_cloud);
  segm_normal.segment(segment_labels_normal, segments_normal);

  pcl::PointCloud<pcl::Label> segment_labels_texture;
  std::vector<pcl::PointIndices> segments_texture;
  segm_texture.setInputCloud(prev_cloud);
  segm_texture.segment_texture(segment_labels_texture, segments_texture);



  CloudPtr cloud_normal(new Cloud);
  *cloud_normal = *prev_cloud;
  CloudPtr cloud_texture(new Cloud);
  *cloud_texture = *prev_cloud;

  /*! Visualizing the segments on the pcd*/
  for (unsigned i = 0; i < segments_normal.size(); ++i)
  {
    if (segments_normal[i].indices.size() > thresh_segments)
    {
      for (unsigned j = 0; j < segments_normal[i].indices.size(); ++j)
      {
        cloud_normal->points[segments_normal[i].indices[j]].g = 255;
      } // for j
    } //if
  }// for i

  for (unsigned i = 0; i < segments_texture.size(); ++i)
  {
    if (segments_texture[i].indices.size() > thresh_segments)
    {
      for (unsigned j = 0; j < segments_texture[i].indices.size(); ++j)
      {
        cloud_texture->points[segments_texture[i].indices[j]].g = 255;
      } // for j
    } //if
  }// for i

  prev_cloud_segm_normal = cloud_normal;
  prev_cloud_segments_normal = segments_normal;
  prev_cloud_segment_labels_normal = segment_labels_normal;

  prev_cloud_segm_texture = cloud_texture;
  prev_cloud_segments_texture = segments_texture;
  prev_cloud_segment_labels_texture = segment_labels_texture;

}
