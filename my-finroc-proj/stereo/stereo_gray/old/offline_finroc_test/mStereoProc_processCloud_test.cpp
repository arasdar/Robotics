


#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc_test/mStereoGrayOffline.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline_test;

void mStereoGrayOffline::processSmallSegments()
{

  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "REGION_SIZE of ALL INDICES: ", prev_region_indices.size());
  PointIndices::Ptr indices(new PointIndices);
//  ExtractIndices<PointT>* extract(new ExtractIndices);
  ExtractIndices<PointT> extract;

  if (prev_enoughPoints)
  {
    unsigned thresh = 500; //number of inlying points in each segmented region
    for (unsigned i = 0; i < prev_region_indices.size(); ++i)
    {
      if (prev_region_indices[i].indices.size() > thresh && prev_region_indices[i].indices.size() < prev_ground_size)
      {
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
        Eigen::Matrix3f clust_cov;
        computeMeanAndCovarianceMatrix(*prev_cloud, prev_region_indices[i].indices, clust_cov, clust_centroid);

        /*! Checking the cluster centroid or each segmented region elevation (height) to see
         * if it is within dominant ground elevation region*/
        pcl::PointXYZ centroid_pt(clust_centroid[0], clust_centroid[1], clust_centroid[2]);
        double ptp_dist =  pcl::pointToPlaneDistance(centroid_pt, prev_ground_normal[0], prev_ground_normal[1], prev_ground_normal[2], prev_ground_normal[3]);
        if (ptp_dist < step_max)
        {
          for (unsigned j = 0; j < prev_region_indices[i].indices.size(); j++)
          {
            prev_ground_image->points[prev_region_indices[i].indices[j]].g = 200;
          }


//          *indices = prev_region_indices[i];
//          extract.setInputCloud(prev_ground_image);
//          extract.setIndices(indices);
//          extract.setNegative(true);
//          extract.filter(*prev_ground_image);

        }
        /*
         * ! detecting obstacle but too noisy
                //if (ptp_dist > step_max && centroid_pt[2] > )
                else if (prev_region_indices[i].indices.size() > 2 * thresh)
                {
                  for (unsigned j = 0; j < prev_region_indices[i].indices.size(); ++j)
                  {
                    prev_ground_image->points[prev_region_indices[i].indices[j]].r = 255;

                  }
                }
        */

      } //if thresh
    }// for region_indices[i]
  }// if enoughPoints

}

void
mStereoGrayOffline::processCloud_test(const CloudConstPtr& cloud, const CloudConstPtr& cloud_disp)
{
  /*Compute the normals*/
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(cloud); //input
  ne.compute(*normal_cloud); //output

  /*Set up the ground plane comparator*/
  road_comparator->setInputCloud(cloud);
  road_comparator->setInputNormals(normal_cloud);

  /*Run segmentation*/
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> region_indices;
  road_segmentation.setInputCloud(cloud);
  road_segmentation.segment(labels, region_indices);

  /*! Draw the segmentation result*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  CloudPtr ground_image(new Cloud);
  CloudPtr label_image(new Cloud);
  *ground_image = *cloud;
  *label_image = *cloud;
  CloudPtr disp_image(new Cloud);
  *disp_image = *cloud_disp;

  /*! Finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
  std::vector<int> region_indices_size;
  for (unsigned int i = 0; i < region_indices.size(); i++)
  {
    region_indices_size.push_back(region_indices[i].indices.size());
  }
  unsigned dominant_size = *max_element(region_indices_size.begin(), region_indices_size.end()); //region_indices_size.at(region_indices_size.size() - 1);
  Eigen::Vector4f dominant_ground_normal(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f dominant_ground_centroid(0.0, 0.0, 0.0, 0.0);
//  vector<float> dominant_y;

  // Create the filtering object
  ExtractIndices<PointT> extract;
  extract.setInputCloud(disp_image);
  PointIndices::Ptr indices_dom(new PointIndices);

  /*! Dominant traversable ground detection - green*/
  for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
  {

    /*! Looking for only the largest region*/
    if (region_indices[i].indices.size() == dominant_size && dominant_size > 1000)
    {
      /*! Compute plane info*/
      Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
      Eigen::Matrix3f clust_cov;
      pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);

      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
      pcl::eigen33(clust_cov, eigen_value, eigen_vector);
      Eigen::Vector4f plane_params;
      plane_params[0] = eigen_vector[0];
      plane_params[1] = eigen_vector[1];
      plane_params[2] = eigen_vector[2];
      plane_params[3] = 0;

      /*! 1- D*/
      plane_params[3] = -1 * plane_params.dot(clust_centroid);

      /*! 2- D*/
      Eigen::Vector4f vp = Eigen::Vector4f::Zero();
      vp -= clust_centroid;
      float cos_theta = vp.dot(plane_params);
      if (cos_theta < 0)
      {
        plane_params *= -1;
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot(clust_centroid);
      }

      dominant_ground_normal = plane_params;
      dominant_ground_centroid = clust_centroid;
      FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "eigen value for dominant: ", eigen_value);

      /*! Visualizing the dominant detected ground*/
      for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
      {
        unsigned color = 255;
        ground_image->points[region_indices[i].indices[j]].g = color; //static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
        disp_image->points[region_indices[i].indices[j]].g = color;
        label_image->points[region_indices[i].indices[j]].r = 0;
        label_image->points[region_indices[i].indices[j]].g = color;
        label_image->points[region_indices[i].indices[j]].b = 0;

//        /*! Finding the min and max height or elevation*/
//        dominant_y.push_back(ground_image->points[region_indices[i].indices[j]].y);
      }// for

      *indices_dom = region_indices[i];
    }
  }// for i REGION INDICES

  /*! Filtering based on the dominant ground elevation*/
  bool enoughPoints = true;
  unsigned thresh = 1000; //(cloud->width / 10) * (cloud->height / 10) * 1.5 * 10;
  if (dominant_size < thresh)
  {
    enoughPoints = false;
  }

  /*! Updated from now on to continue processing the cloud if we have dominant plane with enough inlying points*/
  if (enoughPoints)
  {
    /*! Filtering the dominant ground */
    extract.setIndices(indices_dom);
    extract.setNegative(true);
    extract.filter(*disp_image);

    prev_enoughPoints = enoughPoints;
    prev_region_indices = region_indices; //this should be done by memecpy
    FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "region indices: ", region_indices.size());
    prev_ground_size = dominant_size;
  }

  /*! Note the NAN points in the image as well*/
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if (!pcl::isFinite(cloud->points[i]))
    {
      ground_image->points[i].b = static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
      label_image->points[i].r = 0;
      label_image->points[i].g = 0;
      label_image->points[i].b = 255;
    }
  }

  /*! Update info for the visualization thread*/
  if (enoughPoints || images_idx == 0) //updated if enough points
  {
    cloud_mutex.lock();
    prev_cloud = cloud;
    prev_normal_cloud = normal_cloud;
    prev_ground_cloud = ground_cloud;
    prev_ground_image = ground_image;
    prev_disp_image = disp_image;
    prev_label_image = label_image;
    prev_ground_normal = dominant_ground_normal;
    prev_ground_centroid = dominant_ground_centroid;
    cloud_mutex.unlock();
  }

}
