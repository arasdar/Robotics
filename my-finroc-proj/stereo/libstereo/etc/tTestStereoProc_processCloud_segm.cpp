


#include "projects/icarus/sensor_processing/libstereo_test/tTestStereoProcessing.h"

using namespace finroc::icarus::sensor_processing::libstereo_test;

void
tTestStereoProcessing::processCloud_segmentation(const CloudConstPtr& cloud)
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


  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids;
  std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> covariances;
  std::vector<pcl::PointIndices> inlier_indices;
  std::vector<float> region_indices_eigvals;


  unsigned int min_inliers = (cloud->width / 10) * (cloud->height / 10) * 0.1; // * 1.5 * 0.5;  //2500
  cout << "threshold_min_dominant_plane_cloud: " << min_inliers << endl;

  /*! traversable regions - green*/
  for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
  {

    /*looking for only the largest region*/
    if (region_indices[i].indices.size() > min_inliers) //100 original //min_inliers - green -- for each segment 1000 original
    {

      /*! Compute plane info*/
      Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
      Eigen::Matrix3f clust_cov;
      pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);

      EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
      EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
      pcl::eigen33(clust_cov, eigen_value, eigen_vector); //eigen Val not used TODO
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

      pcl::ModelCoefficients model;
      model.values.resize(4);
      model.values[0] = plane_params[0];
      model.values[1] = plane_params[1];
      model.values[2] = plane_params[2];
      model.values[3] = plane_params[3]; //check if this is improtant or not TODO not used
      model_coefficients.push_back(model);
      inlier_indices.push_back(region_indices[i]);
      centroids.push_back(clust_centroid);
      covariances.push_back(clust_cov);
      region_indices_eigvals.push_back(eigen_value);

      /*visualizing the result by colorizing the point cloud */ /*! traversable regions - green*/
      for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
      {
        ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
        label_image->points[region_indices[i].indices[j]].r = 0;
        label_image->points[region_indices[i].indices[j]].g = 255 ;
        label_image->points[region_indices[i].indices[j]].b = 0;
        pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                cloud->points[region_indices[i].indices[j]].y,
                                cloud->points[region_indices[i].indices[j]].z);
        ground_cloud->points.push_back(ground_pt);
      }// for


    } //if region
  }// for i


  /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
  if (ground_cloud->points.size() > 0)
  {
    ground_centroid = centroids[0]; // this needs to be the biggest TODO
    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
  }// if


  /*// note the NAN points in the image as well*/
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
  {
    cloud_mutex.lock();
    prev_cloud = cloud;
    prev_normal_cloud = normal_cloud;
    prev_ground_cloud = ground_cloud;
    prev_ground_image = ground_image;
    prev_label_image = label_image;
    prev_ground_normal = ground_plane_params;
    prev_ground_centroid = ground_centroid;
    cloud_mutex.unlock();
  }
}
