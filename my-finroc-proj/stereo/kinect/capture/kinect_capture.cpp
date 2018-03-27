/** \brief kinectTraversability is an application for processing point clouds to classify terrain based on traversability estimation using kinect.
  *
  * \author Aras Dargazany
  */

// FIXME: openni relies on the non-standard "i386" macro GCC used to define
#if __i386__
#define i386 1
#endif
//#include <XnCppWrapper.h>  //this one is from rrlib/openni/tDevice
#include <pcl/io/openni_grabber.h>
#undef i386

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

//#include <pcl/sample_consensus/sac_model_plane.h> //point2plane distace -- seg fault with system libs

//template
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class kinectVisionProcessing
{
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer;
  CloudConstPtr prev_cloud;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;
  boost::mutex cloud_mutex;

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
  int images_idx;

  bool display_normals;
  bool detect_obstacles;

  pcl::PCDWriter writer_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;

  CloudConstPtr cloud_;
  mutable boost::mutex cloud_mutex_;
  pcl::OpenNIGrabber &grabber_;

public:
  kinectVisionProcessing(pcl::OpenNIGrabber &grabber):
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    image_viewer(new pcl::visualization::ImageViewer("Image Viewer"))
    ,
    road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
    road_segmentation(road_comparator)
    ,
    writer_(),
    file_name_("frames_pcd"),
    dir_name_("frames"),
    format_(4)
    ,
    grabber_(grabber)
  {
    display_normals = false;
    detect_obstacles = true; //default false

    images_idx = 0;

    // Set up a 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
    //    viewer->addCoordinateSystem(1.0); //default
    double scale = 2.0;
    float x = 0.0; //0
    float y = 0.5; //0.5
    float z = 1.0 ; //1
    int viewport = 0;
    //Adds 3D axes describing a coordinate system to screen at x, y, z.
    viewer->addCoordinateSystem(scale, x, y, z, viewport);
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(&kinectVisionProcessing::keyboardCallback, *this, 0);

    // Set up the normal estimation
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.03f);
    ne.setNormalSmoothingSize(40.0f); //20.0f

    // Set up the groundplane comparator
    // If the camera was pointing straight out, the normal would be:
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
    // Adjust for camera tilt:
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);
    road_comparator->setGroundAngularThreshold(pcl::deg2rad(10.0f));
    road_comparator->setAngularThreshold(pcl::deg2rad(3.0f));
  }

  ~kinectVisionProcessing()
  {
  }

  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.keyUp())
    {
      switch (event.getKeyCode())
      {
      case 'n':
        display_normals = !display_normals;
        break;
      case 'o':
        detect_obstacles = !detect_obstacles;
        break;
      }
    }
  }

  void
  processCloud(const CloudConstPtr& cloud)
  {
    // Compute the normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.compute(*normal_cloud);

    // Set up the ground plane comparator
    road_comparator->setInputCloud(cloud);
    road_comparator->setInputNormals(normal_cloud);

    // Run segmentation
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> region_indices;
    road_segmentation.setInputCloud(cloud);
    road_segmentation.segment(labels, region_indices);

    // Draw the segmentation result
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    CloudPtr ground_image(new Cloud);
    CloudPtr label_image(new Cloud);
    *ground_image = *cloud;
    *label_image = *cloud;

    Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
    Eigen::Vector4f vp = Eigen::Vector4f::Zero();
    Eigen::Matrix3f clust_cov;
    pcl::ModelCoefficients model;
    model.values.resize(4);

    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids;
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> covariances;
    std::vector<pcl::PointIndices> inlier_indices;

    // traversable regions - green
    for (int i = 0; i < region_indices.size(); i++)
    {
      if (region_indices[i].indices.size() > 1000)
      {

        for (int j = 0; j < region_indices[i].indices.size(); j++)
        {
          pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                  cloud->points[region_indices[i].indices[j]].y,
                                  cloud->points[region_indices[i].indices[j]].z);
          ground_cloud->points.push_back(ground_pt);
          ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
          label_image->points[region_indices[i].indices[j]].r = 0;
          label_image->points[region_indices[i].indices[j]].g = 255;
          label_image->points[region_indices[i].indices[j]].b = 0;
        }

        // Compute plane info
        pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);
        Eigen::Vector4f plane_params;

        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        pcl::eigen33(clust_cov, eigen_value, eigen_vector);
        plane_params[0] = eigen_vector[0];
        plane_params[1] = eigen_vector[1];
        plane_params[2] = eigen_vector[2];
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot(clust_centroid);

        vp -= clust_centroid;
        float cos_theta = vp.dot(plane_params);
        if (cos_theta < 0)
        {
          plane_params *= -1;
          plane_params[3] = 0;
          plane_params[3] = -1 * plane_params.dot(clust_centroid);
        }

        model.values[0] = plane_params[0];
        model.values[1] = plane_params[1];
        model.values[2] = plane_params[2];
        model.values[3] = plane_params[3];
        model_coefficients.push_back(model);
        inlier_indices.push_back(region_indices[i]);
        centroids.push_back(clust_centroid);
        covariances.push_back(clust_cov);
      } //if region
    }// for i

    //Refinement
    std::vector<bool> grow_labels;
    std::vector<int> label_to_model;
    grow_labels.resize(region_indices.size(), false);
    label_to_model.resize(region_indices.size(), 0);

    for (size_t i = 0; i < model_coefficients.size(); i++)
    {
      int model_label = (labels)[inlier_indices[i].indices[0]].label;
      label_to_model[model_label] = static_cast<int>(i);
      grow_labels[model_label] = true;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
    *labels_ptr = labels;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
    refinement_compare->setInputCloud(cloud);
    refinement_compare->setDistanceThreshold(0.15f);
    refinement_compare->setLabels(labels_ptr);
    refinement_compare->setModelCoefficients(model_coefficients);
    refinement_compare->setRefineLabels(grow_labels);
    refinement_compare->setLabelToModel(label_to_model);
    mps.setRefinementComparator(refinement_compare);
    mps.setMinInliers(500);
    mps.setAngularThreshold(pcl::deg2rad(3.0));
    mps.setDistanceThreshold(0.02);
    mps.setInputCloud(cloud);
    mps.setInputNormals(normal_cloud);
    mps.refine(model_coefficients,
               inlier_indices,
               centroids,
               covariances,
               labels_ptr,
               region_indices);

    //Note the regions that have been extended
    Cloud extended_ground_cloud;
    for (int i = 0; i < region_indices.size(); i++)
    {
      if (region_indices[i].indices.size() > 1000)
      {
        for (int j = 0; j < region_indices[i].indices.size(); j++)
        {
          // Check to see if it has already been labeled
          if (ground_image->points[region_indices[i].indices[j]].g == ground_image->points[region_indices[i].indices[j]].b)
          {
            pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                    cloud->points[region_indices[i].indices[j]].y,
                                    cloud->points[region_indices[i].indices[j]].z);
            ground_cloud->points.push_back(ground_pt);
            ground_image->points[region_indices[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].r + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
            label_image->points[region_indices[i].indices[j]].r = 128;
            label_image->points[region_indices[i].indices[j]].g = 128;
            label_image->points[region_indices[i].indices[j]].b = 0;
          }

        }
      }
    }

    Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);

    if (ground_cloud->points.size() > 0)
    {
      ground_centroid = centroids[0];
      ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
    }

//    // Segment Obstacles
//    if (detect_obstacles)
//    {
//      Cloud::CloudVectorType clusters;
//      if (ground_cloud->points.size() > 0)
//      {
//        std::vector<bool> plane_labels;
//        plane_labels.resize(region_indices.size(), false);
//        for (size_t i = 0; i < region_indices.size(); i++)
//        {
//          if (region_indices[i].indices.size() > mps.getMinInliers())
//          {
//            plane_labels[i] = true;
//          }
//        }
//
//        pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
//
//        euclidean_cluster_comparator_->setInputCloud(cloud);
//        euclidean_cluster_comparator_->setLabels(labels_ptr);
//        euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
//        euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);
//
//        pcl::PointCloud<pcl::Label> euclidean_labels;
//        std::vector<pcl::PointIndices> euclidean_label_indices;
//        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
//        euclidean_segmentation.setInputCloud(cloud);
//        euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);
//
//        for (size_t i = 0; i < euclidean_label_indices.size(); i++)
//        {
//          if ((euclidean_label_indices[i].indices.size() > 200))
//          {
//            Cloud cluster;
//            pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
//            clusters.push_back(cluster);
//
//            Eigen::Vector4f cluster_centroid;
//            Eigen::Matrix3f cluster_cov;
//            pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);
//
//            pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//            double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//            if ((ptp_dist > 0.5) && (ptp_dist < 3.0)) //point-to-plane dist
//            {
//
//              for (int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
//              {
//                ground_image->points[euclidean_label_indices[i].indices[j]].r = 255;
//                label_image->points[euclidean_label_indices[i].indices[j]].r = 255;
//                label_image->points[euclidean_label_indices[i].indices[j]].g = 0;
//                label_image->points[euclidean_label_indices[i].indices[j]].b = 0;
//              }
//
//            }
//
//          }
//        } //for size
//
//      } //if ground cloud
//    } //if obstacles

    // note the NAN points in the image as well
    for (int i = 0; i < cloud->points.size(); i++)
    {
      if (!pcl::isFinite(cloud->points[i]))
      {
        ground_image->points[i].b = static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
        label_image->points[i].r = 0;
        label_image->points[i].g = 0;
        label_image->points[i].b = 255;
      }
    } //for NAN

    // Update info for the visualization thread
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

  void
  cloud_cb_(const CloudConstPtr& cloud)
  {
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_ = cloud;
  }

  CloudConstPtr
  getLatestCloud()
  {
    //lock while we swap our cloud and reset it.
    boost::mutex::scoped_lock lock(cloud_mutex_);
    CloudConstPtr temp_cloud;
    temp_cloud.swap(cloud_);  //here we set cloud_ to null, so that it is safe to set it again from ourcallback
    return (temp_cloud);
  }

  void saveCloud(CloudConstPtr& cloud)
  {
    std::stringstream ss;
    ss << dir_name_ << "/" << file_name_ << "_" << images_idx << ".pcd";
    cout << "Saved " << ss.str() << endl;

    if (format_ & 1)
    {
      writer_.writeBinary<PointT> (ss.str(), *cloud);
      //std::cerr << "Data saved in BINARY format to " << ss.str () << std::endl;
    }

    if (format_ & 2)
    {
      writer_.writeBinaryCompressed<PointT> (ss.str(), *cloud);
      //std::cerr << "Data saved in BINARY COMPRESSED format to " << ss.str () << std::endl;
    }

    if (format_ & 4)
    {
      writer_.writeBinaryCompressed<PointT> (ss.str(), *cloud);
      //std::cerr << "Data saved in BINARY COMPRESSED format to " << ss.str () << std::endl;
    }
  }

  void run_capture()
  {

    // make callback function from member function
    boost::function<void (const CloudConstPtr&)> f = boost::bind(&kinectVisionProcessing::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = grabber_.registerCallback(f);

    // start receiving point clouds
    grabber_.start();

    while (!viewer->wasStopped())
    {

      // Process a new image
      //if (trigger || continuous)
      //{ trigger = false;} //if


      if (cloud_)
      {
        CloudConstPtr cloud = getLatestCloud();
        saveCloud(cloud);
        processCloud(cloud);
      }// if

      // for saving the cloud
      images_idx ++;

      // Draw visualizations
      //if (cloud_mutex.try_lock())
      //{
      //cloud_mutex.unlock();
      //}//if

      if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
        viewer->addPointCloud(prev_ground_image, "cloud");

      if (prev_normal_cloud->points.size() > 1000 && display_normals)
      {
        viewer->removePointCloud("normals");
        viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
      }

      if (prev_cloud->points.size() > 1000)
      {
        image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
      }

      // Show the groundplane normal
      Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
      // Adjust for camera tilt
      Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;

      // Show the groundplane normal
      pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
      pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                        prev_ground_centroid[1] + prev_ground_normal[1],
                        prev_ground_centroid[2] + prev_ground_normal[2]);
      pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                        prev_ground_centroid[1] + tilt_road_normal[1],
                        prev_ground_centroid[2] + tilt_road_normal[2]);

      viewer->removeShape("ground_norm");
      viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
      viewer->removeShape("expected_ground_norm");
      viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");

      viewer->spinOnce(100);
      image_viewer->spinOnce(100);

    }//while

    grabber_.stop();

  }//run

}; //class

int
main(int argc, char** argv)
{

  // kinect setup
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber grabber("#1", depth_mode, image_mode);

  // Process and display
  kinectVisionProcessing kinect_vision_processing(grabber);
  kinect_vision_processing.run_capture();

  return 0;
}
