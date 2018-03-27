/** \brief kinectTraversability is an application for processing point clouds to classify terrain based on traversability estimation using kinect.
  *
  * \author Aras Dargazany
  */


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

////#include <pcl/segmentation/organized_connected_component_segmentation.h>
//#include "projects/icarus/sensor_processing/libsegmentation/organized_connected_component_segmentation.h"
//
////#include <pcl/segmentation/organized_multi_plane_segmentation.h>
//#include "projects/icarus/sensor_processing/libsegmentation/organized_multi_plane_segmentation.h"

#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

//#include <pcl/filters/passthrough.h>
//#include <pcl/sample_consensus/sac_model_plane.h>

using namespace std;
using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;

// templates
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class pointCloudProcessing
{
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_test;
  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_test;

  boost::mutex cloud_mutex;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  Eigen::Vector4f prev_ground_normal;
  Eigen::Vector4f prev_ground_centroid;
  CloudConstPtr prev_ground_image_test; //ransac

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;

  std::vector<std::string> left_images;
  int files_idx;
  int images_idx;
  int frames_number_total;

  bool trigger;
  bool continuous;
  bool display_normals;

public:
  pointCloudProcessing(const std::vector<std::string> left_images, const int frames_number_total):
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
    viewer_test(new pcl::visualization::PCLVisualizer("3D Viewer test"))
    ,
    image_viewer_test(new pcl::visualization::ImageViewer("Image Viewer test")),
    road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
    road_segmentation(road_comparator)
  {
    trigger = true;
    continuous = false;
    display_normals = false;

    //      this->right_images = right_images;
    this->left_images = left_images;
    files_idx = 0;
    images_idx = 0;
    this -> frames_number_total = frames_number_total;

    // Set up a 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer->registerKeyboardCallback(&pointCloudProcessing::keyboardCallback, *this, 0);


    // Set up a 3D viewer
    viewer_test->setBackgroundColor(0, 0, 0);
    viewer_test->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer_test->registerKeyboardCallback(&pointCloudProcessing::keyboardCallback, *this, 0);

    /*Set up the normal estimation based on kinematic capability and the terrain for GroundPlaneComparator*/
    /*
     * COVARIANCE_MATRIX - creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood.
    AVERAGE_3D_GRADIENT - creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients.
    AVERAGE_DEPTH_CHANGE - creates only a single integral image and computes the normals from the average depth changes.
     */
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);

    /*The depth change threshold for computing object borders.
    max_depth_change_factor  the depth change threshold for computing object borders based on depth changes
     * */
    ne.setMaxDepthChangeFactor(0.03f);

    /*normal_smoothing_size factor which influences the size of the area used to smooth normals (depth dependent if useDepthDependentSmoothing is true)
     * */
    ne.setNormalSmoothingSize(40.0f); //20.0f

    /*BORDER_POLICY_IGNORE  BORDER_POLICY_MIRROR*/
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

    /*  decides whether the smoothing is depth dependent*/
    bool use_depth_dependent_smoothing = false;
    ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

    // Set up the ground plane comparator -- If the camera was pointing straight out, the normal would be:
    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)

    /*Adjust for camera tilt for traversability analysis*/
    /*Set the expected ground plane normal with respect to the stereo camera.
     * Pixels labeled as ground must be within ground_angular_threshold radians of this normal to be labeled as ground.
     *     */
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);

    /*slope threshold and Set the tolerance in radians for difference in normal direction between a point and the expected ground normal.*/
    float   ground_angular_threshold = pcl::deg2rad(20.0f);
    road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original


    /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
    road_comparator->setAngularThreshold(angular_threshold); //3.0f original

    /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);
  } // constructor

  ~pointCloudProcessing()
  {
  }

  void
  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.keyUp())
    {
      switch (event.getKeyCode())
      {
      case ' ':
        trigger = true;
        break;
      case 'c':
        continuous = !continuous;
        break;
      case 'n':
        display_normals = !display_normals;
        break;
      }
    }
  }

  void
  processCloud(const CloudConstPtr& cloud)
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


    /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
    /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
    /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
    /* finding dominant segmented plane for more accuracy, speed and almost eliminating the for loop :) */
    std::vector<int> region_indices_size;
    vector<float> dominant_plane_y; //y in pcd which is height
    for (unsigned int i = 0; i < region_indices.size(); i++)
    {
      region_indices_size.push_back(region_indices[i].indices.size());
    }
    std::sort(region_indices_size.begin(), region_indices_size.end());  //from small to big descending order
    unsigned int largest_1 = region_indices_size.at(region_indices_size.size() - 1);

    unsigned int threshold_min_dominant_plane_cloud = (cloud->width / 10) * (cloud->height / 10) * 1.5 * 0.5;  //2500
    cout << "threshold_min_dominant_plane_cloud: " << threshold_min_dominant_plane_cloud << endl;

    /*! traversable regions - green*/
    for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
    {

      /*looking for only the largest region*/
      if (region_indices[i].indices.size() == largest_1 && largest_1 >= threshold_min_dominant_plane_cloud) //100 original //min_inliers - green -- for each segment 1000 original
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

        pcl::ModelCoefficients model;
        model.values.resize(4);
        model.values[0] = plane_params[0];
        model.values[1] = plane_params[1];
        model.values[2] = plane_params[2];
        model.values[3] = plane_params[3];
        model_coefficients.push_back(model);
        inlier_indices.push_back(region_indices[i]);
        centroids.push_back(clust_centroid);
        covariances.push_back(clust_cov);
        region_indices_eigvals.push_back(eigen_value);

        //std::cout << "plane_params A B C D (eigen vector): "  << plane_params << '\n';
        //std::cout << "eigen_value: "  << eigen_value << '\n';
        //std::cout << "dominant segmentaed plane inliers number: " << largest_1 << std::endl;

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

          // getting the max and min y for filtering
          dominant_plane_y.push_back(cloud->points[region_indices[i].indices[j]].y);
        }// for


      } //if region
    }// for i



    /*checking the number of point in the cloud for verification*/
    /*checking the number of point in the cloud for verification*/
    /*checking the number of point in the cloud for verification*/
    /*checking the number of point in the cloud for verification*/
    /*checking the number of point in the cloud for verification*/
    //dominant plane y limit -- height limits
    float dominant_plane_y_max, dominant_plane_y_min;
    bool frame_isGood = true;
    if (ground_cloud->points.size() > 0)
    {
      //std::sort(region_indices_size.begin(), region_indices_size.end());  //from small to big descending order
      sort(dominant_plane_y.begin(), dominant_plane_y.end());
      //int largest_1 = region_indices_size.at(region_indices_size.size() - 1);
      dominant_plane_y_max = dominant_plane_y.at(dominant_plane_y.size() - 1);
      dominant_plane_y_min = dominant_plane_y.at(0);
      cout << "max y: " << dominant_plane_y_max << endl;
      cout << "min y: " << dominant_plane_y_min << endl;
    }// if ground

//    /*filtering*/
//    pcl::PassThrough<PointT> pass;
//    pass.setInputCloud(cloud);
//    //pass.setFilterFieldName("x");
//    //pass.setFilterLimits(-10, 10);
//    pass.setFilterFieldName("y");
//    //pass.setFilterLimits(dominant_plane_y_min - 3, dominant_plane_y_max + 1);
//    pass.setFilterLimits(dominant_plane_y_min, dominant_plane_y_max);
//    //pass.setFilterFieldName("z");
//    //pass.setFilterLimits(0, 50.0);
//    CloudPtr cloud_filtered(new Cloud);
//    pass.filter(*cloud_filtered);
//    //CloudPtr ground_image_test(new Cloud);
//    //pass.filter(*ground_image_test);
//    cout << "size: " << cloud_filtered->points.size() << endl;
//    //cout << "size: " << ground_image_test->points.size() << endl;
//    //cout << "size 2: " << ground_image_test->size() << endl;
//    unsigned int threshold_dominant_plane_cloud = (cloud->width / 10) * (cloud->height / 10) * 1.5 * 10;
//    cout << "threshold dominant_plane_cloud: " << threshold_dominant_plane_cloud << endl;
//    if (cloud_filtered->size() < threshold_dominant_plane_cloud) //5000 =~ 80 * 60 = 4800
//    {
//      cout << "bad point cloud and not relibale ........................................... because cloud size is : " << cloud_filtered->size() << endl;
//      frame_isGood = false;
//
//      //put text on the cloud or images
//      /* add text:*/
//      std::stringstream ss;
//      ss << "BAD FRAME" ;
//      double  textScale = 0.5; //0.3;  //0.4; //0.1; //1.0;
//      double  r = 1.0;
//      double  g = 0.0;
//      double  b = 0.0;
//      //char str[10] = "text_id";
//      //char str_2[10] = "text_id_2";
//      PointT position; //(centroids[0])
//      position.x = -2; //centroids[0][0]; //0;
//      position.y = 0; //centroids[0][1];//0;
//      position.z = 5; //centroids[0][2];//1;
//      viewer->addText3D(ss.str(), position, textScale, r, g, b,  "cloud");
//      //viewer->addText(ss.str(), 400, 300, textScale, r, g, b, "cloud");
//      //viewer_test->addText3D(ss.str(), position, textScale, r, g, b, "cloud_test");
//
//      unsigned int x_min = 0, y_min = 0, x_max = cloud->width, y_max = cloud->height; //image.width=800, image.height=600
//      //bool  addLine (unsigned int x_min, unsigned int y_min, unsigned int x_max, unsigned int y_max, double r, double g, double b, const std::string &layer_id="line", double opacity=1.0)
//      double opacity = 1.0;
//      //const string layer_id;
//      image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
//      //image_viewer_test->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
//      x_min = cloud->width;
//      y_min = 0;
//      x_max = 0;
//      y_max = cloud->height;
//      image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
//      //image_viewer_test->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
//    }



    /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
    /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
    /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
    /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
    Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
    if (ground_cloud->points.size() > 0)
    {
      ground_centroid = centroids[0];
      ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
    }// if


//    /*! other traversable regions in dominant plane limits - green*/
//    /*! other traversable regions in dominant plane limits - green*/
//    /*! other traversable regions in dominant plane limits - green*/
//    /*! other traversable regions in dominant plane limits - green*/
//    if (frame_isGood)
//    {
//      /*! other traversable regions in dominant plane limits - green*/
//      for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
//      {
//
//        /*looking for other the largest region*/
//        if (region_indices[i].indices.size() > threshold_min_dominant_plane_cloud &&
//            region_indices[i].indices.size() <= largest_1 &&
//            region_indices[i].indices.size() > largest_1 / 2) //100 original //min_inliers - green -- for each segment 1000 original
//        {
//
//          /*! Compute plane info*/
//          Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
//          Eigen::Matrix3f clust_cov;
//          pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);
//
//          //cout << "cluster_centroid: " << clust_centroid << endl;
//          cout << "cluster_centroid_y: " << clust_centroid[1] << endl;
//
//
//          pcl::PointXYZ centroid_pt(clust_centroid[0], clust_centroid[1], clust_centroid[2]);
//          //double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//          double ptp_dist =  pcl::pointToPlaneDistance(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//          //if (plane_params[3] < dominant_plane_y_max && plane_params[3] > dominant_plane_y_min)
//          if (clust_centroid[1] <= dominant_plane_y_max && clust_centroid[1] >= dominant_plane_y_min && ptp_dist < 1)
//          {
//
//            cout << "cluster_centroid meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
//
//            /*visualizing the result by colorizing the point cloud */ /*! traversable regions - green*/
//            for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
//            {
//              ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
//              label_image->points[region_indices[i].indices[j]].r = 0;
//              label_image->points[region_indices[i].indices[j]].g = 255 ;
//              label_image->points[region_indices[i].indices[j]].b = 0;
//              pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
//                                      cloud->points[region_indices[i].indices[j]].y,
//                                      cloud->points[region_indices[i].indices[j]].z);
//              ground_cloud->points.push_back(ground_pt);
//
//            }// for
//          }//if
//          else
//          {
//            cout << "cluster_centroid NOT meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
//          }//else
//
//
//        } //if region
//      }// for i
//    }// frame


    /*plane refinement by mps*/
    /*plane refinement by mps*/
    /*plane refinement by mps*/
    /*plane refinement by mps*/
    /*plane refinement by mps*/
    if (frame_isGood && ground_cloud->points.size() > 0)
    {

      pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare(new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
      refinement_compare->setInputCloud(cloud);
      refinement_compare->setInputNormals(normal_cloud);
      float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
      bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
      refinement_compare->setDistanceThreshold(distance_threshold, depth_dependent);
      refinement_compare->setModelCoefficients(model_coefficients);

      /*the angle between the normals is supposedly pretty noisy that s using refineCOmparator */
      //float angular_threshold = pcl::deg2rad(20.0f); //the tolerance in radians
      //refinement_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle capability //these are noisy regions for normals

      std::vector<bool> grow_labels;
      std::vector<int> label_to_model;
      grow_labels.resize(region_indices.size(), false);
      label_to_model.resize(region_indices.size(), 0);
      //vector<float> model_coefficients_float; //PlaneCoefficientComparator

      for (size_t i = 0; i < model_coefficients.size(); i++) //this is only the biggest plane
      {
        int model_label = (labels)[inlier_indices[i].indices[0]].label;
        label_to_model[model_label] = static_cast<int>(i);
        grow_labels[model_label] = true;
        //model_coefficients_float.push_back(model_coefficients[i].values[3]); //PlaneCoefficientComparator
      }

      refinement_compare->setRefineLabels(grow_labels);
      refinement_compare->setLabelToModel(label_to_model);

      boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
      *labels_ptr = labels;
      refinement_compare->setLabels(labels_ptr);

      /* organized multi plane segmentation demo using different comparators*/
      OrganizedMultiPlaneSegmentation<PointT, Normal, Label> mps;
      mps.setInputCloud(cloud);

      /*Provide a pointer to the vector of indices that represents the input data.*/
      //mps_test.setIndices(const IndicesPtr &  indices); //not needed since we have the input

      mps.setInputNormals(normal_cloud);

      /*Set the minimum number of inliers required for a plane.  * image is 800* 600  = width*height */
      unsigned min_inliers = 800; //500 first time //this shows how stable and reliable a plane is //max w and height of the image
      mps.setMinInliers(min_inliers);

      float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
      mps.setAngularThreshold(angular_threshold);

      /*this is only for detecting segmenting plane without using comparator*/
      float distance_threshold_mps = 0.1f;  //the tolerance in meters (at 1m)
      mps.setDistanceThreshold(distance_threshold_mps);

      /*Set the maximum curvature allowed for a planar region.* The tolerance for maximum curvature after fitting a plane.
      Used to remove smooth, but non-planar regions.*/
      double maximum_curvature = 0.01;
      mps.setMaximumCurvature(maximum_curvature);  // a small curvature

      /*Set whether or not to project boundary points to the plane, or leave them in the original 3D space.*/
      bool project_points = true;
      mps.setProjectPoints(project_points);

      /*Provide a pointer to the comparator to be used for refinement.*/
      mps.setRefinementComparator(refinement_compare);

      /*Perform a refinement of an initial segmentation, by comparing points to adjacent regions detected by the initial segmentation.*/
      mps.refine(model_coefficients,
                 inlier_indices,
                 centroids,
                 covariances,
                 labels_ptr,
                 region_indices); //not the region indices

      /*! Draw the segmentation result testtttttt*/
      //CloudPtr ground_image_test(new Cloud);
      //*ground_image_test = *cloud;
      vector<PointIndices> region_indices_test(inlier_indices); //region_indices

      //Note the regions that have been extended
      for (unsigned int i = 0; i < region_indices_test.size(); i++)
      {
        if (region_indices_test[i].indices.size() >= 1000)
        {
          for (unsigned int j = 0; j < region_indices_test[i].indices.size(); j++)
          {
            if (label_image->points[region_indices_test[i].indices[j]].g != 255)  //region_indices_test[i].indices.size() >= 1000
            {
              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
              //ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
              ground_image->points[region_indices_test[i].indices[j]].r = 255;
              ground_image->points[region_indices_test[i].indices[j]].g = 255;
            }//if
          }//for
        }//if
      }// for

    }//if for plane refinement


///////////////////////////////////////////////////////////////////////test




///////////////////////////////////////////////////////////////////////////test

    // note the NAN points in the image as well
    // note the NAN points in the image as well
    // note the NAN points in the image as well
    // note the NAN points in the image as well
    // note the NAN points in the image as well
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
      prev_ground_image_test = ground_image;  //testttttttttttttt
      prev_label_image = label_image;
      prev_ground_normal = ground_plane_params;
      prev_ground_centroid = ground_centroid;
      cloud_mutex.unlock();
    }
  }

  void run()
  {

    while (!viewer->wasStopped())
    {

      if (frames_number_total == images_idx)
      {
        cout << "frames_number_total == images_idx -> " << images_idx << std::endl;
        break;
      } // if

      // Process a new image
      if (trigger || continuous)
      {
        // showing the input images
        cout << "images_idx: " << images_idx << endl;
        cout << "frames_number_total: " << frames_number_total << endl;

        CloudPtr left_cloud(new Cloud);
        pcl::PCDReader pcd;
        pcd.read(left_images[images_idx], *left_cloud);
        viewer->removeText3D("cloud");
        viewer_test->removeText3D("cloud_test");
        image_viewer->removeLayer("line");  //const std::string &layer_id
        image_viewer_test->removeLayer("line");
        processCloud(left_cloud);

        trigger = false;
        images_idx ++;
      }

      /*Draw visualizations*/
      //if (cloud_mutex.try_lock())  //undefined ref to pthread with new gcc 4.8
      //{         cloud_mutex.unlock();      }//if

      if (!viewer->updatePointCloud(prev_ground_image, "cloud")) //&& !viewer_test->updatePointCloud(prev_ground_image_test, "cloud_test")
      {

        viewer->addPointCloud(prev_ground_image, "cloud");
        //viewer_test->addPointCloud(prev_ground_image_test, "cloud_test");

      }// if

      if (!viewer_test->updatePointCloud(prev_ground_image_test, "cloud_test"))
      {
        viewer_test->addPointCloud(prev_ground_image_test, "cloud_test");
      }// if


      if (prev_cloud->points.size() > 1000)
      {
        image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
        image_viewer_test->addRGBImage<PointT>(prev_ground_image_test, "rgb_image_test", 0.3);
      }// if

      // showing normals on point clouds
      if (prev_normal_cloud->points.size() > 1000 && display_normals)
      {
        viewer->removePointCloud("normals");
        viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
      }

      /*comparing the normals of real ground and expected ground plane for normal*/
      //if (prev_ground_cloud->points.size() > 0) {        }// if

      // Show the ground plane normal
      Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);

      // Adjust for camera tilt
      Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;

      // Show the ground plane normal
      pcl::PointXYZ np1(prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
      pcl::PointXYZ np2(prev_ground_centroid[0] + prev_ground_normal[0],
                        prev_ground_centroid[1] + prev_ground_normal[1],
                        prev_ground_centroid[2] + prev_ground_normal[2]);
      pcl::PointXYZ np3(prev_ground_centroid[0] + tilt_road_normal[0],
                        prev_ground_centroid[1] + tilt_road_normal[1],
                        prev_ground_centroid[2] + tilt_road_normal[2]);

      // comparing the normals of real ground and expected ground plane for normal
      viewer->removeShape("ground_norm");
      viewer->addArrow(np2, np1, 1.0, 0, 0, false, "ground_norm");
      viewer->removeShape("expected_ground_norm");
      viewer->addArrow(np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");

      //prev_ground_cloud->removeAllPointClouds();
      //prev_ground_cloud->clear();


      viewer->spinOnce(100);
      image_viewer->spinOnce(100);
      viewer_test->spinOnce(100);
      image_viewer_test->spinOnce(100);

    }//while

  }//run

}; //class

int
main(int argc, char** argv)
{

  if (argc < 2)
  {
    PCL_INFO("usage: aras_icarus_sensorProcessing_pointCloudProcessing pcd_frames_directory \n");
    PCL_INFO("note: frames must be in PCD format. \n");
    PCL_INFO("for example : \n"
             "aras_icarus_sensorProcessing_pointCloudProcessing ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/acso/frames_pcd_left\n "
             "aras_icarus_sensorProcessing_pointCloudProcessing  ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/seq100/acso_test/frames_pcd_left \n");
    return -1;

  }

  // variable initial
  int img_number_left = 0;

  // Get list of stereo files
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());
  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  // showing the input images
  cout <<  "frames_number: " << img_number_left << endl;

  // Process and display
  pointCloudProcessing point_cloud_processing(left_images , img_number_left);
  point_cloud_processing.run();

  return 0;
}
