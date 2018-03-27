  /** \brief StereoGroundSegmentation is a demonstration application for using PCL's stereo tools and segmentation tools to detect smooth surfaces suitable for driving.
    *
    * \author Alex Trevor
    * \author Federico Tombari
    * \author Aras Dargazany
    */

// system files
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/stereo/stereo_grabber.h> //not used //???
#include <pcl/stereo/stereo_matching.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

// debugging
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <vector>

// namespace
using namespace pcl;
using namespace std;
using namespace cv;

// template
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class stereoVisionProcessing
{
  private:
    boost::shared_ptr<visualization::PCLVisualizer> viewer;
    boost::shared_ptr<visualization::ImageViewer> image_viewer;
    boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
    boost::mutex cloud_mutex;
    CloudConstPtr prev_cloud;
    CloudConstPtr prev_ground_image;
    CloudConstPtr prev_label_image;
    pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
    Eigen::Vector4f prev_ground_normal;
    Eigen::Vector4f prev_ground_centroid;
    std::vector<std::string> left_images;
    std::vector<std::string> right_images;
    int images_idx;
    int img_pairs_num;
	// arguments 3 and 4
	string input_intrinsic_filename;
	string input_extrinsic_filename ;
	Mat frame_0;
	Mat frame_1;


    pcl::AdaptiveCostSOStereoMatching stereo;
    bool trigger;
    bool continuous;
    bool display_normals;
    bool detect_obstacles;
    int smooth_weak;
    int smooth_strong;

  public:
    stereoVisionProcessing (const std::vector<std::string> left_images, const std::vector<std::string> right_images,
    		const int img_pairs_num ,
    		const string input_intrinsic_filename, const string input_extrinsic_filename ) :
      viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")),
      image_viewer (new pcl::visualization::ImageViewer ("Image Viewer")),
      image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
      prev_cloud (new Cloud),
      prev_normal_cloud (new pcl::PointCloud<pcl::Normal>),
      prev_ground_cloud (new pcl::PointCloud<pcl::PointXYZ>),
      prev_ground_image (new Cloud),
      prev_label_image (new Cloud),
      road_comparator (new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
      road_segmentation (road_comparator)
    {
      trigger = true;
      continuous = false;
      display_normals = false;
      detect_obstacles = false;

      this->left_images = left_images;
      this->right_images = right_images;
      images_idx = 0;
      this->img_pairs_num = img_pairs_num;
      this->input_intrinsic_filename = input_intrinsic_filename;
      this->input_extrinsic_filename = input_extrinsic_filename;

      // Set up a 3D viewer
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      viewer->registerKeyboardCallback (&stereoVisionProcessing::keyboardCallback, *this, 0);
      
      // Set up the stereo matching
      stereo.setMaxDisparity(60);
      stereo.setXOffset(0);
      stereo.setRadius(5);
      
      smooth_weak = 20;
      smooth_strong = 100;
      stereo.setSmoothWeak(smooth_weak);
      stereo.setSmoothStrong(smooth_strong);
      stereo.setGammaC(25);
      stereo.setGammaS(10);
      
      stereo.setRatioFilter(20);
      stereo.setPeakFilter(0);
      
      stereo.setLeftRightCheck(true);
      stereo.setLeftRightCheckThreshold(1);
      
      stereo.setPreProcessing(true);
      
      // Set up the normal estimation
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor (0.03f);
      ne.setNormalSmoothingSize (40.0f);//20.0f

      // Set up the groundplane comparator
      // If the camera was pointing straight out, the normal would be:
      Eigen::Vector3f nominal_road_normal (0.0, -1.0, 0.0);
      // Adjust for camera tilt:
      Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf (pcl::deg2rad (5.0f), Eigen::Vector3f::UnitX ()) * nominal_road_normal;
      road_comparator->setExpectedGroundNormal (tilt_road_normal);
      road_comparator->setGroundAngularThreshold (pcl::deg2rad (10.0f));   
      road_comparator->setAngularThreshold (pcl::deg2rad (3.0f));
    } // constructor
    
    ~stereoVisionProcessing ()
    {
    }
    
    void
    cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!viewer->wasStopped ())
      {
        cloud_mutex.lock ();
        prev_cloud = cloud;
        cloud_mutex.unlock ();
      }
    }

    void
    keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.keyUp ())
      {
        switch (event.getKeyCode ())
        {
          case ' ':
            trigger = true;
            break;
          case '1':
            smooth_strong -= 10;
            PCL_INFO ("smooth_strong: %d\n", smooth_strong);
            stereo.setSmoothStrong (smooth_strong);
            break;
          case '2':
            smooth_strong += 10;
            PCL_INFO ("smooth_strong: %d\n", smooth_strong);
            stereo.setSmoothStrong (smooth_strong);
            break;
          case '3':
            smooth_weak -= 10;
            PCL_INFO ("smooth_weak: %d\n", smooth_weak);
            stereo.setSmoothWeak (smooth_weak);
            break;
          case '4':
            smooth_weak += 10;
            PCL_INFO ("smooth_weak: %d\n", smooth_weak);
            stereo.setSmoothWeak (smooth_weak);
            break;
          case 'c':
            continuous = !continuous;
            break;
          case 'n':
            display_normals = !display_normals;
            break;
          case 'o':
            detect_obstacles = !detect_obstacles;
            break;
        }
      }
    }
    
	int stereo_capture()
	{

		VideoCapture capture;
		VideoCapture capture_1;

		// camera index 0
		capture.open(0);  //right
		capture_1.open(1);

		// RIGHT CAMERA_CAMERAINDEX=0
		Mat frame_found_0;
		if( !capture.grab() ){
			 cout << "Can not grab images." << endl;
			 return -1;
		 }// if
		capture.retrieve(frame_found_0);
		frame_found_0.copyTo(frame_0);

		//////////left camera camera index=1
		Mat frame_found_1;
		if( !capture_1.grab() ){
		 cout << "Can not grab images." << endl;
		 return -1;
		 } // if
		capture_1.retrieve(frame_found_1);
		frame_found_1.copyTo(frame_1);

		return 0;
	} // stereo_capture

	int stereo_rectify ()
	{

			//variable initialization
			const char* intrinsic_filename = (char*) input_intrinsic_filename.c_str();
			const char* extrinsic_filename = (char*) input_extrinsic_filename.c_str();

			// reading png images
			Mat img1, img2;
			frame_0.copyTo(img1);
			frame_1.copyTo(img2);

			float scale = 1.f;
			if( scale != 1.f )
			{
				Mat temp1, temp2;
				int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
				resize(img1, temp1, Size(), scale, scale, method);
				img1 = temp1;
				resize(img2, temp2, Size(), scale, scale, method);
				img2 = temp2;
			}


			Size img_size = img1.size();
			Rect roi1, roi2;
			Mat Q;

			// rectification module
			if( intrinsic_filename )
			{
				// reading intrinsic parameters
				FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
				if(!fs.isOpened())
				{
					printf("Failed to open file %s\n", intrinsic_filename);
					return -1;
				}

				Mat M1, D1, M2, D2;
				fs["M1"] >> M1;
				fs["D1"] >> D1;
				fs["M2"] >> M2;
				fs["D2"] >> D2;

				fs.open(extrinsic_filename, CV_STORAGE_READ);
				if(!fs.isOpened())
				{
					printf("Failed to open file %s\n", extrinsic_filename);
					return -1;
				}

				Mat R, T, R1, P1, R2, P2;
				fs["R"] >> R;
				fs["T"] >> T;

				stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

				Mat map11, map12, map21, map22;
				initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
				initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

				Mat img1r, img2r;
				remap(img1, img1r, map11, map12, INTER_LINEAR);
				remap(img2, img2r, map21, map22, INTER_LINEAR);

				img1 = img1r;
				img2 = img2r;
			}

			//display the results
			bool no_display = false;
			if( !no_display )
			{
				namedWindow("left", 1);
				imshow("left", img1);
				namedWindow("right", 1);
				imshow("right", img2);
				printf("press any key to continue...");
				fflush(stdout);
				waitKey(500);
				printf("\n");
			}

		return 0;
	}// stereo_rectify

    PointCloud<RGB>::Ptr png2pcd (const string input_png_images)
    {

    	// showing the input images
		cout << "png2pcd---------------input_png_images: " << input_png_images << std::endl;

		//variable initialization
		const char* img1_filename = (char*) input_png_images.c_str();

		// Command line parsing
		bool format = false;
		std::string mode = "FORCE_COLOR";

		// Load the input file
		vtkSmartPointer<vtkImageData> image_data;
		vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
		reader->SetFileName (img1_filename);
		image_data = reader->GetOutput ();
		image_data->Update ();

		// Retrieve the entries from the image data and copy them into the output RGB cloud
		int components = image_data->GetNumberOfScalarComponents();
		double* pixel = new double [4];
		memset (pixel, 0, sizeof(double) * 4);
		PointCloud<RGB> cloud;

		int dimensions[3];
		image_data->GetDimensions (dimensions);
		cloud.width = dimensions[0];
		cloud.height = dimensions[1]; // This indicates that the point cloud is organized
		cloud.is_dense = true;
		cloud.points.resize (cloud.width * cloud.height);

	  for (int y = 0; y < dimensions[1]; y++)
	  {
		for (int x = 0; x < dimensions[0]; x++)
		{
		  for (int c = 0; c < components; c++)
			pixel[c] = image_data->GetScalarComponentAsDouble(x, y, 0, c);

		  RGB color;
		  color.r = 0;
		  color.g = 0;
		  color.b = 0;
		  color.a = 0;
		  color.rgb = 0.0f;
		  color.rgba = 0;

		  int rgb;
		  int rgba;
		  switch (components)
		  {
			case 1:  color.r = static_cast<uint8_t> (pixel[0]);
			color.g = static_cast<uint8_t> (pixel[0]);
			color.b = static_cast<uint8_t> (pixel[0]);

			rgb = (static_cast<int> (color.r)) << 16 |
				(static_cast<int> (color.g)) << 8 |
				(static_cast<int> (color.b));

			rgba = rgb;
			color.rgb = static_cast<float> (rgb);
			color.rgba = static_cast<uint32_t> (rgba);
			break;

			case 3:  color.r = static_cast<uint8_t> (pixel[0]);
			color.g = static_cast<uint8_t> (pixel[1]);
			color.b = static_cast<uint8_t> (pixel[2]);

			rgb = (static_cast<int> (color.r)) << 16 |
				(static_cast<int> (color.g)) << 8 |
				(static_cast<int> (color.b));

			rgba = rgb;
			color.rgb = static_cast<float> (rgb);
			color.rgba = static_cast<uint32_t> (rgba);
			break;

			case 4:  color.r = static_cast<uint8_t> (pixel[0]);
			color.g = static_cast<uint8_t> (pixel[1]);
			color.b = static_cast<uint8_t> (pixel[2]);
			color.a = static_cast<uint8_t> (pixel[3]);

			rgb = (static_cast<int> (color.r)) << 16 |
				(static_cast<int> (color.g)) << 8 |
				(static_cast<int> (color.b));
			rgba = (static_cast<int> (color.a)) << 24 |
				(static_cast<int> (color.r)) << 16 |
				(static_cast<int> (color.g)) << 8 |
				(static_cast<int> (color.b));

			color.rgb = static_cast<float> (rgb);
			color.rgba = static_cast<uint32_t> (rgba);
			break;
		  }//switch

		  // Point<RGB> (x,y) = RGB
		  cloud (x, dimensions[1] - y - 1) = color;

		}//for
	  }//for

	  delete[] pixel;

	  // conversion from PointCloud<RGB> to PointCloud<RGB>::Ptr
	  PointCloud<RGB>::Ptr cloud_output (new PointCloud<RGB>);
	  *cloud_output = cloud;

//      return (PointCloud<RGB>::Ptr&)cloud; //not
	  return cloud_output;
    }// png2pcd

    void
    processStereoPair (const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
    {
      stereo.compute (*left_image, *right_image);
      stereo.medianFilter (4);
      stereo.getPointCloud(318.112200f, 224.334900f, 368.534700f, 0.8387445f, out_cloud, left_image);
    }

    void
    processCloud (const pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      // Compute the normals
      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
      ne.setInputCloud (cloud);
      ne.compute (*normal_cloud);

      // Set up the groundplane comparator
      road_comparator->setInputCloud (cloud);
      road_comparator->setInputNormals (normal_cloud);

      // Run segmentation
      pcl::PointCloud<pcl::Label> labels;
      std::vector<pcl::PointIndices> region_indices;
      road_segmentation.setInputCloud (cloud);
      road_segmentation.segment (labels, region_indices);

      // Draw the segmentation result
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_image (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image (new pcl::PointCloud<pcl::PointXYZRGB>);
      *ground_image = *cloud;
      *label_image = *cloud;

      Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
      Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
      Eigen::Matrix3f clust_cov;
      pcl::ModelCoefficients model;
      model.values.resize (4);

      std::vector<pcl::ModelCoefficients> model_coefficients;
      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
      std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
      std::vector<pcl::PointIndices> inlier_indices;

      for (int i = 0; i < region_indices.size (); i++)
      {
        if (region_indices[i].indices.size () > 1000)
        {

          for (int j = 0; j < region_indices[i].indices.size (); j++)
          {  
            pcl::PointXYZ ground_pt (cloud->points[region_indices[i].indices[j]].x,
                                     cloud->points[region_indices[i].indices[j]].y,
                                     cloud->points[region_indices[i].indices[j]].z);
            ground_cloud->points.push_back (ground_pt);
            ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t> ((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
            label_image->points[region_indices[i].indices[j]].r = 0;
            label_image->points[region_indices[i].indices[j]].g = 255;
            label_image->points[region_indices[i].indices[j]].b = 0;
          } 
          
          // Compute plane info
          pcl::computeMeanAndCovarianceMatrix (*cloud, region_indices[i].indices, clust_cov, clust_centroid);
          Eigen::Vector4f plane_params;
          
          EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
          EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
          pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
          plane_params[0] = eigen_vector[0];
          plane_params[1] = eigen_vector[1];
          plane_params[2] = eigen_vector[2];
          plane_params[3] = 0;
          plane_params[3] = -1 * plane_params.dot (clust_centroid);
          
          vp -= clust_centroid;
          float cos_theta = vp.dot (plane_params);
          if (cos_theta < 0)
          {
            plane_params *= -1;
            plane_params[3] = 0;
            plane_params[3] = -1 * plane_params.dot (clust_centroid);
          }
          
          model.values[0] = plane_params[0];
          model.values[1] = plane_params[1];
          model.values[2] = plane_params[2];
          model.values[3] = plane_params[3];
          model_coefficients.push_back (model);
          inlier_indices.push_back (region_indices[i]);
          centroids.push_back (clust_centroid);
          covariances.push_back (clust_cov);
        }
      }
      
      //Refinement
      std::vector<bool> grow_labels;
      std::vector<int> label_to_model;
      grow_labels.resize (region_indices.size (), false);
      label_to_model.resize (region_indices.size (), 0);
      
      for (size_t i = 0; i < model_coefficients.size (); i++)
      {
        int model_label = (labels)[inlier_indices[i].indices[0]].label;
        label_to_model[model_label] = static_cast<int> (i);
        grow_labels[model_label] = true;
      }
      
      boost::shared_ptr<pcl::PointCloud<pcl::Label> > labels_ptr (new pcl::PointCloud<pcl::Label>());
      *labels_ptr = labels;
      pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
      pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr refinement_compare (new pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>());
      refinement_compare->setInputCloud (cloud);
      refinement_compare->setDistanceThreshold (0.15f);
      refinement_compare->setLabels (labels_ptr);
      refinement_compare->setModelCoefficients (model_coefficients);
      refinement_compare->setRefineLabels (grow_labels);
      refinement_compare->setLabelToModel (label_to_model);
      mps.setRefinementComparator (refinement_compare);
      mps.setMinInliers (500);
      mps.setAngularThreshold (pcl::deg2rad (3.0));
      mps.setDistanceThreshold (0.02);
      mps.setInputCloud (cloud);
      mps.setInputNormals (normal_cloud);
      mps.refine (model_coefficients,
                  inlier_indices,
                  centroids,
                  covariances,
                  labels_ptr,
                  region_indices);
      
      //Note the regions that have been extended
      pcl::PointCloud<PointT> extended_ground_cloud;
      for (int i = 0; i < region_indices.size (); i++)
      {
        if (region_indices[i].indices.size () > 1000)
        {
          for (int j = 0; j < region_indices[i].indices.size (); j++)
          {
            // Check to see if it has already been labeled
            if (ground_image->points[region_indices[i].indices[j]].g == ground_image->points[region_indices[i].indices[j]].b)
            {
              pcl::PointXYZ ground_pt (cloud->points[region_indices[i].indices[j]].x,
                                       cloud->points[region_indices[i].indices[j]].y,
                                       cloud->points[region_indices[i].indices[j]].z);
              ground_cloud->points.push_back (ground_pt);
              ground_image->points[region_indices[i].indices[j]].r = static_cast<uint8_t> ((cloud->points[region_indices[i].indices[j]].r + 255) / 2);
              ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t> ((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
              label_image->points[region_indices[i].indices[j]].r = 128;
              label_image->points[region_indices[i].indices[j]].g = 128;
              label_image->points[region_indices[i].indices[j]].b = 0;
            }
            
          }
        }
      }

      // Segment Obstacles (Disabled by default)
      Eigen::Vector4f ground_plane_params (1.0, 0.0, 0.0, 1.0);
      Eigen::Vector4f ground_centroid (0.0, 0.0, 0.0, 0.0);
      
      if (ground_cloud->points.size () > 0)
      {
        ground_centroid = centroids[0];
        ground_plane_params = Eigen::Vector4f (model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
      }

      if (detect_obstacles)
      {
        pcl::PointCloud<PointT>::CloudVectorType clusters;
        if (ground_cloud->points.size () > 0)
        {
          std::vector<bool> plane_labels;
          plane_labels.resize (region_indices.size (), false);
          for (size_t i = 0; i < region_indices.size (); i++)
          {
            if (region_indices[i].indices.size () > mps.getMinInliers ())
            {
              plane_labels[i] = true;
            }
          }
        
          pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_ (new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

          euclidean_cluster_comparator_->setInputCloud (cloud);
          euclidean_cluster_comparator_->setLabels (labels_ptr);
          euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
          euclidean_cluster_comparator_->setDistanceThreshold (0.05f, false);
        
          pcl::PointCloud<pcl::Label> euclidean_labels;
          std::vector<pcl::PointIndices> euclidean_label_indices;
          pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
          euclidean_segmentation.setInputCloud (cloud);
          euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
        
          for (size_t i = 0; i < euclidean_label_indices.size (); i++)
          {
            if ((euclidean_label_indices[i].indices.size () > 200))
            {
              pcl::PointCloud<PointT> cluster;
              pcl::copyPointCloud (*cloud, euclidean_label_indices[i].indices, cluster);
              clusters.push_back (cluster);

              Eigen::Vector4f cluster_centroid;
              Eigen::Matrix3f cluster_cov;
              pcl::computeMeanAndCovarianceMatrix (*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);

              pcl::PointXYZ centroid_pt (cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
              double ptp_dist =  pcl::pointToPlaneDistanceSigned (centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

              if ((ptp_dist > 0.5) && (ptp_dist < 3.0))
              {
              
                for (int j = 0; j < euclidean_label_indices[i].indices.size (); j++)
                {
                  ground_image->points[euclidean_label_indices[i].indices[j]].r = 255;
                  label_image->points[euclidean_label_indices[i].indices[j]].r = 255;
                  label_image->points[euclidean_label_indices[i].indices[j]].g = 0;
                  label_image->points[euclidean_label_indices[i].indices[j]].b = 0;
                }

              }
            
            }    
          }
        
        }
      }

      // note the NAN points in the image as well
      for (int i = 0; i < cloud->points.size (); i++)
      {
        if (!pcl::isFinite (cloud->points[i]))
        {
          ground_image->points[i].b = static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
          label_image->points[i].r = 0;
          label_image->points[i].g = 0;
          label_image->points[i].b = 255;
        }
      }

      // Update info for the visualization thread
      {
        cloud_mutex.lock ();
        prev_cloud = cloud;
        prev_normal_cloud = normal_cloud;
        prev_ground_cloud = ground_cloud;
        prev_ground_image = ground_image;
        prev_label_image = label_image;
        prev_ground_normal = ground_plane_params;
        prev_ground_centroid = ground_centroid;
        cloud_mutex.unlock ();
      }
    }

    void
    run ()
    {
      while (!viewer->wasStopped ())
      {

  		// exiting criteria
  		if (img_pairs_num == images_idx){
  			cout << "img_pairs_num == images_idx " << images_idx << std::endl;
  			break;
  		} //if

        //stereo capture
        // stereo_capture (left_images_folder, right_images_folder); // to save images
        stereo_capture ();

        //stereo rectify
//        stereo_rectify (left_images[images_idx], right_images[images_idx]);
        stereo_rectify ();

  		// Process a new image
        if (trigger || continuous)
        {

          pcl::PointCloud<pcl::RGB>::Ptr left_cloud (new pcl::PointCloud<pcl::RGB>);
          pcl::PointCloud<pcl::RGB>::Ptr right_cloud (new pcl::PointCloud<pcl::RGB>);
          CloudPtr out_cloud (new Cloud);

          //png2pcd -- reading png images and converting to pcd
          left_cloud = png2pcd (left_images[images_idx]);
          right_cloud = png2pcd (right_images[images_idx]);

          if ( left_cloud->height > 1 && right_cloud->height > 1){

              // process the disparity map and point cloud
              processStereoPair (left_cloud, right_cloud, out_cloud); //used with png2pcd()

              // segmentation on output cloud
              processCloud (out_cloud);

              ///displaying disparity map on output cloud
         	  pcl::PointCloud<pcl::RGB>::Ptr vmap (new pcl::PointCloud<pcl::RGB>);
         	  stereo.getVisualMap(vmap);
         	  image_viewer_disparity->addRGBImage<pcl::RGB> (vmap);

          }// if

          images_idx++;
          trigger = false;

        }// if trigger
        
		// showing the input images
		cout << "left_images[img_index]: " << left_images[images_idx-1] << std::endl;
		cout << "right_images[img_index]: " << right_images[images_idx-1] << std::endl;
		cout << "images_idx: " << images_idx-1 << endl;
		cout << "img_pairs_num: " << img_pairs_num << endl;

//        // Draw visualizations
//        if (cloud_mutex.try_lock ())
//        {
//          if (!viewer->updatePointCloud (prev_ground_image, "cloud"))
//        	  viewer->addPointCloud (prev_ground_image, "cloud");
//
//          if (prev_normal_cloud->points.size () > 1000 && display_normals)
//          {
//            viewer->removePointCloud ("normals");
//            viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
//            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
//          }
//
//          if (prev_cloud->points.size () > 1000)
//          {
//            image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
//          }
//
//          // Show the groundplane normal
//          Eigen::Vector3f nominal_road_normal (0.0, -1.0, 0.0);
//
//          // Adjust for camera tilt
//          Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf (pcl::deg2rad (5.0f), Eigen::Vector3f::UnitX ()) * nominal_road_normal;
//
//          // Show the groundplane normal
//          pcl::PointXYZ np1 (prev_ground_centroid[0], prev_ground_centroid[1], prev_ground_centroid[2]);
//          pcl::PointXYZ np2 (prev_ground_centroid[0] + prev_ground_normal[0],
//                             prev_ground_centroid[1] + prev_ground_normal[1],
//                             prev_ground_centroid[2] + prev_ground_normal[2]);
//          pcl::PointXYZ np3 (prev_ground_centroid[0] + tilt_road_normal[0],
//                             prev_ground_centroid[1] + tilt_road_normal[1],
//                             prev_ground_centroid[2] + tilt_road_normal[2]);
//
//          viewer->removeShape ("ground_norm");
//          viewer->addArrow (np2, np1, 1.0, 0, 0, false, "ground_norm");
//          viewer->removeShape ("expected_ground_norm");
//          viewer->addArrow (np3, np1, 0.0, 1.0, 0, false, "expected_ground_norm");
//
//          cloud_mutex.unlock ();
//        } //if cloud_mutex

        viewer->spinOnce (1);
        image_viewer->spinOnce (1);
        image_viewer_disparity->spinOnce (1);
        
      } // while
    }// run

};//class

int
main (int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO ("usage: pcl_stereo_ground_segmentation left_png_image_directory \n");
    PCL_INFO ("note: images in both left and right folders must be in PNG format.\n");
  }

	int img_number_left = 0, img_number_right = 0 ;
	int img_pairs_num = 0;

  // Get list of stereo files
	// reading left images from folder
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back (itr->path ().string ());
	img_number_left++;
  }
  sort (left_images.begin (), left_images.end ());

  // reading right images from folder
  std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr (argv[2]); itr != end_itr; ++itr)
  {
    right_images.push_back (itr->path ().string ());
	img_number_right++;
  }
  sort (right_images.begin (), right_images.end ());

  PCL_INFO ("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  // showing the input images
  cout << "img_number_left: " << img_number_left << std::endl;
  cout << "img_number_right: " << img_number_right << std::endl;
  if (img_number_left == img_number_right)
	img_pairs_num = img_number_left;

  // calibration parameters for rectification
  string input_intrinsic_filename = argv[3];
  string input_extrinsic_filename = argv[4];

  // Process and display
  stereoVisionProcessing stereo_vision_processing (left_images, right_images, img_pairs_num, input_intrinsic_filename, input_extrinsic_filename);
  stereo_vision_processing.run ();
  
  return 0;
}// main
