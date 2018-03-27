  /** \brief StereoVisioProcessing is an application for processing stereo images to estimate terrain traversability using stereo camera to detect traversable and drivable surfaces.
    *
    * \author Aras Dargazany
    */

// system files
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_grabber.h>
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

// debugging
#include <iostream>
#include <vector>
#include <stdio.h>

// using namespace
using namespace std;
using namespace pcl;
using namespace cv; //opencv
using namespace boost;
using namespace Eigen;

// template
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class StereoVisionProcessing
{
  private:
    boost::shared_ptr<visualization::PCLVisualizer> viewer;
    boost::shared_ptr<visualization::ImageViewer> image_viewer;
    boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
    mutex cloud_mutex;
    CloudConstPtr prev_cloud;
    CloudConstPtr prev_ground_image;
    CloudConstPtr prev_label_image;
    pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation;
    Eigen::Vector4f prev_ground_normal;
    Vector4f prev_ground_centroid;
    string left_images;
    string right_images;
	// stereo
    pcl::AdaptiveCostSOStereoMatching stereo;
    bool trigger;
    bool continuous;
    bool display_normals;
    bool detect_obstacles;
    int smooth_weak;
    int smooth_strong;
    // stereo capture, rectification initialization
	Mat frame_0;
	Mat frame_1;
	string input_intrinsic_filename;
	string input_extrinsic_filename ;
	Mat left_rect_png_img;
	Mat right_rect_png_img;

  public:
	StereoVisionProcessing (const string left_images, const string right_images,
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
      this->input_intrinsic_filename = input_intrinsic_filename;
      this->input_extrinsic_filename = input_extrinsic_filename;
      
      // Set up a 3D viewer
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      viewer->registerKeyboardCallback (&StereoVisionProcessing::keyboardCallback, *this, 0);
      
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
    
    ~StereoVisionProcessing ()
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
    
	void
	stereo_capture( const int frame_number)
	{

		VideoCapture capture;
		VideoCapture capture_1;

		// camera index 0
		capture.open(0);  //right
		capture_1.open(1);

		// CAMERA_CAMERAINDEX=0
		Mat frame_found_0;
		if( !capture.grab() ){
			 cout << "Can not grab images." << endl;
		 }// if
		capture.retrieve(frame_found_0);
		frame_found_0.copyTo(frame_0);

		//////////camera camera index=1
		Mat frame_found_1;
		if( !capture_1.grab() ){
		 cout << "Can not grab images." << endl;
		 } // if
		capture_1.retrieve(frame_found_1);
		frame_found_1.copyTo(frame_1);

		//display the results
		namedWindow("frame_0", 0);
		imshow("frame_0", frame_0);
		namedWindow("frame_1", 0);
		imshow("frame_1", frame_1);
		// saving the frames in right and left folders
		char filename[100];
		sprintf(filename, "left/left%.2d.png", frame_number);
		imwrite(filename, frame_0);
		cout << "Saved " << filename << endl;
		sprintf(filename, "right/right%.2d.png", frame_number);
		imwrite(filename, frame_1);
		cout << "Saved " << filename << endl;

	} // stereo_capture

	void
	stereo_rectify ( const Mat png_images_left_frame, const Mat png_images_right_frame )
	{

		const char* intrinsic_filename = (char*) input_intrinsic_filename.c_str();
		const char* extrinsic_filename = (char*) input_extrinsic_filename.c_str();
		// reading png images
		Mat img1 ;
		Mat img2 ;
		png_images_left_frame.copyTo(img1);
		png_images_right_frame.copyTo(img2);

		// main rectification part
		float scale = 1.f;
		if( scale != 1.f )
		{
			Mat temp1, temp2;
			int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
			resize(img1, temp1, Size(), scale, scale, method);
			img1 = temp1;
			resize(img2, temp2, Size(), scale, scale, method);
			img2 = temp2;
		} // if
		// variable initialization
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

		// png2pcd -- check this one????
		img2.copyTo (left_rect_png_img);
		img1.copyTo (right_rect_png_img);
		//display the results
		namedWindow("left_rect_png_img", 0);
		imshow("left_rect_png_img", left_rect_png_img);
		namedWindow("right_rect_png_img", 0);
		imshow("right_rect_png_img", right_rect_png_img);

	}// stereo_rectify

    PointCloud<RGB>::Ptr png2pcd (const Mat rect_png_image)
    {

    	Mat image;
    	rect_png_image.copyTo(image);

		/// Convert the image from Gray to RGB
		cvtColor( image, image, CV_GRAY2BGR);

		// Retrieve the entries from the image data and copy them into the output RGB cloud
		double* pixel = new double [4];
		memset (pixel, 0, sizeof(double) * 4);
		PointCloud<RGB> cloud;

	   // cloud initialization
		cloud.width = image.cols;
		cloud.height = image.rows; // This indicates that the point cloud is organized
		cloud.is_dense = true;
		cloud.points.resize (cloud.width * cloud.height);

		// png to pcd loop
	   for( int y = 0; y < image.rows; y++ ){
		  for( int x = 0; x < image.cols; x++ ){
			  pixel[0] =  (double) image.at<Vec3b>(y,x)[0];
			  RGB color;
			  color.r = 0;
			  color.g = 0;
			  color.b = 0;
			  color.a = 0;
			  color.rgb = 0.0f;
			  color.rgba = 0;

			  int rgb;
			  int rgba;

//				  		//case 1: // component ==1
			color.r = static_cast<uint8_t> (pixel[0]);
			color.g = static_cast<uint8_t> (pixel[0]);
			color.b = static_cast<uint8_t> (pixel[0]);

			rgb = (static_cast<int> (color.r)) << 16 |
				(static_cast<int> (color.g)) << 8 |
				(static_cast<int> (color.b));

			rgba = rgb;
			color.rgb = static_cast<float> (rgb);
			color.rgba = static_cast<uint32_t> (rgba);

			  // Point<RGB> (x,y) = RGB
//				  		  cloud (x, image.rows - y - 1) = color;
			  cloud (x, y) = color;

		   } //for x
		  }// for y

	   // free memory
	  delete[] pixel;

	  // output pcd
	  // conversion from PointCloud<RGB> to PointCloud<RGB>::Ptr
	  PointCloud<RGB>::Ptr cloud_output (new PointCloud<RGB>);
	  *cloud_output = cloud;

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
    run_camera ()
    {
      for (int i=0;;i++)
      {

        pcl::PointCloud<pcl::RGB>::Ptr left_cloud (new pcl::PointCloud<pcl::RGB>);
        pcl::PointCloud<pcl::RGB>::Ptr right_cloud (new pcl::PointCloud<pcl::RGB>);
        CloudPtr out_cloud (new Cloud);

        //stereo capture
        stereo_capture(i);

        // online
        stereo_rectify(frame_0, frame_1);

  		//png2pcd -- mat img input
  		left_cloud = png2pcd (left_rect_png_img);
  		right_cloud = png2pcd (right_rect_png_img);

        // process the disparity map and point cloud
        processStereoPair (left_cloud, right_cloud, out_cloud);
        ///displaying disparity map
		pcl::PointCloud<pcl::RGB>::Ptr vmap (new pcl::PointCloud<pcl::RGB>);
		stereo.getVisualMap(vmap);
		image_viewer_disparity->addRGBImage<pcl::RGB> (vmap);
        image_viewer_disparity->spinOnce (100);

        // key to exit or continiue //used with stereo_rectify
//		printf("number of iages captured: ", i);
//		fflush(stdout);
		cout << "number of images captured: " << i << endl;
		char key = waitKey(5);
		if (key == 27) //esc
			break;
        
      } // while_old -for_new
    }// run

};//class

int
main (int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO ("usage: stereo_vision_processing left_directory right_directory intrinsic_filename extrinsic_filename\n");
    PCL_INFO ("note: frames will be saved in both left and right folders in PNG format.\n");
  }

  // save list of stereo frames in the current folders
  string left_frames_folder = argv[1];
  string right_frames_folder = argv[2];

  // calibration parameters for rectification
  string input_intrinsic_filename = argv[3];
  string input_extrinsic_filename = argv[4];

  // Process and display
  StereoVisionProcessing hrcs (left_frames_folder, right_frames_folder, input_intrinsic_filename, input_extrinsic_filename);
  hrcs.run_camera ();
  
  return 0;
}// main
