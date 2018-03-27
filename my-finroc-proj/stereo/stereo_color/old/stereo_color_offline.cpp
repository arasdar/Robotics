/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//#include <pcl/stereo/stereo_matching.h>
#include "projects/icarus/sensor_processing/libstereo/stereo_matching.h"

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

//#include <pcl/filters/passthrough.h>
//#include <pcl/sample_consensus/sac_model_plane.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace pcl;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

struct callback_args
{
  /*structure used to pass arguments to the callback function*/
  Cloud::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_2;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_3;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_4;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr_5;
};

class StereoVisionProcessing
{
private:
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_disparity_processed;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_right;
  boost::shared_ptr<visualization::PCLVisualizer> viewer_original;
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_disparity_processed;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_right;
  boost::shared_ptr<visualization::ImageViewer> image_viewer_original;
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
  bool trigger;
  bool continuous;
  bool display_normals;
  bool detect_obstacles;
  bool displayed;

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename;
  string input_extrinsic_filename ;
  Mat left_img_rect;
  Mat right_img_rect;

  pcl::AdaptiveCostSOStereoMatching stereo;
  int smooth_weak;
  int smooth_strong;

  /*save cloud parameters*/
  pcl::PCDWriter writer_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;

  struct callback_args cb_args;
  string   text_id_str [100];
  unsigned int text_id = 0;



public:
  StereoVisionProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                         const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename) :
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    viewer_disparity(new pcl::visualization::PCLVisualizer("3D Viewer disparity")),
    viewer_disparity_processed(new pcl::visualization::PCLVisualizer("3D Viewer disparity processed")),
    viewer_right(new pcl::visualization::PCLVisualizer("3D Viewer right")),
    viewer_original(new pcl::visualization::PCLVisualizer("3D Viewer original")),
    image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
    image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
    image_viewer_disparity_processed(new visualization::ImageViewer("Image Viewer Disparity processed")),
    image_viewer_right(new visualization::ImageViewer("Image Viewer right")),
    image_viewer_original(new visualization::ImageViewer("Image Viewer Original")),
    prev_cloud(new Cloud),
    prev_normal_cloud(new pcl::PointCloud<pcl::Normal>),
    prev_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    prev_ground_image(new Cloud),
    prev_label_image(new Cloud),
    road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
    road_segmentation(road_comparator),
    writer_()
  {
    trigger = true;
    continuous = false;
    display_normals = false;
    detect_obstacles = true;

    this->left_images = left_images;
    this->right_images = right_images;
    images_idx = 0;
    this->img_pairs_num = img_pairs_num;
    this->input_intrinsic_filename = input_intrinsic_filename;
    this->input_extrinsic_filename = input_extrinsic_filename;

    /*! Set up a 3D viewer*/
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);
    viewer->registerPointPickingCallback(&StereoVisionProcessing::pp_callback, *this, (void*)&cb_args);

    viewer_disparity->setBackgroundColor(0, 0, 0);
    viewer_disparity->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer_disparity->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);
    viewer_disparity->registerPointPickingCallback(&StereoVisionProcessing::pp_callback, *this, (void*)&cb_args);

    viewer_original->setBackgroundColor(0, 0, 0);
    viewer_original->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer_original->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);
    viewer_original->registerPointPickingCallback(&StereoVisionProcessing::pp_callback, *this, (void*)&cb_args);

    viewer_right->setBackgroundColor(0, 0, 0);
    viewer_right->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer_right->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);
    viewer_right->registerPointPickingCallback(&StereoVisionProcessing::pp_callback, *this, (void*)&cb_args);

    viewer_disparity_processed->setBackgroundColor(0, 0, 0);
    viewer_disparity_processed->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
    viewer_disparity_processed->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);
    viewer_disparity_processed->registerPointPickingCallback(&StereoVisionProcessing::pp_callback, *this, (void*)&cb_args);


    /*! susgested for AdaptiveCostSOStereoMatching*/
    stereo.setMaxDisparity(60); //original

    /*
     * setter for horizontal offset, i.e. number of pixels to shift the disparity range over the target image
     */
    stereo.setXOffset(0); //original
    stereo.setRadius(5); //original

    smooth_weak = 20;
    smooth_strong = 100;
    stereo.setSmoothWeak(smooth_weak); //20 original
    stereo.setSmoothStrong(smooth_strong); //original
    stereo.setGammaC(25); //original
    stereo.setGammaS(10); //10 original was original

    /*
     * setter for the value of the ratio filter
     * Parameters:
     * [in] ratio_filter  value of the ratio filter; it is a number in the range [0, 100] (0: no filtering action; 100: all disparities are filtered)
     *
     */
    /**
      *
      * * postprocessing: Ratio Filter (eliminating generic matching ambiguities, similar to that present in OpenCV Block Matching Stereo)
      */
    stereo.setRatioFilter(20);

    /*
     * setter for the value of the peak filter
     * Parameters:
     * [in] peak_filter value of the peak filter; it is a number in the range [0, inf] (0: no filtering action)
     *
     */
    /**
      * * postprocessing: filtering of wrong disparities via Peak Filter (eliminating ambiguities due to low-textured regions)
      *
      */
    stereo.setPeakFilter(0);

    /** \brief Stereo Matching abstract class
      * * postprocessing: Left-Right consistency check (eliminates wrong disparities at the cost of twice the stereo matching
      *   computation)
      * * postprocessing: subpixel refinement of computed disparities, to reduce the depth quantization effect
      */

    stereo.setLeftRightCheck(true);
    stereo.setLeftRightCheckThreshold(1);

    /**
      * * preprocessing of the image pair, to improve robustness against photometric distortions
      *   (wrt. to a spatially constant additive photometric factor)
      */
    stereo.setPreProcessing(true);

    //    /*
    //     *     Set up the ground plane comparator If the camera was pointing straight out, the normal would be:
    //     */
    //    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);
    //    /*Adjust for camera tilt:*/
    //    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    //    road_comparator->setExpectedGroundNormal(tilt_road_normal);
    //    road_comparator->setGroundAngularThreshold(pcl::deg2rad(10.0f));
    //    road_comparator->setAngularThreshold(pcl::deg2rad(3.0f));

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
    float   ground_angular_threshold = pcl::deg2rad(20.0f);  //10.0f original
    road_comparator->setGroundAngularThreshold(ground_angular_threshold); //10.0f original


    /*    Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.*/
    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians //3.0f original
    road_comparator->setAngularThreshold(angular_threshold); //3.0f original

    /*step threashold and Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.*/
    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
    road_comparator->setDistanceThreshold(distance_threshold, depth_dependent);
  } // constructor

  ~StereoVisionProcessing()
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
      case '1':
        smooth_strong -= 10;
        PCL_INFO("smooth_strong: %d\n", smooth_strong);
        stereo.setSmoothStrong(smooth_strong);
        break;
      case '2':
        smooth_strong += 10;
        PCL_INFO("smooth_strong: %d\n", smooth_strong);
        stereo.setSmoothStrong(smooth_strong);
        break;
      case '3':
        smooth_weak -= 10;
        PCL_INFO("smooth_weak: %d\n", smooth_weak);
        stereo.setSmoothWeak(smooth_weak);
        break;
      case '4':
        smooth_weak += 10;
        PCL_INFO("smooth_weak: %d\n", smooth_weak);
        stereo.setSmoothWeak(smooth_weak);
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
  pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
  {
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
      return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);

    /*Draw clicked points in red:*/
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 0, 0, 255); //blue now
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");

    /*Draw clicked points in red on viewer_2:*/
    data->viewerPtr_2->removePointCloud("clicked_points_2");
    data->viewerPtr_2->addPointCloud(data->clicked_points_3d, red, "clicked_points_2");
    data->viewerPtr_2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_2");

    /*Draw clicked points in red on viewer_3:*/
    data->viewerPtr_3->removePointCloud("clicked_points_3");
    data->viewerPtr_3->addPointCloud(data->clicked_points_3d, red, "clicked_points_3");
    data->viewerPtr_3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_3");

    /*Draw clicked points in red on viewer_3:*/
    data->viewerPtr_4->removePointCloud("clicked_points_4");
    data->viewerPtr_4->addPointCloud(data->clicked_points_3d, red, "clicked_points_4");
    data->viewerPtr_4->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_4");

    /*Draw clicked points in red on viewer_3:*/
    data->viewerPtr_5->removePointCloud("clicked_points_5");
    data->viewerPtr_5->addPointCloud(data->clicked_points_3d, red, "clicked_points_5");
    data->viewerPtr_5->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points_5");

    std::cout << "(X, Y, Z) -------> X: " << current_point.x << " -- Y: " << current_point.y << " -- Z: " << current_point.z << std::endl;
    std::cout << "(R, G, B) -------> R: " << current_point.r << " -- G: " << current_point.g << " -- B: " << current_point.b << std::endl;


    /* add text:*/
    std::stringstream ss;
    //ss << " X: " << current_point.x << ", Y: " << current_point.y << ", Z: " << current_point.z;
    //ss << " Z: " << current_point.z;
    //ss << current_point.z * 1.25f; //1.25 this si the const added to the disparity map
    ss << current_point.z;

    //const std::string   text,
    //const PointT  position;
    double  textScale = 0.3;  //0.4; //0.1; //1.0;
    double  r = 1.0;
    double  g = 1.0;
    double  b = 1.0;
    //const string   id [100];
    //int   viewport = 0;

    char str[512];
    sprintf(str, "text_id_%03d", text_id);
    text_id_str [text_id] = str;
    data->viewerPtr->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
    data->viewerPtr_2->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
    data->viewerPtr_3->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
    data->viewerPtr_4->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
    data->viewerPtr_5->addText3D(ss.str(), current_point, textScale, r, g, b,  str);
    cout << str <<  endl;
    text_id ++;
  }

  void
  stereo_rectify(const string input_images_left, const string input_images_right)
  {

    // showing the input images
    cout << "stereo_rectify -------> input_images_left: " << input_images_left << endl;
    cout << "stereo_rectify -------> input_images_right: " << input_images_right << endl;

    //variable initialization
    const char* img1_filename = (char*) input_images_left.c_str();
    const char* img2_filename = (char*) input_images_right.c_str();
    const char* intrinsic_filename = (char*) input_intrinsic_filename.c_str();
    const char* extrinsic_filename = (char*) input_extrinsic_filename.c_str();

    // reading png images
    int color_mode = 1; //grayscale
    Mat img1 = imread(img1_filename, color_mode);
    Mat img2 = imread(img2_filename, color_mode);
    namedWindow("left", 0);
    imshow("left", img1);
    namedWindow("right", 0);
    imshow("right", img2);

    // Mat img = imread(goodImageList[i*2+k]
    Mat img[2];
    img1.copyTo(img[0]);
    img2.copyTo(img[1]);
    //Size imageSize = img1.size();

    /// Convert the image from Gray to RGB
    //cvtColor(image, image, CV_GRAY2BGR);
    //cvtColor(img1, img[0], CV_BGR2GRAY);
    //cvtColor(img2, img[1], CV_BGR2GRAY);


    Size imageSize = img[0].size();
    Mat canvas;
    double sf;
    int w, h;
    // OpenCV can handle left-right or up-down camera arrangements
    bool isVerticalStereo = false; //fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    if (!isVerticalStereo)
    {
      sf = 600. / MAX(imageSize.width, imageSize.height);
      w = cvRound(imageSize.width * sf);
      h = cvRound(imageSize.height * sf);
      canvas.create(h, w * 2, CV_8UC3);
    }


    //Mat img_rect[2];
    for (int k = 0; k < 2; k++)
    {
      //cvtColor(rimg, cimg, CV_GRAY2BGR);
      Mat canvasPart = canvas(Rect(w * k, 0, w, h)) ;
      resize(img[k], canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
    }


    if (!isVerticalStereo)
      for (int j = 0; j < canvas.rows; j += 16)
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

    imshow("rectified_true", canvas);


    //img_rect[0].copyTo(left_img_rect);
    //img_rect[1].copyTo(right_img_rect);
    img[0].copyTo(left_img_rect);
    img[1].copyTo(right_img_rect);
    namedWindow("left_rect", 0);
    imshow("left_rect", left_img_rect);
    namedWindow("right_rect", 0);
    imshow("right_rect", right_img_rect);

  }// stereo_rectify_test

  PointCloud<RGB>::Ptr
  img2pcd(const Mat image_rect)
  {

    Mat image;
    image_rect.copyTo(image);

    /// Convert the image from Gray to RGB
    //cvtColor(image, image, CV_GRAY2BGR);

    // Retrieve the entries from the image data and copy them into the output RGB cloud
    //double* pixel = new double [4];
    //memset(pixel, 0, sizeof(double) * 4);
    //pixel[0] = (double) image.at<Vec3b>(y, x)[0];
    // free memory
    //delete[] pixel;

    double pixel;
    PointCloud<RGB> cloud;

    // cloud initialization
    cloud.width = image.cols;
    cloud.height = image.rows; // This indicates that the point cloud is organized
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    // sky removal
    vector<int> valid_indices;

    // png to pcd loop
    for (int y = 0; y < image.rows; y++)
    {
      for (int x = 0; x < image.cols; x++)
      {
        pixel = (double) image.at<Vec3b>(y, x)[0];

        //sky detection test
        //if (pixel < 250)
        //{}// if color for sky

        RGB color;
        color.r = 0;
        color.g = 0;
        color.b = 0;
        color.a = 0;
        color.rgb = 0.0f;
        color.rgba = 0;


        //case 1: component ==1
        color.r = static_cast<uint8_t>(pixel);
        color.g = static_cast<uint8_t>(pixel);
        color.b = static_cast<uint8_t>(pixel);

        int rgb;
        rgb = (static_cast<int>(color.r)) << 16 |
              (static_cast<int>(color.g)) << 8 |
              (static_cast<int>(color.b));

        //color.rgb = static_cast<double>(pixel_test);
        color.rgb = static_cast<float>(rgb);
        color.rgba = static_cast<uint32_t>(rgb);



        // Point<RGB> (x,y) = RGB
        //cloud (x, image.rows - y - 1) = color;
        cloud(x, y) = color;

        valid_indices.push_back(y + x);


      } //for x
    }// for y


    // output pcd
    // conversion from PointCloud<RGB> to PointCloud<RGB>::Ptr
    PointCloud<RGB>::Ptr cloud_output(new PointCloud<RGB>);
    *cloud_output = cloud;
    //copyPointCloud(cloud, valid_indices, *cloud_output); //sky removal but different sizes


    return cloud_output;
  }// png2pcd

  void
  processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud, pcl::PointCloud<pcl::RGB>::Ptr& texture)
  {

    stereo.compute(*left_image, *right_image);

    /**
      * * postprocessing: smoothing of the disparity map via median filter
      * * after stereo matching a PCL point cloud can be computed, given the stereo intrinsic (focal, principal point
      *   coordinates) and extrinsic (baseline) calibration parameters
      */

    stereo.medianFilter(4); //10 used for long time very good but noisy // better but slower //optimal


    /*stereo images camera calibration parameters for haris camera color*/
    float u_c = 644.292; //dx //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
    float v_c = 491.472; //dy //358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
    float focal = 961.074; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
    float baseline = 0.12; //0.46; //meter for unit //0.359294689f; //real one using calculator
    stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud, texture);

  }// processPair

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
      prev_label_image = label_image;
      prev_ground_normal = ground_plane_params;
      prev_ground_centroid = ground_centroid;
      cloud_mutex.unlock();
    }
  }


  void
  saveCloud(const CloudConstPtr& cloud)
  {
    std::stringstream ss;

    /*an alternative*/
    //char str[512];
    //sprintf(str, "text_id_%03d", text_id);

    if (images_idx < 10 && images_idx < 100 && images_idx < 1000)
    {
      ss << dir_name_ << "/" << file_name_ << "_00" << images_idx << ".pcd";
    }
    if (images_idx >= 10 && images_idx < 100 && images_idx < 1000)
    {
      ss << dir_name_ << "/" << file_name_ << "_0" << images_idx << ".pcd";
    }
    if (images_idx > 10 && images_idx >= 100 && images_idx < 1000)
    {
      ss << dir_name_ << "/" << file_name_ << "_" << images_idx << ".pcd";
    }

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

    if (format_ & 4) //this is the one now default
    {
      writer_.writeBinaryCompressed<PointT> (ss.str(), *cloud);
      //std::cerr << "Data saved in BINARY COMPRESSED format to " << ss.str () << std::endl;
    }
  }

  void
  saveCloud_left(const PointCloud<RGB>::ConstPtr& cloud)
  {
    std::stringstream ss;
    ss << "frames_pcd_left_RGB" << "/" << "left_pcd" << "_" << images_idx << ".pcd";
    cout << "Saved " << ss.str() << endl;
    writer_.writeBinaryCompressed<RGB> (ss.str(), *cloud);
  }

  void
  saveCloud_right(const PointCloud<RGB>::ConstPtr& cloud)
  {
    std::stringstream ss;
    ss << "frames_pcd_right_RGB" << "/" << "right_pcd" << "_" << images_idx << ".pcd";
    cout << "Saved " << ss.str() << endl;
    writer_.writeBinaryCompressed<RGB> (ss.str(), *cloud);
  }

  void
  saveCloud_disp(const PointCloud<RGB>::ConstPtr& cloud)
  {
    std::stringstream ss;
    ss << "frames_pcd_disp_pcd" << "/" << "disp_pcd" << "_" << images_idx << ".pcd";
    cout << "Saved " << ss.str() << endl;
    writer_.writeBinaryCompressed<RGB> (ss.str(), *cloud);
  }

  void
  run()
  {
    while (!viewer->wasStopped())
    {

      /*exiting criteria*/
      if (img_pairs_num == images_idx)
      {
        cout << "img_pairs_num == images_idx -> " << images_idx << std::endl;
        break;
      } //if

      /*Proceed or nor to the next image*/
      if (trigger || continuous)
      {

        stereo_rectify(left_images[images_idx], right_images[images_idx]);

        pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
        pcl::PointCloud<pcl::RGB>::Ptr right_cloud(new pcl::PointCloud<pcl::RGB>);
        CloudPtr out_cloud(new Cloud);

        /*png2pcd -- mat img input*/
        left_cloud = img2pcd(left_img_rect);
        right_cloud = img2pcd(right_img_rect);
        //saveCloud_left(left_cloud);
        //saveCloud_right(right_cloud);

        /*process the disparity map and point cloud*/
        processStereoPair(left_cloud, right_cloud, out_cloud, left_cloud);
        file_name_ = "frames_pcd_left";
        dir_name_ = "frames_pcd_left";
        format_ = 4;
        saveCloud(out_cloud);
        viewer->removeText3D("cloud");
        //viewer_test->removeText3D("cloud_test");
        image_viewer->removeLayer("line");  //const std::string &layer_id
        //image_viewer_test->removeLayer("line");
        processCloud(out_cloud);

        /* displaying the points selected by gui - mouse and */
        Cloud::Ptr clicked_points_3d(new Cloud);
        cb_args.clicked_points_3d = clicked_points_3d;
        cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
        cb_args.viewerPtr_2 = pcl::visualization::PCLVisualizer::Ptr(viewer_disparity);
        cb_args.viewerPtr_3 = pcl::visualization::PCLVisualizer::Ptr(viewer_original);
        cb_args.viewerPtr_4 = pcl::visualization::PCLVisualizer::Ptr(viewer_right);
        cb_args.viewerPtr_5 = pcl::visualization::PCLVisualizer::Ptr(viewer_disparity_processed);


        /*removing and clean up*/
        cb_args.clicked_points_3d->clear();
        viewer->removeAllPointClouds();  //ok
        viewer_disparity->removeAllPointClouds();
        viewer_disparity_processed->removeAllPointClouds();
        viewer_original ->removeAllPointClouds();
        viewer_right->removeAllPointClouds();
        for (int i = 0; i < 100; i++)
        {
          viewer->removeText3D(text_id_str[i]);
          viewer_disparity->removeText3D(text_id_str[i]);
          viewer_disparity_processed->removeText3D(text_id_str[i]);
          viewer_original->removeText3D(text_id_str[i]);
          viewer_right->removeText3D(text_id_str[i]);
        }//for
        text_id = 0;

        /*! displaying disparity map*/
        pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
        stereo.getVisualMap(vmap);
        //image_viewer_disparity->addRGBImage<pcl::RGB> (vmap); //working
        //saveCloud_disp(vmap); //working
        CloudPtr out_cloud_disp(new Cloud);
        processStereoPair(left_cloud, right_cloud, out_cloud_disp, vmap);
        file_name_ = "frames_pcd_disp";
        dir_name_ = "frames_pcd_disp";
        format_ = 4;
        saveCloud(out_cloud_disp);
        image_viewer_disparity->addRGBImage<PointT> (out_cloud_disp);
        if (!viewer_disparity->updatePointCloud(out_cloud_disp, "cloud disparity"))
        {
          viewer_disparity->addPointCloud(out_cloud_disp, "cloud disparity");
        }//if


        /*! displaying disparity map processed*/
        CloudPtr out_cloud_disp_processed(new Cloud);
        processStereoPair(left_cloud, right_cloud, out_cloud_disp_processed, vmap);
        image_viewer_disparity_processed->addRGBImage<PointT> (out_cloud_disp_processed);
        if (!viewer_disparity_processed->updatePointCloud(out_cloud_disp, "cloud disparity processed"))
        {
          viewer_disparity_processed->addPointCloud(out_cloud_disp, "cloud disparity processed");
        }//if

        /* displaying point cloud from left camera pose*/
        CloudPtr out_cloud_left(new Cloud);
        processStereoPair(left_cloud, right_cloud, out_cloud_left, left_cloud);
        image_viewer_original->addRGBImage<PointT>(out_cloud_left);
        if (!viewer_original->updatePointCloud(out_cloud_left, "cloud original"))
        {
          viewer_original->addPointCloud(out_cloud_left, "cloud original");
        }//if


        /* displaying point cloud form right image pose*/
        CloudPtr out_cloud_right(new Cloud);
        processStereoPair(left_cloud, right_cloud, out_cloud_right, right_cloud);
        file_name_ = "frames_pcd_right";
        dir_name_ = "frames_pcd_right";
        format_ = 4;
        saveCloud(out_cloud_right);
        image_viewer_right->addRGBImage<PointT> (out_cloud_right);
        if (!viewer_right->updatePointCloud(out_cloud_right, "cloud right"))
        {
          viewer_right->addPointCloud(out_cloud_right, "cloud right");
        }//if

        cout << "left_images[img_index]: " << left_images[images_idx] << std::endl;
        cout << "right_images[img_index]: " << right_images[images_idx] << std::endl;
        cout << "images_idx: " << images_idx << endl;
        cout << "img_pairs_num: " << img_pairs_num << endl;
        cout << "press q or Q on the main viewer to exit or space to continue................. " << endl;

        images_idx++;
        trigger = false;
      }// if trigger

      /*! Draw visualizations*/
      //if (cloud_mutex.try_lock())
      //{        cloud_mutex.unlock();      } //if cloud_mutex


      if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
      {
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_ground_image);
        viewer->addPointCloud(prev_ground_image, rgb, "cloud"); //works too
        //viewer->addPointCloud(prev_ground_image, "cloud");


      }//if

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

      /*Show the ground plane normal*/
      Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0);

      /*Adjust for camera tilt*/
      Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;

      /*Show the ground plane normal*/
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



      viewer->spinOnce(1);
      viewer_disparity->spinOnce(1);
      viewer_right->spinOnce(1);
      viewer_disparity->spinOnce(1);
      viewer_original->spinOnce(1);
      image_viewer->spinOnce(1);
      image_viewer_disparity->spinOnce(1);



      /* key to exit or continue*/
      char key = waitKey(1);
      if (key == 27)
        break;

    } // while
  }// run

};//class

int
main(int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO("usage: aras_icarus_sensorProcessing_stereoColor_offline left_image_directory right_image_directory intrinsic_parameter_filename extrinsic_parameter_filename\n");
    PCL_INFO("note: images in both left and right folders can be in different format.\n");
    PCL_INFO("for example : \n"
             "aras_icarus_sensorProcessing_stereoColor_offline ~/stereo_images/stereo_lugv/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_1/left/ ~/stereo_images/stereo_lugv/2013-11-13_Dataset_Meerdaal_13Nov2013/Dataset_1/right/ ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/intrinsics.yml ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/extrinsics.yml\n ");
    return -1;
  }


  /*variable initial*/
  int img_number_left = 0, img_number_right = 0 ;
  int img_pairs_num = 0;

  /*Get list of stereo files from left folder*/
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  /*reading right images from folder*/
  std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr(argv[2]); itr != end_itr; ++itr)
  {
    right_images.push_back(itr->path().string());
    img_number_right++;
  }
  sort(right_images.begin(), right_images.end());
  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  /*showing the input images*/
  cout << "img_number_left: " << img_number_left << std::endl;
  cout << "img_number_right: " << img_number_right << std::endl;
  if (img_number_left == img_number_right)
    img_pairs_num = img_number_left;

  /*calibration parameters*/
  string input_intrinsic_filename = argv[3];
  string input_extrinsic_filename = argv[4];

  /*Process and display*/
  StereoVisionProcessing stereo_vision_processing(left_images, right_images, img_pairs_num, input_intrinsic_filename, input_extrinsic_filename);
  stereo_vision_processing.run();

  return 0;
}// main
