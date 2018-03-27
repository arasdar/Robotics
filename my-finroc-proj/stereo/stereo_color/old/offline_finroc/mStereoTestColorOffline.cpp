//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/icarus/sensor_processing/stereo_color/offline/mStereoTestColorOffline.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-02-25
 *
 */
//----------------------------------------------------------------------
#include "projects/icarus/sensor_processing/stereo_color/offline/finroc/mStereoTestColorOffline.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace icarus
{
namespace sensor_processing
{
namespace stereo_color
{
namespace offline
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mStereoTestColorOffline> cCREATE_ACTION_FOR_M_STEREOTESTCOLOROFFLINE("StereoTestColorOffline");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mStereoTestColorOffline constructor
//----------------------------------------------------------------------
mStereoTestColorOffline::mStereoTestColorOffline(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
  //If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
  viewer_disparity(new pcl::visualization::PCLVisualizer("3D Viewer disparity")),
  viewer_disparity_processed(new pcl::visualization::PCLVisualizer("3D Viewer disparity processed")),
  viewer_right(new pcl::visualization::PCLVisualizer("3D Viewer right")),
  viewer_original(new pcl::visualization::PCLVisualizer("3D Viewer original")),
  image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
  image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
  image_viewer_disparity_processed(new visualization::ImageViewer("Image Viewer Disparity processed")),
  image_viewer_right(new visualization::ImageViewer("Image Viewer right")),
  image_viewer_original(new visualization::ImageViewer("Image Viewer Original"))
  ,
  road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
  road_segmentation(road_comparator)
  ,
  writer_()
{
  trigger = true;
  continuous = false;
  display_normals = false;

  this->left_images = left_images;
  this->right_images = right_images;
  images_idx = 0;
  this->img_pairs_num = img_pairs_num;
  this->input_intrinsic_filename = input_intrinsic_filename;
  this->input_extrinsic_filename = input_extrinsic_filename;

  /*! Set up a 3D viewer*/
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer->registerKeyboardCallback(&mStereoTestColorOffline::keyboardCallback, *this, 0);
  viewer->registerPointPickingCallback(&mStereoTestColorOffline::pp_callback, *this, (void*)&cb_args);

  viewer_disparity->setBackgroundColor(0, 0, 0);
  viewer_disparity->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disparity->registerKeyboardCallback(&mStereoTestColorOffline::keyboardCallback, *this, 0);
  viewer_disparity->registerPointPickingCallback(&mStereoTestColorOffline::pp_callback, *this, (void*)&cb_args);

  viewer_original->setBackgroundColor(0, 0, 0);
  viewer_original->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_original->registerKeyboardCallback(&mStereoTestColorOffline::keyboardCallback, *this, 0);
  viewer_original->registerPointPickingCallback(&mStereoTestColorOffline::pp_callback, *this, (void*)&cb_args);

  viewer_right->setBackgroundColor(0, 0, 0);
  viewer_right->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_right->registerKeyboardCallback(&mStereoTestColorOffline::keyboardCallback, *this, 0);
  viewer_right->registerPointPickingCallback(&mStereoTestColorOffline::pp_callback, *this, (void*)&cb_args);

  viewer_disparity_processed->setBackgroundColor(0, 0, 0);
  viewer_disparity_processed->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
  viewer_disparity_processed->registerKeyboardCallback(&mStereoTestColorOffline::keyboardCallback, *this, 0);
  viewer_disparity_processed->registerPointPickingCallback(&mStereoTestColorOffline::pp_callback, *this, (void*)&cb_args);


  /*! susgested for AdaptiveCostSOStereoMatching same as gray*/
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

  //stereo.setRatioFilter(20);
  //stereo.setPeakFilter(0);
  //stereo.setLeftRightCheck(true);
  //stereo.setLeftRightCheckThreshold(1);
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


  /*
   * reading the input folders and calibration files
   */
  /*variable initial*/
  int img_number_left = 0, img_number_right = 0 ;
  //int img_pairs_num = 0;

  //--> /home/aras/stereo_images/Dataset Meerdaal 13Nov2013/Dataset 2
  string argv_1 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 2/left";
  string argv_2 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 2/right";
  string argv_3 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 2/calib_1-ok/intrinsics.yml";
  string argv_4 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 2/calib_1-ok/extrinsics.yml";


  //  //--> /home/aras/stereo_images/Dataset Meerdaal 13Nov2013/Dataset 2 TODO testing
  //  string argv_1 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/left";
  //  string argv_2 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/right";
  //  string argv_3 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/calib_1-ok/intrinsics.yml";
  //  string argv_4 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/calib_1-ok/extrinsics.yml";

  //  //TODO testing
  //  string argv_1 = "/home/aras/stereo_images/stereo_lugv/2014-02-13_3dv-dataset 3/left_rect";
  //  string argv_2 = "/home/aras/stereo_images/stereo_lugv/2014-02-13_3dv-dataset 3/right_rect";
  //  string argv_3 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/calib_1-ok/intrinsics.yml";
  //  string argv_4 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/calib_1-ok/extrinsics.yml";

  //  //TODO testing
  //  string argv_1 = "/home/aras/stereo_images/stereo_lugv/sample/left_rect";
  //  string argv_2 = "/home/aras/stereo_images/stereo_lugv/sample/right_rect";
  //  string argv_3 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/calib_1-ok/intrinsics.yml";
  //  string argv_4 = "/home/aras/stereo_images/stereo_lugv/2013-11-13_Dataset Meerdaal 13Nov2013/Dataset 1/calib_1-ok/extrinsics.yml";

  /*Get list of stereo files from left folder*/
  //std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv_1); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  /*reading right images from folder*/
  //std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr(argv_2); itr != end_itr; ++itr)
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
  input_intrinsic_filename = argv_3;
  input_extrinsic_filename = argv_4;
}// constructor

void
mStereoTestColorOffline::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
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
    }
  }
}

void
mStereoTestColorOffline::pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
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
mStereoTestColorOffline::stereo_rectify(const string input_images_left, const string input_images_right)
{

  // showing the input images
  cout << "stereo_rectify -------> input_images_left: " << input_images_left << endl;
  cout << "stereo_rectify -------> input_images_right: " << input_images_right << endl;

  //variable initialization
  const char* img1_filename = (char*) input_images_left.c_str();
  const char* img2_filename = (char*) input_images_right.c_str();
//  const char* intrinsic_filename = (char*) input_intrinsic_filename.c_str();
//  const char* extrinsic_filename = (char*) input_extrinsic_filename.c_str();

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
mStereoTestColorOffline::img2pcd(const Mat image_rect)
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
mStereoTestColorOffline::processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud, pcl::PointCloud<pcl::RGB>::Ptr& texture)
{

  stereo.compute(*left_image, *right_image);
  stereo.medianFilter(4); //10 used for long time very good but noisy // better but slower //optimal

  /*stereo images camera calibration parameters for haris camera color*/
  float u_c = 644.292f; //dx //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  float v_c = 491.472f; //dy //358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
  float focal = 961.074f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  float baseline = 0.120098f; //0.46; //meter for unit //0.359294689f; //real one using calculator
  stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud, texture);

}// processPair

void
mStereoTestColorOffline::run()
{

  if (img_pairs_num == images_idx)
  {
    cout << "img_pairs_num == images_idx -> " << images_idx << std::endl;
    //break;
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

    /*process the disparity map and point cloud*/
    processStereoPair(left_cloud, right_cloud, out_cloud, left_cloud);
    file_name_ = "frames_pcd_left";
    dir_name_ = "frames_pcd_left";
    format_ = 4;
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

  /*! Draw visualizations old before gcc4.8*/
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
  else if (!display_normals)
  {
    viewer->removePointCloud("normals");
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
  waitKey(1);
  //char key = waitKey(1);
  //if (key == 27)
  //break;

}// run

void
mStereoTestColorOffline::processCloud(const CloudConstPtr& cloud)
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

  /*filtering*/
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  //pass.setFilterFieldName("x");
  //pass.setFilterLimits(-10, 10);
  pass.setFilterFieldName("y");
  //pass.setFilterLimits(dominant_plane_y_min - 3, dominant_plane_y_max + 1);
  pass.setFilterLimits(dominant_plane_y_min, dominant_plane_y_max);
  //pass.setFilterFieldName("z");
  //pass.setFilterLimits(0, 50.0);
  CloudPtr cloud_filtered(new Cloud);
  pass.filter(*cloud_filtered);
  //CloudPtr ground_image_test(new Cloud);
  //pass.filter(*ground_image_test);
  cout << "size: " << cloud_filtered->points.size() << endl;
  //cout << "size: " << ground_image_test->points.size() << endl;
  //cout << "size 2: " << ground_image_test->size() << endl;
  unsigned int threshold_dominant_plane_cloud = (cloud->width / 10) * (cloud->height / 10) * 1.5 * 10;
  cout << "threshold dominant_plane_cloud: " << threshold_dominant_plane_cloud << endl;
  if (cloud_filtered->size() < threshold_dominant_plane_cloud) //5000 =~ 80 * 60 = 4800
  {
    cout << "bad point cloud and not relibale ........................................... because cloud size is : " << cloud_filtered->size() << endl;
    frame_isGood = false;

    //put text on the cloud or images
    /* add text:*/
    std::stringstream ss;
    ss << "BAD FRAME" ;
    double  textScale = 0.5; //0.3;  //0.4; //0.1; //1.0;
    double  r = 1.0;
    double  g = 0.0;
    double  b = 0.0;
    //char str[10] = "text_id";
    //char str_2[10] = "text_id_2";
    PointT position; //(centroids[0])
    position.x = -2; //centroids[0][0]; //0;
    position.y = 0; //centroids[0][1];//0;
    position.z = 5; //centroids[0][2];//1;
    viewer->addText3D(ss.str(), position, textScale, r, g, b,  "cloud");
    //viewer->addText(ss.str(), 400, 300, textScale, r, g, b, "cloud");
    //viewer_test->addText3D(ss.str(), position, textScale, r, g, b, "cloud_test");

    unsigned int x_min = 0, y_min = 0, x_max = cloud->width, y_max = cloud->height; //image.width=800, image.height=600
    //bool  addLine (unsigned int x_min, unsigned int y_min, unsigned int x_max, unsigned int y_max, double r, double g, double b, const std::string &layer_id="line", double opacity=1.0)
    double opacity = 1.0;
    //const string layer_id;
    image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    //image_viewer_test->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    x_min = cloud->width;
    y_min = 0;
    x_max = 0;
    y_max = cloud->height;
    image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
    //image_viewer_test->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
  }



  /*! dominant ground plane parameters for the dominant plane which is the largest one :)*/
  Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);
  if (ground_cloud->points.size() > 0)
  {
    ground_centroid = centroids[0];
    ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
  }// if


  /*! other traversable regions in dominant plane limits - green*/
  if (frame_isGood)
  {
    /*! other traversable regions in dominant plane limits - green*/
    for (unsigned int i = 0; i < region_indices.size(); i++) //only looking max value
    {

      /*looking for other the largest region*/
      if (region_indices[i].indices.size() > threshold_min_dominant_plane_cloud &&
          region_indices[i].indices.size() <= largest_1 &&
          region_indices[i].indices.size() > largest_1 / 2) //100 original //min_inliers - green -- for each segment 1000 original
      {

        /*! Compute plane info*/
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero();
        Eigen::Matrix3f clust_cov;
        pcl::computeMeanAndCovarianceMatrix(*cloud, region_indices[i].indices, clust_cov, clust_centroid);

        //cout << "cluster_centroid: " << clust_centroid << endl;
        cout << "cluster_centroid_y: " << clust_centroid[1] << endl;


        pcl::PointXYZ centroid_pt(clust_centroid[0], clust_centroid[1], clust_centroid[2]);
        //double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
        double ptp_dist =  pcl::pointToPlaneDistance(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

        //if (plane_params[3] < dominant_plane_y_max && plane_params[3] > dominant_plane_y_min)
        if (clust_centroid[1] <= dominant_plane_y_max && clust_centroid[1] >= dominant_plane_y_min && ptp_dist < 1)
        {

          cout << "cluster_centroid meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;

          /*visualizing the result by colorizing the point cloud */ /*! traversable regions - green*/
          for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
          {
            ground_image->points[region_indices[i].indices[j]].r = 0;
            ground_image->points[region_indices[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices[i].indices[j]].g + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].b = 0;
            label_image->points[region_indices[i].indices[j]].r = 0;
            label_image->points[region_indices[i].indices[j]].g = 255 ;
            label_image->points[region_indices[i].indices[j]].b = 0;
            pcl::PointXYZ ground_pt(cloud->points[region_indices[i].indices[j]].x,
                                    cloud->points[region_indices[i].indices[j]].y,
                                    cloud->points[region_indices[i].indices[j]].z);
            ground_cloud->points.push_back(ground_pt);

          }// for
        }//if
        else
        {
          cout << "cluster_centroid NOT meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
        }//else


      } //if region
    }// for i
  }// frame


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

    //Note the regions that have been extended
    for (unsigned int i = 0; i < region_indices.size(); i++)
    {
      if (region_indices[i].indices.size() >= 1000)
      {
        for (unsigned int j = 0; j < region_indices[i].indices.size(); j++)
        {
          if (label_image->points[region_indices[i].indices[j]].g != 255)  //region_indices_test[i].indices.size() >= 1000
          {
            //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
            //ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
            ground_image->points[region_indices[i].indices[j]].r = 255;
            ground_image->points[region_indices[i].indices[j]].g = 255;
            ground_image->points[region_indices[i].indices[j]].b = 0;
            label_image->points[region_indices[i].indices[j]].r = 255;
            label_image->points[region_indices[i].indices[j]].g = 255;
            label_image->points[region_indices[i].indices[j]].b = 0;
          }//if
        }//for
      }//if
    }// for

  }//if for plane refinement



//  /*Segment Obstacles using OrganizedConnectedComponentSegmentation and EuclideanClusterComparator*/
//  if (frame_isGood && ground_cloud->points.size())
//  {
//    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_compare(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
//    euclidean_cluster_compare->setInputCloud(cloud);
//    euclidean_cluster_compare->setInputNormals(normal_cloud);
//
//    float distance_threshold = 0.1f;  //the tolerance in meters (at 1m)
//    bool depth_dependent = false;   //whether to scale the threshold based on range from the sensor (default: false)
//    euclidean_cluster_compare->setDistanceThreshold(distance_threshold, depth_dependent);
//
//    float angular_threshold = pcl::deg2rad(10.0f); //the tolerance in radians
//    euclidean_cluster_compare->setAngularThreshold(angular_threshold); //30 degree //kinematic and vehicle capability
//
//    boost::shared_ptr<pcl::PointCloud<pcl::Label>> labels_ptr(new pcl::PointCloud<pcl::Label>());
//    *labels_ptr = labels;
//    euclidean_cluster_compare->setLabels(labels_ptr);
//
//    std::vector<bool> plane_labels;
//    plane_labels.resize(region_indices.size(), false);
//    for (size_t i = 0; i < region_indices.size(); i++)
//    {
//      if (region_indices[i].indices.size() > 1000)
//      {
//        plane_labels[i] = true;
//      }//if
//    }// for
//    euclidean_cluster_compare->setExcludeLabels(plane_labels);
//
//
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(road_comparator);
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(rgb_compare);
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(edge_aware_compare); //compiling but not working - assertion error!!
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(euclidean_compare);
//    pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_cc_segmentation(euclidean_cluster_compare);
//    //pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> organized_segmentation(refinement_compare); //compiling and wroking but no result!!!
//    organized_cc_segmentation.setInputCloud(cloud);
//    pcl::PointCloud<pcl::Label> labels_test;
//    std::vector<pcl::PointIndices> inlier_indices_test;
//    organized_cc_segmentation.segment(labels_test, inlier_indices_test);
//
//    /*! Draw the segmentation result testtttttt*/
//    //CloudPtr ground_image_test(new Cloud);
//    //*ground_image_test = *cloud;
//    vector<PointIndices> region_indices_test(inlier_indices_test); //region_indices
//
//    /*colorizing the point cloud*/
//    for (unsigned int i = 0; i < region_indices_test.size(); i++)
//    {
//      if (region_indices_test[i].indices.size() >= 1000)
//      {
//
//        //Cloud cluster;
//        //pcl::copyPointCloud(*ground_image_test, region_indices_test[i].indices, cluster);
//        //clusters.push_back(cluster);
//
//        /*! Compute plane info*/
//        Eigen::Vector4f cluster_centroid = Eigen::Vector4f::Zero();
//        Eigen::Matrix3f cluster_covariance;
//        pcl::computeMeanAndCovarianceMatrix(*ground_image, region_indices_test[i].indices, cluster_covariance, cluster_centroid);
//
//        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
//        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
//        pcl::eigen33(cluster_covariance, eigen_value, eigen_vector);
//        Eigen::Vector3f plane_params;
//        plane_params[0] = eigen_vector[0];
//        plane_params[1] = eigen_vector[1];
//        plane_params[2] = eigen_vector[2];
//        //plane_params[3] = 0;
//
//        //float cos_theta = abs(plane_params.dot(ground_plane_params)); //tilt_road_normal
//        Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
//        Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
//        //road_comparator->setExpectedGroundNormal(tilt_road_normal);
//        float cos_theta = abs(plane_params.dot(tilt_road_normal)); //tilt_road_normal
//
//        //#define PI 3.14159265
//        //float   ground_angular_threshold = 20.0f;
//        //float thresold = cos (ground_angular_threshold * PI / 180.0 ); //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//        //float thresold_obstacle = cos ( 45.0 ); // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//        //float threshold_semi_travers = cos (20.0);
//        //float threshold_semi_travers = cos (angular_threshold * PI / 180.0);
//        float thresold_obstacle = cos(70.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//        //float threshold_trav = cos(20.0);    // 2*20 + 5 //cos(20.0); //degree the angular_threshold for ground  //0.1 for prependicular surfaces
//
//
//        pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
//        double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);
//
//        for (unsigned int j = 0; j < region_indices_test[i].indices.size(); j++)
//        {
//
//          //          if ((ptp_dist <= distance_threshold) && (cos_theta > threshold_trav))  //traversable and dominant plane
//          //          {
//          //            if (label_image->points[region_indices_test[i].indices[j]].g == 255)  //region_indices_test[i].indices.size() >= 1000
//          //            {
//          //              //ground_image->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//          //              ground_image_test->points[region_indices_test[i].indices[j]].g = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].g + 255) / 2);
//          //            }//if
//          //            else
//          //            {
//          //              ground_image_test->points[region_indices_test[i].indices[j]].b = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].b + 255) / 2);
//          //            }
//          //          }//else ptp
//
//          if ((ptp_dist > distance_threshold) && (cos_theta < thresold_obstacle))  //obstacle
//          {
//            //ground_image_test->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//            ground_image->points[region_indices_test[i].indices[j]].r = static_cast<uint8_t>((cloud->points[region_indices_test[i].indices[j]].r + 255) / 2);
//            ground_image->points[region_indices_test[i].indices[j]].g = 0;
//            ground_image->points[region_indices_test[i].indices[j]].b = 0;
//            label_image->points[region_indices_test[i].indices[j]].r = 255;
//            label_image->points[region_indices_test[i].indices[j]].g = 0;
//            label_image->points[region_indices_test[i].indices[j]].b = 0;
//          }// if ptp
//        }//for j indices
//      }// if for min inliers to 1000
//    }// for i the mainn one for regions
//
//  }// if obstacle for euclidean + cc


  // note the NAN points in the image as well
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if (!pcl::isFinite(cloud->points[i]))
    {
      ground_image->points[i].r = 0;
      ground_image->points[i].g = 0;
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
mStereoTestColorOffline::gridmap()
{
  data_ports::tPortDataPointer<rrlib::mapping::tMapGridCartesian2D<double>> output_gridmap_ptr = output_gridmap.GetUnusedBuffer();
  //  rrlib::mapping::tMapGridCartesian2D<double>& map_stereo = *output_gridmap_ptr;
  output_gridmap_ptr->Clear(0);

  bounds_stereo_env.upper_bounds[0] = rrlib::math::tVec2d(bound_limit, 0);
  bounds_stereo_env.lower_bounds[0] = rrlib::math::tVec2d(-bound_limit, 0);
  bounds_stereo_env.upper_bounds[1] = rrlib::math::tVec2d(0, bound_limit);
  bounds_stereo_env.lower_bounds[1] = rrlib::math::tVec2d(0, -bound_limit);

  output_gridmap_ptr->SetBounds(bounds_stereo_env);
  output_gridmap_ptr->SetResolution(rrlib::math::tVec2d(cell_resolution, cell_resolution)); //(0.1, 0.1));  //0.1 cell resolution is one meter in simulation //LUGV 3m - 3 cell size - 0.3 in grid map

  //sectr mapping stuff
  vector<int>* cartesian_flc_sectors_distance = new vector<int> [6];
  vector<double>* polar_frp_sectors_distance = new vector<double> [9];


  /*
   * --------------------- environment mapping ----------------
   */
  // drawing the mapping model on image //model fitting on image ==> to know about the location of cells
  for (unsigned int i = 0; i < prev_ground_image->height; i++) //height y row  vertical
  {
    for (unsigned int j = 0; j < prev_ground_image->width; j++) //width x column  horizon horizontal
    {

//      //obstacles
//    if (prev_label_image->at(j, i).r == 255 && prev_label_image->at(j, i).g == 0 && prev_label_image->at(j, i).b == 0) //blue unknown //out of range
//    {
//      double gridmap_x = prev_ground_image->at(j, i).x;
//      double gridmap_y = prev_ground_image->at(j, i).z;
//      output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 3; //red obstacle
//    }// if red obstacle

      /*
       * ! if green or traversible
       * */
      if (prev_label_image->at(j, i).r == 0 && prev_label_image->at(j, i).g == 255 && prev_label_image->at(j, i).b == 0)
      {

        //local map in RCS
        double gridmap_x = prev_ground_image->at(j, i).x; //image coordinate
        double gridmap_y = prev_ground_image->at(j, i).z;
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x: ", gridmap_x);
        FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_y: ", gridmap_y);

        // make sure the points are within certain bounds
        if (abs(gridmap_x) < bound_limit && gridmap_y < bound_limit)
        {
          output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 1; //green

//          if (output_gridmap_ptr->GetConstCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) != 3 ){
//            output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 1; //green
//          }

          /*          filling the sectomaps for cartesian
           */
          //flc
          if (gridmap_x > -3 && gridmap_x < -2)
          {
            cartesian_flc_sectors_distance[0].push_back(gridmap_y);
          }
          if (gridmap_x > -2 && gridmap_x < -1)
          {
            cartesian_flc_sectors_distance[1].push_back(gridmap_y);
          }
          if (gridmap_x > -1 && gridmap_x < 0)
          {
            cartesian_flc_sectors_distance[2].push_back(gridmap_y);
          }

          //frc
          if (gridmap_x > 0 && gridmap_x < 1)
          {
            cartesian_flc_sectors_distance[3].push_back(gridmap_y);
          }
          if (gridmap_x > 1 && gridmap_x < 2)
          {
            cartesian_flc_sectors_distance[4].push_back(gridmap_y);
          }
          if (gridmap_x > 2 && gridmap_x < 3)
          {
            cartesian_flc_sectors_distance[5].push_back(gridmap_y);
          }


          /*
           * polar sectors filling
           */

          double gridmap_x_trans = gridmap_x; // - 2.5;
          double gridmap_y_trans = gridmap_y; // - 5;
          double param = (gridmap_y_trans) / (gridmap_x_trans);
          double result;
          //param = 45.0;
          //result = tan(param * PI / 180.0);
          result = atan(param) * 180 / PI;  //degree
          //for (int i = 1; i <;){//          }
          if (result > 0 && result < 10)
          {
            double length = sqrt((gridmap_x_trans * gridmap_x_trans) + (gridmap_y_trans * gridmap_y_trans));
            polar_frp_sectors_distance[0].push_back(length);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x_trans: ", gridmap_x_trans, " gridmap_y_trans: " , gridmap_y_trans);
          }
          if (result > 80 && result < 90)
          {
            double length = sqrt((gridmap_x_trans * gridmap_x_trans) + (gridmap_y_trans * gridmap_y_trans));
            polar_frp_sectors_distance[8].push_back(length);
            FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "gridmap_x_trans: ", gridmap_x_trans, " gridmap_y_trans: " , gridmap_y_trans);
          }

        } //if in limits and bounds
      }// if greeen

      if (prev_label_image->at(j, i).r == 255 && prev_label_image->at(j, i).g == 255 && prev_label_image->at(j, i).b == 0) //yellow
      {
        double gridmap_x = prev_ground_image->at(j, i).x;
        double gridmap_y = prev_ground_image->at(j, i).z;

        // make sure the points are within certain bounds
        if (abs(gridmap_x) < bound_limit && gridmap_y < bound_limit)
        {
          output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(gridmap_x, gridmap_y)) = 2; //yellow
        }
      }// if yellow

    }//for j
  } // for i

  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "width: ", prev_ground_image->width, "----- height: ", prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "ALL IMAGE PIXELS: ", prev_ground_image->width * prev_ground_image->height);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "NUMBER OF CELLS: ", output_gridmap_ptr->GetNumberOfCells());

  /*---------------> 4- pose of robot and environment <----------------------*/
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, 0)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.1)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.2)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.3)) = 0.2; //origin and location of stereo camera -- left one
  output_gridmap_ptr->GetCellByCoordinate(rrlib::math::tVec2d(0, -0.4)) = 0.2; //origin and location of stereo camera -- left one

  //publishing the port
  output_gridmap.Publish(output_gridmap_ptr);



//  int* dist = new int[6];
//  //std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].be, cartesian_flc_sectors_distance[0] + cartesian_flc_sectors_distance[0].size()) << '\n';
//  if (cartesian_flc_sectors_distance[0].size() > 0)
//  {
//    std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[0].begin(), cartesian_flc_sectors_distance[0].end()) << '\n';
//    dist[0] = *std::max_element(cartesian_flc_sectors_distance[0].begin(), cartesian_flc_sectors_distance[0].end());
//  }//if
//  if (cartesian_flc_sectors_distance[1].size() > 0)
//  {
//    std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[1].begin(), cartesian_flc_sectors_distance[1].end()) << '\n';
//    dist[1] = *std::max_element(cartesian_flc_sectors_distance[1].begin(), cartesian_flc_sectors_distance[1].end());
//  }//if
//  if (cartesian_flc_sectors_distance[2].size() > 0)
//  {
//    std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[2].begin(), cartesian_flc_sectors_distance[2].end()) << '\n';
//    dist[2] = *std::max_element(cartesian_flc_sectors_distance[2].begin(), cartesian_flc_sectors_distance[2].end());
//  }//if
//  if (cartesian_flc_sectors_distance[3].size() > 0)
//  {
//    std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[3].begin(), cartesian_flc_sectors_distance[3].end()) << '\n';
//    dist[3] = *std::max_element(cartesian_flc_sectors_distance[3].begin(), cartesian_flc_sectors_distance[3].end());
//  }//if
//  if (cartesian_flc_sectors_distance[4].size() > 0)
//  {
//    std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[4].begin(), cartesian_flc_sectors_distance[4].end()) << '\n';
//    dist[4] = *std::max_element(cartesian_flc_sectors_distance[4].begin(), cartesian_flc_sectors_distance[4].end());
//  }//if
//  if (cartesian_flc_sectors_distance[5].size() > 0)
//  {
//    std::cout << "The max element is " << *std::max_element(cartesian_flc_sectors_distance[5].begin(), cartesian_flc_sectors_distance[5].end()) << '\n';
//    dist[5] = *std::max_element(cartesian_flc_sectors_distance[5].begin(), cartesian_flc_sectors_distance[5].end());
//  }//if
//
//  //polar sector distances
//  double* dist_polar = new double[9];
//  if (polar_frp_sectors_distance[0].size() > 0)
//  {
//    std::cout << "The polar max element in 10 is " << *std::max_element(polar_frp_sectors_distance[0].begin(), polar_frp_sectors_distance[0].end()) << '\n';
//    dist_polar[0] = *std::max_element(polar_frp_sectors_distance[0].begin(), polar_frp_sectors_distance[0].end());
//  }//if
//  if (polar_frp_sectors_distance[8].size() > 0)
//  {
//    std::cout << "The polar max element in 90 is " << *std::max_element(polar_frp_sectors_distance[8].begin(), polar_frp_sectors_distance[8].end()) << '\n';
//    dist_polar[8] = *std::max_element(polar_frp_sectors_distance[8].begin(), polar_frp_sectors_distance[8].end());
//  }//if


}// gridmap

void
mStereoTestColorOffline::sectormap()
{

  //front left cartesian FLC
  typedef finroc::icarus::mapping::tSectorMap<rrlib::mapping::state_space::tCartesian, 2> tSectorMapCartesian2D;
  tSectorMapCartesian2D cartesian;
  rrlib::mapping::tCartesian2D::tCoordinate coordinate(-2.5, 5); //scaled to 10
  tSectorMapCartesian2D::tBounds bounds;
  bounds.lower_bounds[0] = coordinate;
  bounds.lower_bounds[1] = coordinate;
  coordinate += rrlib::mapping::tCartesian2D::tCoordinate(2.5, 10);
  bounds.upper_bounds[0] = coordinate;
  bounds.upper_bounds[1] = coordinate;
  cartesian.SetBounds(bounds);
  coordinate = rrlib::mapping::tCartesian2D::tCoordinate(0.5, 10);
  cartesian.SetResolution(coordinate);
  //  RRLIB_LOG_PRINT( WARNING, cartesian.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, cartesian.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, cartesian.GetBounds(), " FLC bounds and number of cells:", cartesian.GetNumberOfCells());

  cartesian.position = mapping::tSectorMapPosition::eFRONT_LEFT_CARTESIAN;
  cartesian.status = mapping::tSectorMapStatus::eSECTOR_MAP_STATUS_VALID;
  cartesian.origin = rrlib::math::tVec3d(-2.5, 5, 0);

  cartesian.GetCellByCellID(1).distance_to_obstacle = 8 ;
  cartesian.GetCellByCellID(1).id = 1;
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, cartesian.GetCoordinateByCellID(0), cartesian.GetCoordinateByCellID(1),
                   cartesian.GetCoordinateByCellID(2), cartesian.GetCoordinateByCellID(3), cartesian.GetCoordinateByCellID(4));
  cartesian.GetCellByCellID(1).quality = 1;
  cartesian.GetCellByCellID(1).status = mapping::tSectorStatus::eSECTOR_STATUS_VALID_RELEVANT;
  cartesian.GetCellByCellID(1).timestamp_last_update = rrlib::time::cNO_TIME;



  //front right cartesian
  tSectorMapCartesian2D cartesian_frc;
  rrlib::mapping::tCartesian2D::tCoordinate coordinate_frc(0, 5); //scaled to 10
  tSectorMapCartesian2D::tBounds bounds_frc;
  bounds_frc.lower_bounds[0] = coordinate_frc;
  bounds_frc.lower_bounds[1] = coordinate_frc;
  coordinate_frc += rrlib::mapping::tCartesian2D::tCoordinate(2.5, 10);
  bounds_frc.upper_bounds[0] = coordinate_frc;
  bounds_frc.upper_bounds[1] = coordinate_frc;
  cartesian_frc.SetBounds(bounds_frc);
  coordinate_frc = rrlib::mapping::tCartesian2D::tCoordinate(0.5, 10);
  cartesian_frc.SetResolution(coordinate_frc);
  //  RRLIB_LOG_PRINT( WARNING, cartesian.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, cartesian_frc.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, cartesian_frc.GetBounds(), " FLC bounds and number of cells:", cartesian_frc.GetNumberOfCells());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, cartesian_frc.GetCoordinateByCellID(0), cartesian_frc.GetCoordinateByCellID(1),
                   cartesian_frc.GetCoordinateByCellID(2), cartesian_frc.GetCoordinateByCellID(3), cartesian_frc.GetCoordinateByCellID(4));



  //  RRLIB_LOG_PRINT( ERROR, "Polar");
  typedef finroc::icarus::mapping::tSectorMap<rrlib::mapping::state_space::tPolar, 2> tSectorMapPolar2D;

  //front right polar FRP
  tSectorMapPolar2D polar;
  tSectorMapPolar2D::tBounds bounds_polar;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar(0, 0);
  bounds_polar.lower_bounds[0] = coordinate_polar;
  bounds_polar.lower_bounds[1] = coordinate_polar;
  coordinate_polar += rrlib::mapping::tPolar2D::tCoordinate(90, 10);
  bounds_polar.upper_bounds[0] = coordinate_polar;
  bounds_polar.upper_bounds[1] = coordinate_polar;
  polar.SetBounds(bounds_polar);
  //  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(10, 5); //test to make sure about overwriting the legth of sectors
  coordinate_polar = rrlib::mapping::tPolar2D::tCoordinate(10, 10);
  polar.SetResolution(coordinate_polar);
  //  RRLIB_LOG_PRINT(WARNING, polar.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, polar.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, polar.GetNumberOfCells(), "bounds frp", polar.GetBounds());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, polar.GetCoordinateByCellID(0), polar.GetCoordinateByCellID(1),
                   polar.GetCoordinateByCellID(2), polar.GetCoordinateByCellID(3), polar.GetCoordinateByCellID(4), polar.GetCoordinateByCellID(8));

  //front left polar FLP
  tSectorMapPolar2D polar_flp;
  tSectorMapPolar2D::tBounds bounds_polar_flp;
  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar_flp(90, 0);
  bounds_polar_flp.lower_bounds[0] = coordinate_polar_flp;
  bounds_polar_flp.lower_bounds[1] = coordinate_polar_flp;
  coordinate_polar_flp += rrlib::mapping::tPolar2D::tCoordinate(180, 10);
  bounds_polar_flp.upper_bounds[0] = coordinate_polar_flp;
  bounds_polar_flp.upper_bounds[1] = coordinate_polar_flp;
  polar_flp.SetBounds(bounds_polar_flp);
  coordinate_polar_flp = rrlib::mapping::tPolar2D::tCoordinate(10, 10);
  polar_flp.SetResolution(coordinate_polar_flp);
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, polar_flp.GetResolution());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, polar_flp.GetNumberOfCells(), "bounds flp", polar_flp.GetBounds());
  FINROC_LOG_PRINT(DEBUG_VERBOSE_1, polar_flp.GetCoordinateByCellID(0), polar_flp.GetCoordinateByCellID(1),
                   polar_flp.GetCoordinateByCellID(2), polar_flp.GetCoordinateByCellID(3), polar_flp.GetCoordinateByCellID(4), polar_flp.GetCoordinateByCellID(8));


//  //rear left polar RLP
//  tSectorMapPolar2D polar_rlp;
//  tSectorMapPolar2D::tBounds bounds_polar_rlp;
//  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar_rlp(180, 0);
//  bounds_polar_rlp.lower_bounds[0] = coordinate_polar_rlp;
//  bounds_polar_rlp.lower_bounds[1] = coordinate_polar_rlp;
//  coordinate_polar_rlp += rrlib::mapping::tPolar2D::tCoordinate(270, 10);
//  bounds_polar_rlp.upper_bounds[0] = coordinate_polar_rlp;
//  bounds_polar_rlp.upper_bounds[1] = coordinate_polar_rlp;
//  polar_rlp.SetBounds(bounds_polar_rlp);
//  coordinate_polar_rlp = rrlib::mapping::tPolar2D::tCoordinate(10, 10);
//  polar_rlp.SetResolution(coordinate_polar_rlp);
//  FINROC_LOG_PRINT(DEBUG, polar_rlp.GetResolution());
//  FINROC_LOG_PRINT(DEBUG, polar_rlp.GetNumberOfCells(), "bounds rlp", polar_rlp.GetBounds());
//
//  //rear right polar RRP
//  tSectorMapPolar2D polar_rrp;
//  tSectorMapPolar2D::tBounds bounds_polar_rrp;
//  rrlib::mapping::tPolar2D::tCoordinate coordinate_polar_rrp(-90, 0);
//  bounds_polar_rrp.lower_bounds[0] = coordinate_polar_rrp;
//  bounds_polar_rrp.lower_bounds[1] = coordinate_polar_rrp;
//  coordinate_polar_rrp += rrlib::mapping::tPolar2D::tCoordinate(0, 10);
//  bounds_polar_rrp.upper_bounds[0] = coordinate_polar_rrp;
//  bounds_polar_rrp.upper_bounds[1] = coordinate_polar_rrp;
//  polar_rrp.SetBounds(bounds_polar_rrp);
//  coordinate_polar_rrp = rrlib::mapping::tPolar2D::tCoordinate(10, 10);
//  polar_rrp.SetResolution(coordinate_polar_rrp);
//  FINROC_LOG_PRINT(DEBUG, polar_rrp.GetResolution());
//  FINROC_LOG_PRINT(DEBUG, polar_rrp.GetNumberOfCells(), "bounds rrp", polar_rrp.GetBounds());

}//sector map

//----------------------------------------------------------------------
// mStereoTestColorOffline destructor
//----------------------------------------------------------------------
mStereoTestColorOffline::~mStereoTestColorOffline()
{}

//----------------------------------------------------------------------
// mStereoTestColorOffline OnStaticParameterChange
//----------------------------------------------------------------------
void mStereoTestColorOffline::OnStaticParameterChange()
{}

//----------------------------------------------------------------------
// mStereoTestColorOffline OnParameterChange
//----------------------------------------------------------------------
void mStereoTestColorOffline::OnParameterChange()
{}

//----------------------------------------------------------------------
// mStereoTestColorOffline Update
//----------------------------------------------------------------------
void mStereoTestColorOffline::Update()
{
  if (this->InputChanged())
  {
    //At least one of your input ports has changed. Do something useful with its data.
    //However, using the .HasChanged() method on each port you can check in more detail.
  }

  //Do something each cycle independent from changing ports.
  run();
  gridmap();
  sectormap();

  //this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
}
