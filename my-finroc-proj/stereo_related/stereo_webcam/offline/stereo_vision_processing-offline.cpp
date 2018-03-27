/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

// system files - system installation
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
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

// using namespace
using namespace std;
using namespace pcl;
using namespace cv; //opencv

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
  // stereo capture, rectification initialization
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename;
  string input_extrinsic_filename ;
  Mat left_rect_png_img;
  Mat right_rect_png_img;

  pcl::AdaptiveCostSOStereoMatching stereo;

  // save cloud parameters
  pcl::PCDWriter writer_;
  std::string file_name_;
  std::string dir_name_;
  unsigned format_;



public:
  StereoVisionProcessing(const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                         const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename) :
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
    image_viewer_disparity(new visualization::ImageViewer("Image Viewer Disparity")),
    prev_cloud(new Cloud),
    prev_normal_cloud(new pcl::PointCloud<pcl::Normal>),
    prev_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    prev_ground_image(new Cloud),
    prev_label_image(new Cloud),
    road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
    road_segmentation(road_comparator),
    writer_()
    , file_name_("frames_pcd")
    , dir_name_("frames")
    , format_(4)
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

    // Set up a 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
//    viewer->addCoordinateSystem(1.0); //default
    double scale = 2.0;
    float x = 0.0; //0
    float y = 0.5; //0.5
    float z = 1.0 ; //1
    int viewport = 0;
//      Adds 3D axes describing a coordinate system to screen at x, y, z.
    viewer->addCoordinateSystem(scale, x, y, z, viewport);
    viewer->initCameraParameters();
    viewer->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);

    // fede susgested for AdaptiveCostSOStereoMatching
    stereo.setMaxDisparity(60);
    stereo.setXOffset(0);
    stereo.setRadius(5);
    stereo.setSmoothWeak(25);
    stereo.setSmoothStrong(100);
    stereo.setGammaC(25);
    stereo.setGammaS(5);
//        stereo.setRatioFilter(15);
//        stereo.setPeakFilter(0);
//        stereo.setLeftRightCheck(true);
//        stereo.setLeftRightCheckThreshold(1);
    stereo.setPreProcessing(true);

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
//          case '1':
//            smooth_strong -= 10;
//            PCL_INFO ("smooth_strong: %d\n", smooth_strong);
//            stereo.setSmoothStrong (smooth_strong);
//            break;
//          case '2':
//            smooth_strong += 10;
//            PCL_INFO ("smooth_strong: %d\n", smooth_strong);
//            stereo.setSmoothStrong (smooth_strong);
//            break;
//          case '3':
//            smooth_weak -= 10;
//            PCL_INFO ("smooth_weak: %d\n", smooth_weak);
//            stereo.setSmoothWeak (smooth_weak);
//            break;
//          case '4':
//            smooth_weak += 10;
//            PCL_INFO ("smooth_weak: %d\n", smooth_weak);
//            stereo.setSmoothWeak (smooth_weak);
//            break;
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
  stereo_rectify(const string input_png_images_left, const string input_png_images_right)
  {

    // showing the input images
    cout << "stereo_rectify -------> input_png_images_left: " << input_png_images_left << endl;
    cout << "stereo_rectify -------> input_png_images_right: " << input_png_images_right << endl;

    //variable initialization
    const char* img1_filename = (char*) input_png_images_left.c_str();
    const char* img2_filename = (char*) input_png_images_right.c_str();
    const char* intrinsic_filename = (char*) input_intrinsic_filename.c_str();
    const char* extrinsic_filename = (char*) input_extrinsic_filename.c_str();

    // reading png images
    int color_mode = 0; //grayscale
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
    Size imageSize = img1.size();


    //
    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;



    // save intrinsic parameters
//        FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if (fs.isOpened())
    {
//            fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
//                "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
      Mat M1, D1, M2, D2;
      fs["M1"] >> cameraMatrix[0];
      fs["D1"] >> distCoeffs[0];
      fs["M2"] >> cameraMatrix[1];
      fs["D2"] >> distCoeffs[1];

      fs.release();
    }
    else
      cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

//        fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    fs.open(extrinsic_filename, CV_STORAGE_READ);
    if (fs.isOpened())
    {
//            fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//        Mat R, T, R1, P1, R2, P2;
      fs["R"] >> R;
      fs["T"] >> T;
      fs.release();
    }
    else
      cout << "Error: can not save the intrinsic parameters\n";

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);


    //StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true);
    bool useCalibrated = true;
    bool showRectified = true;

    // COMPUTE AND DISPLAY RECTIFICATION
    if (!showRectified)
      return;

    Mat rmap[2][2];
//    // IF BY CALIBRATED (BOUGUET'S METHOD)
//        if( useCalibrated )
//        {
//            // we already computed everything
//        }
//    // OR ELSE HARTLEY'S METHOD
//        else
//     // use intrinsic parameters of each camera, but
//     // compute the rectification transformation directly
//     // from the fundamental matrix
//        {
//            vector<Point2f> allimgpt[2];
//            for( k = 0; k < 2; k++ )
//            {
//                for( i = 0; i < nimages; i++ )
//                    std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
//            }
//            F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
//            Mat H1, H2;
//            stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
//
//            R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
//            R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
//            P1 = cameraMatrix[0];
//            P2 = cameraMatrix[1];
//        }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    // OpenCV can handle left-right or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
    if (!isVerticalStereo)
    {
      sf = 600. / MAX(imageSize.width, imageSize.height);
      w = cvRound(imageSize.width * sf);
      h = cvRound(imageSize.height * sf);
      canvas.create(h, w * 2, CV_8UC3);
    }
    else
    {
      sf = 300. / MAX(imageSize.width, imageSize.height);
      w = cvRound(imageSize.width * sf);
      h = cvRound(imageSize.height * sf);
      canvas.create(h * 2, w, CV_8UC3);
    }
    Mat img_rect[2];
    for (int k = 0; k < 2; k++)
    {
      Mat rimg, cimg;
      remap(img[k], rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
      cvtColor(rimg, cimg, CV_GRAY2BGR);
      Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
      resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
      if (useCalibrated)  //Bouguet's method - not hartley
      {
        Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
                  cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
        rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
      }
      rimg.copyTo(img_rect[k]);
    }

    if (!isVerticalStereo)
      for (int j = 0; j < canvas.rows; j += 16)
        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
    else
      for (int j = 0; j < canvas.cols; j += 16)
        line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
    imshow("rectified_true", canvas);

    // png2pcd
    img_rect[0].copyTo(left_rect_png_img);
    img_rect[1].copyTo(right_rect_png_img);
    namedWindow("left_rect", 0);
    imshow("left_rect", left_rect_png_img);
    namedWindow("right_rect", 0);
    imshow("right_rect", right_rect_png_img);

//    // saving the rectified frames in right and left folders
//    char filename[100];
//    sprintf(filename, "rect/left/left_rect%.2d.png", images_idx);
//    imwrite(filename, left_rect_png_img);
//    cout << "Saved " << filename << endl;
//    sprintf(filename, "rect/right/right_rect%.2d.png", images_idx);
//    imwrite(filename, right_rect_png_img);
//    cout << "Saved " << filename << endl;
//    sprintf(filename, "rect/stereo/stereo_rect%.2d.png", images_idx);
//    imwrite(filename, canvas);
//    cout << "saved " << filename << endl;


  }// stereo_rectify_test

  PointCloud<RGB>::Ptr
  png2pcd(const Mat rect_png_image)
  {

    Mat image;
    rect_png_image.copyTo(image);

    /// Convert the image from Gray to RGB
    cvtColor(image, image, CV_GRAY2BGR);

    // Retrieve the entries from the image data and copy them into the output RGB cloud
    double* pixel = new double [4];
    memset(pixel, 0, sizeof(double) * 4);
    PointCloud<RGB> cloud;

    // cloud initialization
    cloud.width = image.cols;
    cloud.height = image.rows; // This indicates that the point cloud is organized
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    // png to pcd loop
    for (int y = 0; y < image.rows; y++)
    {
      for (int x = 0; x < image.cols; x++)
      {
        pixel[0] = (double) image.at<Vec3b>(y, x)[0];
        RGB color;
        color.r = 0;
        color.g = 0;
        color.b = 0;
        color.a = 0;
        color.rgb = 0.0f;
        color.rgba = 0;

        int rgb;
        int rgba;

//              //case 1: // component ==1
        color.r = static_cast<uint8_t>(pixel[0]);
        color.g = static_cast<uint8_t>(pixel[0]);
        color.b = static_cast<uint8_t>(pixel[0]);

        rgb = (static_cast<int>(color.r)) << 16 |
              (static_cast<int>(color.g)) << 8 |
              (static_cast<int>(color.b));

        rgba = rgb;
        color.rgb = static_cast<float>(rgb);
        color.rgba = static_cast<uint32_t>(rgba);

        // Point<RGB> (x,y) = RGB
//                cloud (x, image.rows - y - 1) = color;
        cloud(x, y) = color;

      } //for x
    }// for y

    // free memory
    delete[] pixel;

    // output pcd
    // conversion from PointCloud<RGB> to PointCloud<RGB>::Ptr
    PointCloud<RGB>::Ptr cloud_output(new PointCloud<RGB>);
    *cloud_output = cloud;

    return cloud_output;
  }// png2pcd

  void
  processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
  {

    stereo.compute(*left_image, *right_image);
//    stereo.medianFilter (4); //post processing
    stereo.medianFilter(10);  // better but slower //optimal
//    stereo.medianFilter (20); // slow //better in some but worse in some other one - depends on disp map
//    stereo.medianFilter (30); //too slow

    // stereo images camera calibration parameters
    float u_c = 379.85181427001953f; // the main one 3.7985181427001953e+02; // pcl //opencv - Cx = 3.8749671936035156e+02;
    float v_c = 305.85922241210938f; // pcl //opencv - Cy = 3.1611055755615234e+02;
    float focal = 920.38355542932538f; //9.7814703939910510e+02;  //f
//    float baseline = abs ( -3.3068892344473029e+03 / 9.2038355542932538e+02 ); //abs(3.6043745150993091e+03 / 9.7814703939910510e+02); //Tx //horizontal baseline //shift between cameras
    float baseline = 0.359294689f; //real one using calculator
    CloudPtr cloud = out_cloud;
    pcl::PointCloud<pcl::RGB>::Ptr texture = left_image;
    stereo.getPointCloud(u_c, v_c, focal, baseline, cloud, texture);

  }// processPair

  void
  processCloud(const pcl::PointCloud<PointT>::ConstPtr& cloud)
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_image(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr label_image(new pcl::PointCloud<pcl::PointXYZRGB>);
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

    // green for traverable terrain
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
        }// for j

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
        } //if

        model.values[0] = plane_params[0];
        model.values[1] = plane_params[1];
        model.values[2] = plane_params[2];
        model.values[3] = plane_params[3];
        model_coefficients.push_back(model);
        inlier_indices.push_back(region_indices[i]);
        centroids.push_back(clust_centroid);
        covariances.push_back(clust_cov);
      } // if
    } // for i

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
    pcl::PointCloud<PointT> extended_ground_cloud;
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

    // Segment Obstacles (Disabled by default)
    Eigen::Vector4f ground_plane_params(1.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f ground_centroid(0.0, 0.0, 0.0, 0.0);

    if (ground_cloud->points.size() > 0)
    {
      ground_centroid = centroids[0];
      ground_plane_params = Eigen::Vector4f(model_coefficients[0].values[0], model_coefficients[0].values[1], model_coefficients[0].values[2], model_coefficients[0].values[3]);
    }

    if (detect_obstacles)
    {
      pcl::PointCloud<PointT>::CloudVectorType clusters;
      if (ground_cloud->points.size() > 0)
      {
        std::vector<bool> plane_labels;
        plane_labels.resize(region_indices.size(), false);
        for (size_t i = 0; i < region_indices.size(); i++)
        {
          if (region_indices[i].indices.size() > mps.getMinInliers())
          {
            plane_labels[i] = true;
          }
        }

        pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

        euclidean_cluster_comparator_->setInputCloud(cloud);
        euclidean_cluster_comparator_->setLabels(labels_ptr);
        euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold(0.05f, false);

        pcl::PointCloud<pcl::Label> euclidean_labels;
        std::vector<pcl::PointIndices> euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud(cloud);
        euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

        for (size_t i = 0; i < euclidean_label_indices.size(); i++)
        {
          if ((euclidean_label_indices[i].indices.size() > 200))
          {
            pcl::PointCloud<PointT> cluster;
            pcl::copyPointCloud(*cloud, euclidean_label_indices[i].indices, cluster);
            clusters.push_back(cluster);

            Eigen::Vector4f cluster_centroid;
            Eigen::Matrix3f cluster_cov;
            pcl::computeMeanAndCovarianceMatrix(*cloud, euclidean_label_indices[i].indices, cluster_cov, cluster_centroid);

            pcl::PointXYZ centroid_pt(cluster_centroid[0], cluster_centroid[1], cluster_centroid[2]);
            double ptp_dist =  pcl::pointToPlaneDistanceSigned(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

            if ((ptp_dist > 0.5) && (ptp_dist < 3.0))
            {

              for (int j = 0; j < euclidean_label_indices[i].indices.size(); j++)
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
    for (int i = 0; i < cloud->points.size(); i++)
    {
      if (!pcl::isFinite(cloud->points[i]))
      {
        ground_image->points[i].b = static_cast<uint8_t>((cloud->points[i].b + 255) / 2);
        label_image->points[i].r = 0;
        label_image->points[i].g = 0;
        label_image->points[i].b = 255;
      }
    }

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
  saveCloud(const CloudConstPtr& cloud)
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

  void
  run()
  {
    while (!viewer->wasStopped())
    {

      // exiting criteria
      if (img_pairs_num == images_idx)
      {
        cout << "img_pairs_num == images_idx -> " << images_idx << std::endl;
        break;
      } //if

      // Proceed or nor to the next image
      if (trigger || continuous)
      {


        // offline
        stereo_rectify(left_images[images_idx], right_images[images_idx]);
//            stereo_rectify_false_scale(left_images[images_idx], right_images[images_idx]);
//            stereo_rectify_false_old(left_images[images_idx], right_images[images_idx]);

        pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
        pcl::PointCloud<pcl::RGB>::Ptr right_cloud(new pcl::PointCloud<pcl::RGB>);
        CloudPtr out_cloud(new Cloud);

        //png2pcd -- mat img input
        left_cloud = png2pcd(left_rect_png_img);
        right_cloud = png2pcd(right_rect_png_img);

        // process the disparity map and point cloud
        processStereoPair(left_cloud, right_cloud, out_cloud);
        saveCloud(out_cloud);
        processCloud(out_cloud);

        ///displaying disparity map
        pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
        stereo.getVisualMap(vmap);
        image_viewer_disparity->addRGBImage<pcl::RGB> (vmap);


        images_idx++;
        trigger = false; //
      }// if trigger


      // showing the input images
      cout << "left_images[img_index]: " << left_images[images_idx - 1] << std::endl;
      cout << "right_images[img_index]: " << right_images[images_idx - 1] << std::endl;
      cout << "images_idx: " << images_idx - 1 << endl;
      cout << "img_pairs_num: " << img_pairs_num << endl;

      // Draw visualizations
      if (cloud_mutex.try_lock())
      {
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

        cloud_mutex.unlock();
      } //if cloud_mutex

      viewer->spinOnce(1);
      image_viewer->spinOnce(1);
      image_viewer_disparity->spinOnce(1);

      // key to exit or continiue -used with stereo_rectify - used for showing images too
      cout << "press esc to exit or other keys to continue................. " << endl;
      char key = waitKey(1);
      if (key == 27) //esc
        break;

    } // while
  }// run

};//class

int
main(int argc, char** argv)
{

  if (argc < 3)
  {
    PCL_INFO("usage: stereo_vision_processing_offline left_image_directory right_image_directory intrinsic_parameter_filename extrinsic_parameter_filename\n");
  }

  // variable initial
  int img_number_left = 0, img_number_right = 0 ;
  int img_pairs_num = 0;

  // Get list of stereo files from left folder
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back(itr->path().string());
    img_number_left++;
  }
  sort(left_images.begin(), left_images.end());

  // reading right images from folder
  std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr(argv[2]); itr != end_itr; ++itr)
  {
    right_images.push_back(itr->path().string());
    img_number_right++;
  }
  sort(right_images.begin(), right_images.end());
  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");

  // showing the input images
  cout << "img_number_left: " << img_number_left << std::endl;
  cout << "img_number_right: " << img_number_right << std::endl;
  if (img_number_left == img_number_right)
    img_pairs_num = img_number_left;

  // calibration parameters for rectification
  string input_intrinsic_filename = argv[3];
  string input_extrinsic_filename = argv[4];

  // Process and display
  StereoVisionProcessing stereo_vision_processing(left_images, right_images, img_pairs_num, input_intrinsic_filename, input_extrinsic_filename);
  stereo_vision_processing.run();

  return 0;
}// main
