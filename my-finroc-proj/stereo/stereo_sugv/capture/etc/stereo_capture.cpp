/** \brief StereoVisioProcessing is an application for processing stereo images to estimate terrain traversability using stereo camera to detect traversable surfaces.
  *
  * \author Aras Dargazany
  */

// system files
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp> //stereo rectify
#include <opencv2/imgproc/imgproc.hpp> //stereo_rectify
#include <FlyCapture2.h>
#include <pcl/io/io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h> //computeMeanAndCovarianceMatrix
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h> //planeRefinementComparator

//#include <pcl/filters/passthrough.h> //segfault
//#include <pcl/sample_consensus/sac_model_plane.h> //pointToPlaneDistance //seg fault

#include <pcl/visualization/image_viewer.h> //vtk and boost_filesystem required
#include <pcl/visualization/pcl_visualizer.h>

#include "projects/icarus/sensor_processing/libstereo/stereo_matching.h"

#include <iostream>

using namespace cv;
using namespace FlyCapture2;
using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class StereoVisionProcessing
{
private:

  //stereo capture
  BusManager bus_manager;
  PGRGuid pointgrey_guid;
  //Camera camera;
  GigECamera camera;
  Image flycap_img;
  //Camera camera_2;
  GigECamera camera_2;
  Image flycap_img_2;

  // convertImageToOpenCV
  Image colorImage;
  bool bInitialized = false;

  // changing the resolution of images
  GigEImageSettings imageSettings;

  //saving images
  int images_idx = 0;

  //stereo_rectify TODO
  string input_intrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/intrinsics.yml", //aras@pasithee:~/stereo_images/stereo_sugv/2_on-sugv/calib_8_good
         input_extrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/extrinsics.yml";
  Mat input_images_left, input_images_right;
  Mat left_img_rect, right_img_rect;

  //stereo matching
  pcl::AdaptiveCostSOStereoMatching stereo;
  int smooth_weak, smooth_strong;

  //processCloud
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::GroundPlaneComparator<PointT, pcl::Normal>::Ptr road_comparator;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> road_segmentation; //error and seg after that???
  CloudConstPtr prev_cloud;
  CloudConstPtr prev_ground_image;
  CloudConstPtr prev_label_image;
  pcl::PointCloud<pcl::Normal>::ConstPtr prev_normal_cloud;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr prev_ground_cloud;

  boost::mutex cloud_mutex; //this is prolly included in visualization library

  Eigen::Vector4f prev_ground_normal; //seg fault in finroc??
  Eigen::Vector4f prev_ground_centroid;

  //  boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_disparity;
  pcl::visualization::ImageViewer::Ptr image_viewer_disparity;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::visualization::ImageViewer::Ptr image_viewer;

public:
  StereoVisionProcessing():
    road_comparator(new pcl::GroundPlaneComparator<PointT, pcl::Normal>),
    road_segmentation(road_comparator),
    image_viewer_disparity(new pcl::visualization::ImageViewer("Image Viewer Disparity")),
    image_viewer(new pcl::visualization::ImageViewer("Image Viewer")),
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
  {

    //changing the resolution
    //GigEImageSettings imageSettings;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.height = 900; //480;  //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
    imageSettings.width = 1200; //640; //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
    imageSettings.pixelFormat = PIXEL_FORMAT_MONO8;

    //stereo_capture -- RunSingleCamera(pointgrey_guid);
    bus_manager.GetCameraFromIndex(0, &pointgrey_guid); //left eth1
    camera.Connect(&pointgrey_guid);
    camera.SetGigEImageSettings(&imageSettings);
    camera.StartCapture();

    bus_manager.GetCameraFromIndex(1, &pointgrey_guid); //right eth2
    camera_2.Connect(&pointgrey_guid);
    camera_2.SetGigEImageSettings(&imageSettings);
    camera_2.StartCapture();

    /*! processStereoPair - stereo matching - AdaptiveCostSOStereoMatching initialization*/
    stereo.setMaxDisparity(60); //original
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

    // run and keyboard callback
    // Set up a 3D viewer
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
//    viewer->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);
    //display_normals = false;

    /* processCloud -- Set up the normal estimation based on kinematic capability and the terrain for GroundPlaneComparator*/
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor(0.03f);
    ne.setNormalSmoothingSize(40.0f); //20.0f
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);

    bool use_depth_dependent_smoothing = false;
    ne.setDepthDependentSmoothing(use_depth_dependent_smoothing);

    Eigen::Vector3f nominal_road_normal(0.0, -1.0, 0.0); //-1 default  x,y,z --> (z is depth here)
    Eigen::Vector3f tilt_road_normal = Eigen::AngleAxisf(pcl::deg2rad(5.0f), Eigen::Vector3f::UnitX()) * nominal_road_normal;
    road_comparator->setExpectedGroundNormal(tilt_road_normal);

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

  ~StereoVisionProcessing()
  {
    // Stop capturing images
    camera.StopCapture();
    camera_2.StopCapture();

    // Disconnect the camera
    camera.Disconnect();
    camera_2.Disconnect();
  }

  IplImage*
  ConvertImageToOpenCV(Image* pImage)
  {
    IplImage* cvImage = NULL;
    bool bColor = true;
    CvSize mySize;
    mySize.height = pImage->GetRows();
    mySize.width = pImage->GetCols();

    switch (pImage->GetPixelFormat())
    {
    case PIXEL_FORMAT_MONO8:
      cvImage = cvCreateImageHeader(mySize, 8, 1);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 1;
      bColor = false;
      break;
    case PIXEL_FORMAT_411YUV8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_422YUV8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_444YUV8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_RGB8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_MONO16:
      cvImage = cvCreateImageHeader(mySize, 16, 1);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 1;
      bColor = false;
      break;
    case PIXEL_FORMAT_RGB16:
      cvImage = cvCreateImageHeader(mySize, 16, 3);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_S_MONO16:
      cvImage = cvCreateImageHeader(mySize, 16, 1);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 1;
      bColor = false;
      break;
    case PIXEL_FORMAT_S_RGB16:
      cvImage = cvCreateImageHeader(mySize, 16, 3);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_RAW8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_RAW16:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_MONO12:
      printf("Not supported by OpenCV");
      bColor = false;
      break;
    case PIXEL_FORMAT_RAW12:
      printf("Not supported by OpenCV");
      break;
    case PIXEL_FORMAT_BGR:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_BGRU:
      cvImage = cvCreateImageHeader(mySize, 8, 4);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 4;
      break;
    case PIXEL_FORMAT_RGBU:
      cvImage = cvCreateImageHeader(mySize, 8, 4);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 4;
      break;
    default:
      printf("Some error occured...\n");
      return NULL;
    }

    if (bColor)
    {
      if (!bInitialized)
      {
        colorImage.SetData(new unsigned char[pImage->GetCols() * pImage->GetRows() * 3], pImage->GetCols() * pImage->GetRows() * 3);
        bInitialized = true;
      }

      pImage->Convert(PIXEL_FORMAT_BGR, &colorImage); //needs to be as BGR to be saved

      cvImage->width = colorImage.GetCols();
      cvImage->height = colorImage.GetRows();
      cvImage->widthStep = colorImage.GetStride();

      cvImage->origin = 0; //interleaved color channels

      cvImage->imageDataOrigin = (char*)colorImage.GetData(); //DataOrigin and Data same pointer, no ROI
      cvImage->imageData         = (char*)(colorImage.GetData());
      cvImage->widthStep    = colorImage.GetStride();
      cvImage->nSize = sizeof(IplImage);
      cvImage->imageSize = cvImage->height * cvImage->widthStep;
    }
    else
    {
      cvImage->imageDataOrigin = (char*)(pImage->GetData());
      cvImage->imageData         = (char*)(pImage->GetData());
      cvImage->widthStep         = pImage->GetStride();
      cvImage->nSize             = sizeof(IplImage);
      cvImage->imageSize         = cvImage->height * cvImage->widthStep;

      //at this point cvImage contains a valid IplImage
    }
    return cvImage;
  }

  void
  stereo_capture()
  {

    camera.RetrieveBuffer(&flycap_img);
    camera_2.RetrieveBuffer(&flycap_img_2);
    IplImage* left_IplImage   = ConvertImageToOpenCV(&flycap_img);
    IplImage* right_IplImage = ConvertImageToOpenCV(&flycap_img_2);

//    //only for testing
//    cvShowImage("left IplImage opencv", left_IplImage);
//    cvShowImage("right IplImage opencv", right_IplImage);

    Mat left_img_opencv(left_IplImage),
        right_img_opencv(right_IplImage);//IplImage * ipl = ...; //cv::Mat m = cv::cvarrToMat(ipl);  // default additional arguments: don't copy data.

    cvReleaseImageHeader(&left_IplImage);
    cvReleaseImage(&left_IplImage);

    //pass the images to stereo_rectify
    left_img_opencv.copyTo(input_images_left);
    right_img_opencv.copyTo(input_images_right);

    /*
        // saving the frames in right and left folders
        //TODO also creating left and right folders automatically
        char filename[100];
        sprintf(filename, "left/left%.4d.png", images_idx);
        imwrite(filename, left_img_opencv);
        cout << "Saved " << filename << endl;
        sprintf(filename, "right/right%.4d.png", images_idx);
        imwrite(filename, right_img_opencv);
        cout << "Saved " << filename << endl;
    */

  } // stereo_capture

  void stereo_rectify()
  {
    const char* intrinsic_filename = (char*) input_intrinsic_filename.c_str();
    const char* extrinsic_filename = (char*) input_extrinsic_filename.c_str();

    Mat img1 ;
    Mat img2 ;
    input_images_left.copyTo(img1);
    input_images_right.copyTo(img2);

    //  namedWindow("left", 0);
    //  imshow("left", img1);
    //  namedWindow("right", 0);
    //  imshow("right", img2);
    //  waitKey(1);

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;

    // save intrinsic parameters
    FileStorage fs(intrinsic_filename, CV_STORAGE_READ);     //FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    if (fs.isOpened())
    {
      //fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
      //"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
      //Mat M1, D1, M2, D2;
      fs["M1"] >> cameraMatrix[0];
      fs["D1"] >> distCoeffs[0];
      fs["M2"] >> cameraMatrix[1];
      fs["D2"] >> distCoeffs[1];

      fs.release();
    }
    else
      cout << "Error: can not read the intrinsic parameters\n";

    fs.open(extrinsic_filename, CV_STORAGE_READ);     //fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    if (fs.isOpened())
    {
      //fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
      //Mat R, T, R1, P1, R2, P2;
      fs["R"] >> R;
      fs["T"] >> T;
      fs.release();
    }
    else
      cout << "Error: can not read the extrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
    Size imageSize = img1.size();

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    Mat rmap[2][2];

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

    // Mat img = imread(goodImageList[i*2+k]
    Mat img[2];
    img1.copyTo(img[0]);
    img2.copyTo(img[1]);

    Mat img_rect[2];

    //StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true);
    bool useCalibrated = true;


    for (int k = 0; k < 2; k++)
    {
      Mat rimg, cimg;
      remap(img[k], rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
      cvtColor(rimg, cimg, CV_GRAY2BGR);
      //Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
      Mat canvasPart = canvas(Rect(w * k, 0, w, h)) ;
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


    //  imshow("rectified_true", canvas);

    //  namedWindow("left_rect", 0);
    //  imshow("left_rect", img_rect[0]);
    //  namedWindow("right_rect", 0);
    //  imshow("right_rect", img_rect[1]);
    //  waitKey(1);

    /*cropping based on the smallest common region between rectified left and right*/
    Mat img_rect_color[2];
    Mat img_rect_color_original[2];


    for (int k = 0; k < 2; k++)
    {
      cvtColor(img_rect[k], img_rect_color[k], CV_GRAY2BGR);
      cvtColor(img_rect[k], img_rect_color_original[k], CV_GRAY2BGR);


      Rect vroi(cvRound(validRoi[k].x), cvRound(validRoi[k].y),
                cvRound(validRoi[k].width), cvRound(validRoi[k].height));
      rectangle(img_rect_color[k], vroi, Scalar(0, 0, 255), 3, 8);
    }

    //    namedWindow("left_rect_test", 0);
    //    imshow("left_rect_test", img_rect_color[0]);
    //    namedWindow("right_rect_test", 0);
    //    imshow("right_rect_test", img_rect_color[1]);

    Mat img_rect_color_cropped[2];
    img_rect_color_cropped[0] = img_rect_color[0](validRoi[0]);
    img_rect_color_cropped[1] = img_rect_color[1](validRoi[1]);

    //    namedWindow("left_rect_test_cropped", 0);
    //    imshow("left_rect_test_cropped", img_rect_color_cropped[0]);
    //    namedWindow("right_rect_test_cropped", 0);
    //    imshow("right_rect_test_cropped", img_rect_color_cropped[1]);


    int x_top_left_max, y_top_left_max;
    int x_bottom_right_min, y_bottom_right_min;

    /* top left max point x and y*/
    if (validRoi[0].x > validRoi[1].x)
    {
      x_top_left_max = validRoi[0].x;
    }
    else
    {
      x_top_left_max = validRoi[1].x;
    }

    if (validRoi[0].y > validRoi[1].y)
    {
      y_top_left_max = validRoi[0].y;
    }
    else
    {
      y_top_left_max = validRoi[1].y;
    }


    /* bottom right min point x and y*/
    if (validRoi[0].x + validRoi[0].width < validRoi[1].x + validRoi[1].width)
    {
      x_bottom_right_min = validRoi[0].x + validRoi[0].width;
    }
    else
    {
      x_bottom_right_min = validRoi[1].x + validRoi[1].width;
    }

    if (validRoi[0].y + validRoi[0].height < validRoi[1].y + validRoi[1].height)
    {
      y_bottom_right_min = validRoi[0].y + validRoi[0].height;
    }
    else
    {
      y_bottom_right_min = validRoi[1].y + validRoi[1].height;
    }

    Rect validRoi_common_left_right;
    validRoi_common_left_right.x = x_top_left_max;
    validRoi_common_left_right.y = y_top_left_max;
    validRoi_common_left_right.width = x_bottom_right_min - x_top_left_max;
    validRoi_common_left_right.height = y_bottom_right_min - y_top_left_max;

    Mat img_rect_color_cropped_rect[2];
    img_rect_color_cropped_rect[0] = img_rect_color_original[0](validRoi_common_left_right);
    img_rect_color_cropped_rect[1] = img_rect_color_original[1](validRoi_common_left_right);

    //    namedWindow("left_rect_test_cropped rect", 0);
    //    imshow("left_rect_test_cropped rect", img_rect_color_cropped_rect[0]);
    //    namedWindow("right_rect_test_cropped rect", 0);
    //    imshow("right_rect_test_cropped rect", img_rect_color_cropped_rect[1]);


    Size imageSize_cropped = img_rect_color_cropped_rect[0].size();
    Mat canvas_cropped;
    double sf_cropped;
    int w_cropped, h_cropped;
    // OpenCV can handle left-right or up-down camera arrangements
    sf_cropped = 600. / MAX(imageSize_cropped.width, imageSize_cropped.height);
    w_cropped = cvRound(imageSize_cropped.width * sf_cropped);
    h_cropped = cvRound(imageSize_cropped.height * sf_cropped);
    canvas_cropped.create(h_cropped, w_cropped * 2, CV_8UC3);

    for (int k = 0; k < 2; k++)
    {
      Mat cimg;
      img_rect_color_cropped_rect[k].copyTo(cimg);
      Mat canvasPart = canvas_cropped(Rect(w_cropped * k, 0, w_cropped, h_cropped)) ;
      resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
    }

    for (int j = 0; j < canvas_cropped.rows; j += 16)
    {
      line(canvas_cropped, Point(0, j), Point(canvas_cropped.cols, j), Scalar(0, 255, 0), 1, 8);
    }//for


    imshow("rectified_true_cropped", canvas_cropped);
    waitKey(1);

    img_rect[0] = img_rect[0](validRoi_common_left_right);
    img_rect[1] = img_rect[1](validRoi_common_left_right);

    //  namedWindow("left_rect_cropped", 0);
    //  imshow("left_rect_cropped", img_rect[0]);
    //  namedWindow("right_rect_cropped", 0);
    //  imshow("right_rect_cropped", img_rect[1]);

    img_rect[0].copyTo(left_img_rect);
    img_rect[1].copyTo(right_img_rect);
  } //stereo_rectify

  pcl::PointCloud<RGB>::Ptr
  img2pcd(const Mat image_rect)
  {

    Mat image;
    image_rect.copyTo(image);

    /// Convert the image from Gray to RGB
    cvtColor(image, image, CV_GRAY2BGR);

    // Retrieve the entries from the image data and copy them into the output RGB cloud
    //double* pixel = new double [4];
    //memset(pixel, 0, sizeof(double) * 4);
    //pixel[0] = (double) image.at<Vec3b>(y, x)[0];
    // free memory
    //delete[] pixel;

    double pixel;
    pcl::PointCloud<RGB> cloud;

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
    pcl::PointCloud<RGB>::Ptr cloud_output(new PointCloud<RGB>);
    *cloud_output = cloud;
    //copyPointCloud(cloud, valid_indices, *cloud_output); //sky removal but different sizes

    return cloud_output;

  }// img2pcd

  void
  processStereoPair(const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
  {
    stereo.compute(*left_image, *right_image);
    stereo.medianFilter(10);  // better but slower //optimal

    //stereo.getPointCloud(318.112200, 224.334900, 368.534700, 0.8387445, out_cloud, left_image); //original
    //stereo.getPointCloud(729.12211608886719, 481.94313430786133, 1266.4347948041523, 0.21, out_cloud, left_image);


    /*
            data: [ 1.3943661484434176e+03, 0.            , 6.4071969604492188e+02, 4.5742178807710679e+03,
                  0.            , 1.3943661484434176e+03, 4.8471289825439453e+02, 0.,
                  0., 0., 1., 0. ]
    */
    stereo.getPointCloud(640.71969604492188, 484.71289825439453, 1394.3661484434176, 0.16, out_cloud, left_image);


  }// processStereoPair

  void processCloud(const CloudConstPtr& cloud)
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
//  //      viewer->addText3D(ss.str(), position, textScale, r, g, b,  "cloud");
//      //viewer->addText(ss.str(), 400, 300, textScale, r, g, b, "cloud");
//      //viewer_test->addText3D(ss.str(), position, textScale, r, g, b, "cloud_test");
//
//      unsigned int x_min = 0, y_min = 0, x_max = cloud->width, y_max = cloud->height; //image.width=800, image.height=600
//      //bool  addLine (unsigned int x_min, unsigned int y_min, unsigned int x_max, unsigned int y_max, double r, double g, double b, const std::string &layer_id="line", double opacity=1.0)
//      double opacity = 1.0;
//      //const string layer_id;
//  //      image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
//      //image_viewer_test->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
//      x_min = cloud->width;
//      y_min = 0;
//      x_max = 0;
//      y_max = cloud->height;
//  //      image_viewer->addLine(x_min, y_min, x_max, y_max, r, g, b, "line", opacity);
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



    /*! other traversable regions in dominant plane limits - green*/
    /*! other traversable regions in dominant plane limits - green*/
    /*! other traversable regions in dominant plane limits - green*/
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
          double ptp_dist = 0.9; //pcl::pointToPlaneDistance(centroid_pt, ground_plane_params[0], ground_plane_params[1], ground_plane_params[2], ground_plane_params[3]);

          //if (plane_params[3] < dominant_plane_y_max && plane_params[3] > dominant_plane_y_min)
          if (clust_centroid[1] <= dominant_plane_y_max && clust_centroid[1] >= dominant_plane_y_min && ptp_dist < 1)
          {

            cout << "cluster_centroid meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;

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
          }//if
          else
          {
            cout << "cluster_centroid NOT meeting the condition: " << clust_centroid[1] << " and ptp_dist: " << ptp_dist << endl;
          }//else


        } //if region
      }// for i
    }// frame


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

  }// processCloud

  void
  run()
  {
    //while (!viewer->wasStopped())
    for (;;)
    {

      stereo_capture(); //images_idx
      images_idx ++;

      stereo_rectify();

      pcl::PointCloud<pcl::RGB>::Ptr left_cloud(new pcl::PointCloud<pcl::RGB>);
      pcl::PointCloud<pcl::RGB>::Ptr right_cloud(new pcl::PointCloud<pcl::RGB>);

      //png2pcd -- mat img input
      left_cloud = img2pcd(left_img_rect);
      right_cloud = img2pcd(right_img_rect);

      CloudPtr out_cloud(new Cloud);
      processStereoPair(left_cloud, right_cloud, out_cloud);

      pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
      stereo.getVisualMap(vmap);

      viewer->removeText3D("cloud");
      image_viewer->removeLayer("line");  //const std::string &layer_id
      processCloud(out_cloud);

      //pcl::visualization::ImageViewer image_viewer_disparity("disparity");  // (new pcl::visualization::ImageViewer("disparity"); // (new pcl::visulization::ImageViewer("disparity") )(new pcl::visulization::ImageViewer)
      image_viewer_disparity->addRGBImage<pcl::RGB> (vmap);

      if (!viewer->updatePointCloud(prev_ground_image, "cloud"))
      {
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(prev_ground_image);
        viewer->addPointCloud(prev_ground_image, rgb, "cloud"); //works too
        //viewer->addPointCloud(prev_ground_image, "cloud");
      }//if

      if (prev_cloud->points.size() > 1000)
      {
        image_viewer->addRGBImage<PointT>(prev_ground_image, "rgb_image", 0.3);
      }

//      if (prev_normal_cloud->points.size() > 1000 && display_normals)
//      {
//        viewer->removePointCloud("normals");
//        viewer->addPointCloudNormals<PointT, pcl::Normal>(prev_ground_image, prev_normal_cloud, 10, 0.15f, "normals");
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
//      }

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

      image_viewer_disparity->spinOnce(1);
      image_viewer->spinOnce(1);
      viewer->spinOnce(1);

      // key to exit or continue -- used with stereo_rectify
      cout << "press Esc to exit or other keys to continue................. " << endl;
      char key = waitKey(1);
      if (key == 27) //esc
        break;

    } // while - for
  }// run

};//class

int
main(int argc, char** argv)
{

  //saving the image??
  //cout << "---------------> all the left and right images will be saved in a folder called --> calib <-- folder in a jpeg format .........................." << endl;


  // Process and display
  StereoVisionProcessing stereo_vision_processing;
  stereo_vision_processing.run();

  return 0;
}// main
