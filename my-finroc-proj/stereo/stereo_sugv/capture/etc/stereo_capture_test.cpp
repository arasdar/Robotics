/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain for traversability and drivability using stereo camera.
  *
  * \author Aras Dargazany
  */

#include <pcl/io/io.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "projects/icarus/sensor_processing/libstereo/tests/stereo_matching.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

#include <FlyCapture2.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace FlyCapture2;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class StereoVisionProcessing
{
private:
  boost::shared_ptr<visualization::ImageViewer> image_viewer;
  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  std::vector<std::string> left_images;
  std::vector<std::string> right_images;
  int images_idx;
  int img_pairs_num;
  bool trigger;
  bool continuous;

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

  /*stereo capture, rectification initialization*/
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/intrinsics.yml", //aras@pasithee:~/stereo_images/stereo_sugv/2_on-sugv/calib_8_good
         input_extrinsic_filename = "/home/aras/finroc/sources/cpp/projects/icarus/sensor_processing/stereo_sugv/capture/extrinsics.yml";
  Mat input_images_left, input_images_right;
  Mat left_img_rect;
  Mat right_img_rect;

  finroc::icarus::sensor_processing::ACSO stereo;
  int smooth_weak;
  int smooth_strong;

public:
  StereoVisionProcessing(/*const std::vector<std::string> left_images, const std::vector<std::string> right_images,
                         const int img_pairs_num, const string input_intrinsic_filename, const string input_extrinsic_filename*/) :
    image_viewer(new visualization::ImageViewer("Image Viewer")),
    viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
  {
    trigger = false;
    continuous = false;

//    this->left_images = left_images;
//    this->right_images = right_images;
//    images_idx = 0;
//    this->img_pairs_num = img_pairs_num;
    this->input_intrinsic_filename = input_intrinsic_filename;
    this->input_extrinsic_filename = input_extrinsic_filename;


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

    /*! Set up a 3D viewer*/
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);  //(0, 0, -2, 0, -1, 0, 0)
//    viewer->registerKeyboardCallback(&StereoVisionProcessing::keyboardCallback, *this, 0);


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

    /**
      * * preprocessing of the image pair, to improve robustness against photometric distortions
      *   (wrt. to a spatially constant additive photometric factor)
      */
    stereo.setPreProcessing(true);

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

//  void
//  keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
//  {
//    if (event.keyUp())
//    {
//      switch (event.getKeyCode())
//      {
//      case ' ':
//        trigger = true;
//        break;
//      case 'c':
//        continuous = !continuous;
//        break;
//      case '1':
//        smooth_strong -= 10;
//        PCL_INFO("smooth_strong: %d\n", smooth_strong);
//        stereo.setSmoothStrong(smooth_strong);
//        break;
//      case '2':
//        smooth_strong += 10;
//        PCL_INFO("smooth_strong: %d\n", smooth_strong);
//        stereo.setSmoothStrong(smooth_strong);
//        break;
//      case '3':
//        smooth_weak -= 10;
//        PCL_INFO("smooth_weak: %d\n", smooth_weak);
//        stereo.setSmoothWeak(smooth_weak);
//        break;
//      case '4':
//        smooth_weak += 10;
//        PCL_INFO("smooth_weak: %d\n", smooth_weak);
//        stereo.setSmoothWeak(smooth_weak);
//        break;
//      }
//    }
//  }


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



  void
  processStereoPair(const pcl::PointCloud<PointT>::Ptr& out_cloud)
  {

    /*stereo images camera calibration parameters for ptgrey grayscale camera*/
    float u_c = 403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
    float v_c = 358.59558486938477f; //305.85922241210938f; //3.5859558486938477e+02
    float focal = 840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
    float baseline = 0.46; //meter for unit //0.359294689f; //real one using calculator
    stereo.getPointCloud(u_c, v_c, focal, baseline, out_cloud, out_cloud);

  }// processPair

  void
  run()
  {
    while (!viewer->wasStopped())
    {

      stereo_capture(); //images_idx
      images_idx ++;

      stereo_rectify();

      /*! displaying disparity map*/
      stereo.compute(left_img_rect.data, right_img_rect.data, left_img_rect.cols, left_img_rect.rows);
      stereo.medianFilter(10);  // better but slower //optimal

      /*! visualizing in opencv*/
      Mat vmap_opencv(left_img_rect.rows, left_img_rect.cols, CV_8UC3);
      stereo.getVisualMap(vmap_opencv);
      imshow("vmap_opencv", vmap_opencv);
      waitKey(1);

//      /*! visualizing in pcl*/
//      pcl::PointCloud<pcl::RGB>::Ptr vmap(new pcl::PointCloud<pcl::RGB>);
//      stereo.getVisualMap(vmap);
//      image_viewer->addRGBImage<RGB> (vmap);

//      /*! generating the point cloud*/
//      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_disp(new pcl::PointCloud<pcl::PointXYZ>);
//      processStereoPair(out_cloud_disp);
//    if (!viewer->updatePointCloud(out_cloud_disp, "cloud disparity"))
//      viewer->addPointCloud(out_cloud_disp, "cloud disparity");


//      cout << "left_images[img_index]: " << left_images[images_idx] << std::endl;
//      cout << "right_images[img_index]: " << right_images[images_idx] << std::endl;
//      cout << "images_idx: " << images_idx << endl;
//      cout << "img_pairs_num: " << img_pairs_num << endl;
//      cout << "press q or Q on the main viewer to exit or space to continue................. " << endl;

      viewer->spinOnce(1);
      image_viewer->spinOnce(1);
    } // while

  }// run



};//class

int
main(int argc, char** argv)
{

//  if (argc < 3)
//  {
//    PCL_INFO("usage: aras_icarus_sensorProcessing_libstereo_gray left_image_directory right_image_directory intrinsic_parameter_filename extrinsic_parameter_filename\n");
//    PCL_INFO("note: images in both left and right folders can be in different format.\n");
//    PCL_INFO("for example : \n"
//             "aras_icarus_test ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/left/ ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/right/ ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/intrinsics.yml ~/stereo_images/stereo_ravon/stereo_45cm-baseline/2013-10-25_myStereo-baseline-45cm_cart_outside_forest/calib_1-ok/extrinsics.yml\n "
//             "aras_icarus_test ~/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/left ~/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/right/ ~/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/intrinsics.yml ~/stereo_images/stereo_sugv/2014-02-20_outside-first-dataset/calib_1_good_used/extrinsics.yml\n "
//            );
//    return -1;
//  }
//
//  /*variable initial*/
//  int img_number_left = 0, img_number_right = 0 ;
//  int img_pairs_num = 0;
//
//  /*Get list of stereo files from left folder*/
//  std::vector<std::string> left_images;
//  boost::filesystem::directory_iterator end_itr;
//  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
//  {
//    left_images.push_back(itr->path().string());
//    img_number_left++;
//  }
//  sort(left_images.begin(), left_images.end());
//
//  /*reading right images from folder*/
//  std::vector<std::string> right_images;
//  for (boost::filesystem::directory_iterator itr(argv[2]); itr != end_itr; ++itr)
//  {
//    right_images.push_back(itr->path().string());
//    img_number_right++;
//  }
//  sort(right_images.begin(), right_images.end());
//  PCL_INFO("Press space to advance to the next frame, or 'c' to enable continuous mode\n");
//
//  /*showing the input images*/
//  cout << "img_number_left: " << img_number_left << std::endl;
//  cout << "img_number_right: " << img_number_right << std::endl;
//  if (img_number_left == img_number_right)
//    img_pairs_num = img_number_left;
//
//  /*calibration parameters*/
//  string input_intrinsic_filename = argv[3];
//  string input_extrinsic_filename = argv[4];

  /*Process and display*/
  StereoVisionProcessing stereo_vision_processing; //(/*left_images, right_images, img_pairs_num, input_intrinsic_filename, input_extrinsic_filename*/);
  stereo_vision_processing.run();

  return 0;
}// main
