

#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::segmentation_with_texture;

void
tStereoProcessing::stereo_rectify(const string input_images_left, const string input_images_right)
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


  Mat cameraMatrix[2], distCoeffs[2];
  cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
  cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
  Mat R, T, E, F;



  // save intrinsic parameters
  /*//FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);*/
  FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
  if (fs.isOpened())
  {
    /*
        //fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
        //"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        //Mat M1, D1, M2, D2;
    */
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

  /*//fs.open("extrinsics.yml", CV_STORAGE_WRITE);*/
  fs.open(extrinsic_filename, CV_STORAGE_READ);
  if (fs.isOpened())
  {
    /*
        //fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        //Mat R, T, R1, P1, R2, P2;
    */
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


  /*//StereoCalib(const vector<string>& imagelist, Size boardSize, bool useCalibrated=true, bool showRectified=true);*/
  bool useCalibrated = true;
  bool showRectified = true;

  // COMPUTE AND DISPLAY RECTIFICATION
  if (!showRectified)
    return;

  Mat rmap[2][2];

  //Precompute maps for cv::remap()
  initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
  initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

  Mat canvas;
  double sf = 0;
  int w = 0, h = 0;
  // OpenCV can handle left-right or up-down camera arrangements
  bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
  if (!isVerticalStereo)
  {
    sf = 600. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width * sf);
    h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);
  }

  Mat img_rect[2];
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

  imshow("rectified_true", canvas);

  img_rect[0].copyTo(left_img_rect);
  img_rect[1].copyTo(right_img_rect);
  namedWindow("left_rect", 0);
  imshow("left_rect", left_img_rect);
  namedWindow("right_rect", 0);
  imshow("right_rect", right_img_rect);

  stereo_cropping(img_rect, validRoi);

  P2_calib = P2;
  //FINROC_LOG_PRINT(DEBUG_VERBOSE_1, "P2_calib: ", P2_calib);

  /*
  P2: [1511.195261451447, 0          , 652.3221817016602, -4931.156669552001;
        0       , 1511.195261451447, 492.8693962097168, 0;
      0       ,         0,         1, 0]
  */

  /*
          float u_c = 652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
          float v_c = 492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
          float focal = 1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
          float baseline = 4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator
  */

  /*
  //  double X_c = P2_calib.at<double>(0, 2); //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  //  double Y_c = P2_calib.at<double>(1, 2); //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
  //  double focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  //  double baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator

    // this is bad not working at all
  //  float X_c = P2_calib.at<float>(0, 2); //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
  //  float Y_c = P2_calib.at<float>(1, 2); //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
  //  float focal = P2_calib.at<float>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
  //  float baseline = abs(P2_calib.at<float>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator
  */

  /*
    float X_c = P2_calib.at<double>(0, 2); //652.32218170166016f; //403.77966308593750f; //379.85181427001953f; // 4.0377966308593750e+02 //calib_1_ok
    float Y_c = P2_calib.at<double>(1, 2); //492.86939620971680f; //305.85922241210938f; //3.5859558486938477e+02
    float focal = P2_calib.at<double>(0, 0); //1511.1952614514469f; //840.67043744070190f; //920.38355542932538f; //8.4067043744070190e+02
    float baseline = abs(P2_calib.at<double>(0, 3)) * 0.05 / focal;//4931.1566695520014f * 0.05 / focal; //0.46; //meter for unit //0.359294689f; //real one using calculator

    FINROC_LOG_PRINT(DEBUG, "X_c: ", X_c, " Y_c:", Y_c, " focal: ", focal, " baseline: ", baseline, " point_crop: ", pt_top_left_crop);

  //  FINROC_LOG_PRINT(DEBUG, "P2_calib.at<double>(0, 0): ", P2_calib.at<double>(0, 0));
  //  FINROC_LOG_PRINT(DEBUG, "P2_calib.at<double>(0, 1): ", P2_calib.at<double>(0, 1));
  //  FINROC_LOG_PRINT(DEBUG, "P2_calib.at<double>(0, 2): ", P2_calib.at<double>(0, 2));
  //
  //  FINROC_LOG_PRINT(DEBUG, "P2_calib.at<double>(1, 1): ", P2_calib.at<double>(1, 1));
  //  FINROC_LOG_PRINT(DEBUG, "P2_calib.at<double>(1, 2): ", P2_calib.at<double>(1, 2));
  //  FINROC_LOG_PRINT(DEBUG, "P2_calib.at<double>(1, 3): ", P2_calib.at<double>(1, 3));
  */


}// stereo_rectify

