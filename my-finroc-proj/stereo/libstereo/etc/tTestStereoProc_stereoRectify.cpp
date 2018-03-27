

#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::libstereo;


void
tTestStereoProcessing::stereo_rectify(const string input_images_left, const string input_images_right)
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
  //FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
  FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
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
    cout << "Error: can not save the intrinsic parameters\n";

  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];

  //fs.open("extrinsics.yml", CV_STORAGE_WRITE);
  fs.open(extrinsic_filename, CV_STORAGE_READ);
  if (fs.isOpened())
  {
    //fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
    //Mat R, T, R1, P1, R2, P2;
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

//    imshow("rectified_true", canvas);


//    img_rect[0].copyTo(left_img_rect);
//    img_rect[1].copyTo(right_img_rect);
//    namedWindow("left_rect", 0);
//    imshow("left_rect", left_img_rect);
//    namedWindow("right_rect", 0);
//    imshow("right_rect", right_img_rect);




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



  img_rect[0] = img_rect[0](validRoi_common_left_right);
  img_rect[1] = img_rect[1](validRoi_common_left_right);

  img_rect[0].copyTo(left_img_rect);
  img_rect[1].copyTo(right_img_rect);
  namedWindow("left_rect", 0);
  imshow("left_rect", left_img_rect);
  namedWindow("right_rect", 0);
  imshow("right_rect", right_img_rect);

}// stereo_rectify
