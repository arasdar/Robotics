

#include "projects/stereo_traversability_experiments/stereo_processing/mStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::stereo_processing;

void
mStereoProcessing::stereo_rectify()
{

  Mat img1(input_images_left), img2(input_images_right);
//  namedWindow("left", 0);
//  imshow("left", img1);
//  namedWindow("right", 0);
//  imshow("right", img2);

  Mat img[2];
  img1.copyTo(img[0]);
  img2.copyTo(img[1]);

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

  /// Convert the image from Gray to RGB
  //cvtColor(image, image, CV_GRAY2BGR);
  cvtColor(img1, img[0], CV_BGR2GRAY);
  cvtColor(img2, img[1], CV_BGR2GRAY);
  img[0].copyTo(left_img_rect);
  img[1].copyTo(right_img_rect);

//  namedWindow("left_rect", 0);
//  imshow("left_rect", left_img_rect);
//  namedWindow("right_rect", 0);
//  imshow("right_rect", right_img_rect);

}// stereo_rectify_test

