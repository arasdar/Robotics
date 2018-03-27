/*
 * mStereoProc_cropping.cpp
 *
 *  Created on: May 21, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/stereo_color/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::stereo_color::offline;

void tStereoProcessing::stereo_cropping(Mat* img_rect, const Rect* validRoi)
{

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

  /*
  //    namedWindow("left_rect_test", 0);
  //    imshow("left_rect_test", img_rect_color[0]);
  //    namedWindow("right_rect_test", 0);
  //    imshow("right_rect_test", img_rect_color[1]);
  */

  Mat img_rect_color_cropped[2];
  img_rect_color_cropped[0] = img_rect_color[0](validRoi[0]);
  img_rect_color_cropped[1] = img_rect_color[1](validRoi[1]);

  /*
  //    namedWindow("left_rect_test_cropped", 0);
  //    imshow("left_rect_test_cropped", img_rect_color_cropped[0]);
  //    namedWindow("right_rect_test_cropped", 0);
  //    imshow("right_rect_test_cropped", img_rect_color_cropped[1]);
  */


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

  /*
  //    namedWindow("left_rect_test_cropped rect", 0);
  //    imshow("left_rect_test_cropped rect", img_rect_color_cropped_rect[0]);
  //    namedWindow("right_rect_test_cropped rect", 0);
  //    imshow("right_rect_test_cropped rect", img_rect_color_cropped_rect[1]);
  */


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

  pt_top_left_crop = Point(x_top_left_max, y_top_left_max);
}






