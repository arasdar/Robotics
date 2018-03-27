/*
 * tStereoProc_chessboardDetection.cpp
 *
 *  Created on: May 13, 2014
 *      Author: aras
 */



#include "projects/icarus/sensor_processing/stereo_sugv/calibration/chessboard/tStereoProcessing.h"

using namespace finroc::icarus::sensor_processing::stereo_sugv::calibration;



chessboardDataOutput tStereoProcessing::chessboardDetection(const IplImage* opencv_img_input)
{
  Mat view(opencv_img_input), view_chessboard, viewGray;
  imageSize = view.size();
  view.copyTo(viewGray); //gray scale camera
  //cvtColor(view, viewGray, CV_BGR2GRAY); //color camera

  found = findChessboardCorners(view, boardSize, pointbuf,
                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //stereo_calib

  // improve the found corners' coordinate accuracy
  if (pattern == CHESSBOARD && found)
    cornerSubPix(viewGray, pointbuf, Size(11, 11),
                 Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01)); //0.1 default

  if (mode == CAPTURING && found &&
      (clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC))  //!capture.isOpened() ||
  {
    imagePoints.push_back(pointbuf);
    prevTimestamp = clock();
  }


  view.copyTo(view_chessboard);
  cvtColor(view, view, CV_GRAY2BGR);
  if (found)
  {
    drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
  }// if

  //added for function output
  chessboardDataOutput data;
  data.found = found;
  view.copyTo(data.view);
  view_chessboard.copyTo(data.view_chessboard);

  return data;
}


void
tStereoProcessing::run()
{

  for (images_idx = 0;; images_idx++)
  {

    // capturing the left image or camera 0
    camera.RetrieveBuffer(&flycap_img);
    IplImage* opencv_img   = ConvertImageToOpenCV(&flycap_img);
    data_1 = chessboardDetection(opencv_img);
    namedWindow("left and camera 0", 1);
    imshow("left and camera 0", data_1.view);

    // capturing the right image or camera 1
    camera_2.RetrieveBuffer(&flycap_img_2);
    IplImage* opencv_img_2 = ConvertImageToOpenCV(&flycap_img_2);
    data_2 = chessboardDetection(opencv_img_2);
    namedWindow("right", 1);
    imshow("right", data_2.view);

    // key to exit or continue
    cout << "number of images captured: " << images_idx << endl;
    char key = waitKey(1);
    if (key == 27) //esc
      break;

    //saving  images
    if (data_1.found && data_2.found && key == ' ') //&& found_1
    {
      //writing left image
      sprintf(filename, "calib/left%.2d.jpg", n_l++);
      imwrite(filename, data_1.view_chessboard);
      cout << "Saved " << filename << endl;

      //writing right image
      sprintf(filename, "calib/right%.2d.jpg", n_r++);
      imwrite(filename, data_2.view_chessboard);
      cout << "Saved" << filename << endl;

    }//if

  } // while_old -for_new
}



