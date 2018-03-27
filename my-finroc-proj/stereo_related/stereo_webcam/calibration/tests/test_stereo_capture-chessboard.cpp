#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/opencv.hpp>

//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/opencv.hpp"

//#include "libcam.h"
#include "projects/icarus/sensorProcessing/stereo_webcam/calibration/libcam.h"

#include <stdio.h>
#include <stdlib.h>

//
//////global system installation
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

/////////debugging
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <vector>

///////namesapce
using namespace cv;
using namespace std;

//const char* liveCaptureHelp =
//  "When the live video from camera is used as input, the following hot-keys may be used:\n"
//  "  <ESC>, 'q' - quit the program\n"
//  "  'g' - start capturing images\n"
//  "  'u' - switch undistortion on/off\n";


enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };


int main()
{
  //capture
  int w = 640, h = 480;
  Camera cl("/dev/video1", w, h, 15); //left camera object using libv4lcam2
  Camera cr("/dev/video0", w, h, 15); //right camera object
  IplImage *l = cvCreateImage(cvSize(w, h), 8, 3); //left image
  IplImage *r = cvCreateImage(cvSize(w, h), 8, 3); //right image
  char key = 'a';
  int k = 0;

  //chessboard
  Size boardSize, imageSize;
  boardSize.width = 9;
  boardSize.height = 6;
  Mat cameraMatrix, distCoeffs;
  ///////
//  int i, nframes = 20;
//  bool undistortImage = true;
  VideoCapture capture;
  VideoCapture capture_1;
  //////////
  int delay = 1;
  clock_t prevTimestamp = 0;
  int mode = DETECTION;
  vector<vector<Point2f>> imagePoints;
  Pattern pattern = CHESSBOARD;

  //saving frames
  int n_r = 1;
  char filename[200];
  int n_l = 1;


  while (key != 27)
  {
    cl.Update(&cr);
    cl.toIplImage(l);
    cr.toIplImage(r);

//    cvNamedWindow("left", 0);
//    cvNamedWindow("right", 0);
//    cvShowImage("left", l);
//    cvShowImage("right", r);

    ////////////RIGHT CAMERA_CAMERAINDEX=0
    Mat view(r), view_chessboard, viewGray;
    bool blink = false;
    /////////
    imageSize = view.size();
    ///////////
    vector<Point2f> pointbuf;
//        cvtColor(view, viewGray, CV_BGR2GRAY);
//        view.copyTo(viewGray); //gray scale camera
    cvtColor(view, viewGray, CV_BGR2GRAY); //color camera
    /////////
    bool found = false;
    //original
//    found = findChessboardCorners(view, boardSize, pointbuf,
//                                  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    found = findChessboardCorners(view, boardSize, pointbuf,
                                  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //stereo_calib
    // improve the found corners' coordinate accuracy
    if (pattern == CHESSBOARD && found)
      cornerSubPix(viewGray, pointbuf, Size(11, 11),
                   Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01)); //0.1 default
//      cornerSubPix(img, corners, Size(11,11),
//      Size(-1,-1),  TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01)); //stereo_calib.cpp
    ////////
    if (mode == CAPTURING && found &&
        (!capture.isOpened() || clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC))
    {
      imagePoints.push_back(pointbuf);
      prevTimestamp = clock();
      blink = capture.isOpened();
    }
    //////////
    view.copyTo(view_chessboard);
    if (found)
      drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
    /////////save the image with the found chessboard  //aras
//    namedWindow("ImageView_right_0", 1);
//    imshow("ImageView_right_0", view);
    ///////
    Mat frame_found;
    if (found)
      view_chessboard.copyTo(frame_found);


    //////////left camera camera index=1
    Mat viewl(l), viewl_chessboard;
//     bool blink = false;
    imageSize = viewl.size();
//    ///////////
//    view.copyTo(viewGray);
    cvtColor(viewl, viewGray, CV_BGR2GRAY);
    /////////
    bool found_1 = false;
    found_1 = findChessboardCorners(viewl, boardSize, pointbuf,
                                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK);
    // improve the found corners' coordinate accuracy
    if (pattern == CHESSBOARD && found_1) cornerSubPix(viewGray, pointbuf, Size(11, 11),
          Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
    ////////
    if (mode == CAPTURING && found_1 &&
        (!capture_1.isOpened() || clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC))
    {
      imagePoints.push_back(pointbuf);
      prevTimestamp = clock();
      blink = capture_1.isOpened();
    }//if
    //////////
    viewl.copyTo(viewl_chessboard);
    if (found_1)
      drawChessboardCorners(viewl, boardSize, Mat(pointbuf), found_1);
    /////////save the image with the found chessboard  //aras
//    namedWindow("ImageView_left_1", 1);
//    imshow("ImageView_left_1", viewl);
    ///////
    Mat frame_found_1;
    if (found_1)
      viewl_chessboard.copyTo(frame_found_1);


    //// order of displaying images
    namedWindow("ImageView_left_1", 1);
    imshow("ImageView_left_1", viewl);
    namedWindow("ImageView_right_0", 1);
    imshow("ImageView_right_0", view);

    key = cvWaitKey(1);

    ///////saving  images
    if (found && found_1 && key == ' ')
    {
      sprintf(filename, "calib/right%.2d.jpg", n_r++);
      imwrite(filename, frame_found);
      cout << "Saved " << filename << endl;
      ////
      sprintf(filename, "calib/left%.2d.jpg", n_l++);
      imwrite(filename, frame_found_1);
      cout << "Saved " << filename << endl;
    }//if

//    key = cvWaitKey(1);
  }// while

  cvDestroyAllWindows();

  return 0;
}// main
