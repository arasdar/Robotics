#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"

#include "libcam.h"

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

const char* liveCaptureHelp =
    "When the live video from camera is used as input, the following hot-keys may be used:\n"
        "  <ESC>, 'q' - quit the program\n"
        "  'g' - start capturing images\n"
        "  'u' - switch undistortion on/off\n";


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
  cvNamedWindow("left", 0);
  cvNamedWindow("right", 0);
  int k = 0;

  //chessboard
  Size boardSize, imageSize;
  boardSize.width=10;
  boardSize.height=7;
  Mat cameraMatrix, distCoeffs;
  ///////
  int i, nframes = 20;
  bool undistortImage = true;
  VideoCapture capture;
  VideoCapture capture_1;
  //////////
  int delay = 1;
  clock_t prevTimestamp = 0;
  int mode = DETECTION;
  vector<vector<Point2f> > imagePoints;
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

    cvShowImage("left", l);
    cvShowImage("right", r);

	////////////RIGHT CAMERA_CAMERAINDEX=0
    Mat view(r), viewGray;
    bool blink = false;
    /////////
     imageSize = view.size();
     ///////////
     vector<Point2f> pointbuf;
//        cvtColor(view, viewGray, CV_BGR2GRAY);
//        view.copyTo(viewGray); //gray scale camera
     cvtColor(view, viewGray, CV_BGR2GRAY); //color camera
     /////////
     bool found=false;
		found = findChessboardCorners( view, boardSize, pointbuf,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    // improve the found corners' coordinate accuracy
     if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(11,11),
         Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
     ////////
     if( mode == CAPTURING && found &&
        (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) ){
         imagePoints.push_back(pointbuf);
         prevTimestamp = clock();
         blink = capture.isOpened();
     }
     //////////
     if(found)
         drawChessboardCorners( view, boardSize, Mat(pointbuf), found );

     string msg = mode == CAPTURING ? "100/100" :
         mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
     int baseLine = 0;
     Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
     Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

     if( mode == CAPTURING )
     {
         if(undistortImage)
             msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
         else
             msg = format( "%d/%d", (int)imagePoints.size(), nframes );
     }

     putText( view, msg, textOrigin, 1, 1,
              mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

     if( blink )
         bitwise_not(view, view);


     if( mode == CALIBRATED && undistortImage )
     {
         Mat temp = view.clone();
         undistort(temp, view, cameraMatrix, distCoeffs);
     }
     /////////save the image with the found chessboard  //aras
     namedWindow( "ImageView_right_0", 1 );
     imshow("ImageView_right_0", view);
     ///////
     Mat frame_found;
     if (found)
     	view.copyTo(frame_found);



     //////////left camera camera index=1
     Mat viewl(l);
//     bool blink = false;
	imageSize = viewl.size();
//		///////////
//		view.copyTo(viewGray);
	cvtColor (viewl, viewGray, CV_BGR2GRAY);
	/////////
	bool found_1=false;
	found_1 = findChessboardCorners( viewl, boardSize, pointbuf,
	CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
 		// improve the found corners' coordinate accuracy
 		if( pattern == CHESSBOARD && found_1) cornerSubPix( viewGray, pointbuf, Size(11,11),
 		   Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
 		////////
 		if( mode == CAPTURING && found_1 &&
 		  (!capture_1.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) ){
 		   imagePoints.push_back(pointbuf);
 		   prevTimestamp = clock();
 		   blink = capture_1.isOpened();
 		}//if
         //////////
         if(found_1)
             drawChessboardCorners( viewl, boardSize, Mat(pointbuf), found_1 );
         if( mode == CAPTURING )
         {
             if(undistortImage)
                 msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
             else
                 msg = format( "%d/%d", (int)imagePoints.size(), nframes );
         }

         putText( viewl, msg, textOrigin, 1, 1,
                  mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

         if( blink )
             bitwise_not(viewl, viewl);


         if( mode == CALIBRATED && undistortImage )
         {
             Mat temp = viewl.clone();
             undistort(temp, viewl, cameraMatrix, distCoeffs);
         }
         /////////save the image with the found chessboard  //aras
         namedWindow( "ImageView_left_1", 1 );
         imshow("ImageView_left_1", viewl);
         ///////
 		Mat frame_found_1;
 		if (found_1)
 			viewl.copyTo(frame_found_1);



	///////saving  images
	 if (found && found_1){
		sprintf(filename, "stereo_calib/right%.2d.jpg", n_r++);
		imwrite(filename, frame_found);
		cout << "Saved " << filename << endl;
		////
		sprintf(filename, "stereo_calib/left%.2d.jpg", n_l++);
		imwrite(filename, frame_found_1);
		cout << "Saved " << filename << endl;
	 }//if

// 	////////////left CAMERA_CAMERAINDEX=0
//     Mat view(l), viewGray;
//     bool blink = false;
//     /////////
//      imageSize = view.size();
//      ///////////
//      vector<Point2f> pointbuf;
// //        cvtColor(view, viewGray, CV_BGR2GRAY);
// //        view.copyTo(viewGray); //gray scale camera
//      cvtColor(view, viewGray, CV_BGR2GRAY); //color camera
//      /////////
//      bool found=false;
// 		found = findChessboardCorners( view, boardSize, pointbuf,
// 			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
//     // improve the found corners' coordinate accuracy
//      if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(11,11),
//          Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
//      ////////
//      if( mode == CAPTURING && found &&
//         (!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) ){
//          imagePoints.push_back(pointbuf);
//          prevTimestamp = clock();
//          blink = capture.isOpened();
//      }
//      //////////
//      if(found)
//          drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
//
//      string msg = mode == CAPTURING ? "100/100" :
//          mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
////      int baseLine = 0;
//      Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
//      Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);
//
//      if( mode == CAPTURING )
//      {
//          if(undistortImage)
//              msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
//          else
//              msg = format( "%d/%d", (int)imagePoints.size(), nframes );
//      }
//
//      putText( view, msg, textOrigin, 1, 1,
//               mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));
//
//      if( blink )
//          bitwise_not(view, view);
//
//
//      if( mode == CALIBRATED && undistortImage )
//      {
//          Mat temp = view.clone();
//          undistort(temp, view, cameraMatrix, distCoeffs);
//      }
//      /////////save the image with the found chessboard  //aras
//      namedWindow( "ImageView_left_1", 1 );
//      imshow("ImageView_left_1", view);

    key = cvWaitKey(1);
  }// while

  cvDestroyAllWindows();

  return 0;
}// main
