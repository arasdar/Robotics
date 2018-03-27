/*
* starter_video.cpp
*
*  Created on: Nov 23, 2010
*      Author: Ethan Rublee
*
* A starter sample for using opencv, get a video stream and display the images
* easy as CV_PI right?
*/
//#include "opencv2/highgui/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;


int process(VideoCapture& capture, string window_name)
{
  int n = 0;
  char filename[200];
  //string window_name = "video | q or esc to quit";
  cout << "press space to save a picture. q or esc to quit" << endl;
  namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
  Mat frame;
  for (;;)
  {
    capture >> frame;
    if (frame.empty())
      break;
    imshow(window_name, frame);
    char key = (char)waitKey(5); //delay N millis, usually long enough to display and capture input
    switch (key)
    {
    case 'q':
    case 'Q':
    case 27: //escape key
      return 0;
    case ' ': //Save an image
      sprintf(filename, "filename%.3d.jpg", n++);
      imwrite(filename, frame);
      cout << "Saved " << filename << endl;
      break;
    default:
      break;
    }
  }
  return 0;
}

#define IMAGE_WIDTH 480 //480*360
#define IMAGE_HEIGHT 360
#define CHESS_SIZE cvSize(7,4)

void simple_template()
{
  CvCapture *leftcam = 0;
  CvCapture *rightcam = 0;
  IplImage *leftim = 0;
  IplImage *rightim = 0;

  leftcam = cvCreateCameraCapture(0);
  rightcam = cvCreateCameraCapture(1);

  double w = IMAGE_WIDTH, h = IMAGE_HEIGHT;
  //camera setup(width and height)
  cvSetCaptureProperty(leftcam, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty(leftcam, CV_CAP_PROP_FRAME_HEIGHT, h);
  cvSetCaptureProperty(rightcam, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty(rightcam, CV_CAP_PROP_FRAME_HEIGHT, h);

  cvNamedWindow("3D Webcam left", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("3D Webcam right", CV_WINDOW_AUTOSIZE);

  for (;;)
  {
    leftim = cvQueryFrame(leftcam);
    rightim = cvQueryFrame(rightcam);

    cvShowImage("3D Webcam left", leftim);
    //  cvWaitKey(0);
    cvShowImage("3D Webcam right", rightim);
    cvWaitKey(100);
  } // FOR

  cvReleaseCapture(&leftcam);
  cvReleaseCapture(&rightcam);
//  cvDestroyWindow ("3D Webcam");
}

//new
int main_test()
{

  //initialize and allocate memory to load the video stream from camera
  CvCapture *capture1 = cvCaptureFromCAM(0);

  if (!capture1) return 1;

  //create a window with the title "Video1"
  cvNamedWindow("Video1");

  while (true)
  {
    //grab and retrieve each frames of the video sequentially
    IplImage* frame1 = cvQueryFrame(capture1);

    if (!frame1) break;

    //show the retrieved frame in the "Video1" window
    cvShowImage("Video1", frame1);

    //wait for 40 milliseconds
    int c = cvWaitKey(40);

    //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
    if ((char)c == 27) break;
  }

  //initialize and allocate memory to load the video stream from camera
  CvCapture *capture2 = cvCaptureFromCAM(1);

  if (!capture2) return 1;

  //create a window with the title "Video2"
  cvNamedWindow("Video2");

  while (true)
  {
    //grab and retrieve each frames of the video sequentially
    IplImage* frame2 = cvQueryFrame(capture2);

    if (!frame2) break;

    //show the retrieved frame in the "Video2" window
    cvShowImage("Video2", frame2);

    //wait for 40 milliseconds
    int c = cvWaitKey(40);

    //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
    if ((char)c == 27) break;
  }

  //destroy the opened window
  cvDestroyWindow("Video1");
  cvDestroyWindow("Video2");
  //release memory
  cvReleaseCapture(&capture1);
  cvReleaseCapture(&capture2);

  return 0;

  //VideoCapture1();
  //VideoCapture2();


}

int main(int ac, char** av)
{
  simple_template();

//  main_test();

//  ///////////left cam
//  std::string arg = av[1];
//  VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file
//  if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
//    capture.open(atoi(arg.c_str()));
//  string windowName = "capture_camLeft";
//  process(capture, windowName);

//    //////////right cam
//    string arg_r=av[2];
//    VideoCapture capture_r(arg_r);
//    if (!capture_r.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
//        capture_r.open(atoi(arg_r.c_str()));
//    string windowName_r="capture_camRight";
//    process(capture_r,windowName_r);

  return 0;
}
