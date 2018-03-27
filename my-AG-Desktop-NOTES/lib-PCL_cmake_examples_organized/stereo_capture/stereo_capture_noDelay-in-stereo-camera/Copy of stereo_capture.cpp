//////////system
#include <opencv2/highgui/highgui.hpp>
//////////local
//#include "projects/icarus/stereoVision/tSBM_Sample.h"
/////////debugging
#include <iostream>
#include <vector>
#include <stdio.h>
////////namespace
using namespace cv;
using namespace std;


int main(int ac, char** av)
{

  ///////////camera 0
//    string arg_0 = av[1];
  string arg_0 = "0";
  VideoCapture capture_0(arg_0); //try to open string, this will attempt to open it as a video file
  if (!capture_0.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
    capture_0.open(atoi(arg_0.c_str()));
  Mat frame_0;



  //////////camera 1
//        string arg_1=av[2];
  string arg_1 = "1";
  VideoCapture capture_1(arg_1);
  if (!capture_1.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
    capture_1.open(atoi(arg_1.c_str()));
  Mat frame_1;

  int n_l = 1;
  int n_r = 1;

  char filename[200];
  cout << "press space to save a picture. q or esc to quit" << endl;

  for (;;)
  {
    ////////camera 0
    capture_0 >> frame_0;
    if (frame_0.empty())
      break;
    namedWindow("capture_0", 0);
    imshow("capture_0", frame_0);

    ///////camera 1
    capture_1 >> frame_1;
    if (frame_1.empty())
      break;
    namedWindow("capture_1", 0);
    imshow("capture_1", frame_1);

//    ////////main_SBM_Sample
//    SBM_Sample(frame_0, frame_1);


    ////////exit
    char key = (char) waitKey(5); //delay N millis, usually long enough to display and capture input
    switch (key)
    {
    case 'q':
    case 'Q':
    case 27: //escape key
      return 0;
    case ' ': //Save an image
      sprintf(filename, "left%.2d.jpg", n_l++);
      imwrite(filename, frame_0);
      cout << "Saved " << filename << endl;
      sprintf(filename, "right%.2d.jpg", n_r++);
      imwrite(filename, frame_1);
      cout << "Saved " << filename << endl;
      break;
    default:
      break;
    }//switch
  }//for
//        return 0;



  return 0;
}//main
