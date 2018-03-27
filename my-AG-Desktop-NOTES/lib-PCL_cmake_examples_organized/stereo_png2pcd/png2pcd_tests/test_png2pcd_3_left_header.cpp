//////////system-global
#include <opencv2/highgui/highgui.hpp>

//////////local
#include "projects/stereo/png2pcd_3_left.h"

/////////debugging
#include <iostream>
#include <vector>
#include <stdio.h>

////////namespace
using namespace cv;
using namespace std;


int main(int ac, char** av)
{

	Mat frame;
	imread(frame,"frame.png");
    namedWindow("left.png", 0);
    imshow("left.png", frame);

    ////////exit
    char key = (char) waitKey(5); //delay N millis, usually long enough to display and capture input
    switch (key){
		case 'q':
		case 'Q':
		case 27: //escape key
			return 0;
		case 'l':
			main_png2pcd_left ();
		default:
			break;
	}//switch


  return 0;
}//main
