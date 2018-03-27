

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
        "  'space' - start capturing images\n";

int main( int argc, char** argv )
{

    VideoCapture capture;
    VideoCapture capture_1;

    ///////// camera index 0
	capture.open(0);  //right
	capture_1.open(1);
	/////save image
	int n_r = 1, n_l=1;
	char filename[200];
    ////////////
    if( capture.isOpened() && capture_1.isOpened() )
        printf( "%s", liveCaptureHelp );
    ////
    capture.set( CV_CAP_PROP_FRAME_WIDTH, 800 );
    capture.set( CV_CAP_PROP_FRAME_HEIGHT, 600 );
    capture.set( CV_CAP_PROP_FPS, 30 );
    /////////// Print some avalible device settings.
    cout << "\nDepth generator output mode for camera 0 (rightttt):" << endl <<
            "FRAME_WIDTH      " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT     " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FPS              " << capture.get( CV_CAP_PROP_FPS ) << endl;

    /////////camera index 1
    capture_1.set( CV_CAP_PROP_FRAME_WIDTH, 800 );
    capture_1.set( CV_CAP_PROP_FRAME_HEIGHT, 600 );
    capture_1.set( CV_CAP_PROP_FPS, 30 );
    /////////// Print some avalible device settings.
    cout << "\nDepth generator output mode fro camera 1 (leftttttt):" << endl <<
            "FRAME_WIDTH      " << capture_1.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT     " << capture_1.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FPS              " << capture_1.get( CV_CAP_PROP_FPS ) << endl;



    for(;;)
    {

    	////////////RIGHT CAMERA_CAMERAINDEX=0
        Mat view, viewGray;
        /////////
        Mat view0;
        if( !capture.grab() ){
             cout << "Can not grab images." << endl;
             return -1;
         }
        capture.retrieve(view0);
        view0.copyTo(view);
        /////////save the image with the found chessboard  //aras
        namedWindow( "ImageView_right_0", 1 );
        imshow("ImageView_right_0", view);
        ///////
        Mat frame_found;
		view0.copyTo(frame_found);

		//////////left camera camera index=1
        if( !capture_1.grab() ){
                 cout << "Can not grab images." << endl;
                 return -1;
		 }
		capture_1.retrieve(view0);
		view0.copyTo(view);

        /////////save the image with the found chessboard  //aras
        namedWindow( "ImageView_left_1", 1 );
        imshow("ImageView_left_1", view);
        ///////
		Mat frame_found_1;
		view0.copyTo(frame_found_1);


        ////////exit
        char key = (char) waitKey(5); //delay N millis, usually long enough to display and capture input
        switch (key)
        {
			case 'q':
			case 'Q':
			case 27: //escape key
			  return 0;
			case ' ': //Save an image
			  sprintf(filename, "left/left%.2d.png", n_l++);
			  imwrite(filename, frame_found);
			  cout << "Saved " << filename << endl;
			  sprintf(filename, "right/right%.2d.png", n_r++);
			  imwrite(filename, frame_found_1);
			  cout << "Saved " << filename << endl;
			  break;
			default:
			  break;
		}//switch

    }//for

    return 0;

}//main
