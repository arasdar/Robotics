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



int main( int argc, char** argv )
{
    Size boardSize, imageSize;
    boardSize.width=9;
    boardSize.height=6;
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




    ///////// camera index 0
	capture.open(0);  //right
	/////save image
	int n_r = 1;
	char filename[200];
    ////////////
    if( capture.isOpened() )
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
	capture_1.open(1); //left
	int n_l = 1;
    if( capture_1.isOpened() )
        printf( "%s", liveCaptureHelp );
    //////////
    capture_1.set( CV_CAP_PROP_FRAME_WIDTH, 800 );
    capture_1.set( CV_CAP_PROP_FRAME_HEIGHT, 600 );
    capture_1.set( CV_CAP_PROP_FPS, 30 );
    /////////// Print some avalible device settings.
    cout << "\nDepth generator output mode fro camera 1 (leftttttt):" << endl <<
            "FRAME_WIDTH      " << capture_1.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT     " << capture_1.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FPS              " << capture_1.get( CV_CAP_PROP_FPS ) << endl;



    for(i = 0;;i++)
    {



    	////////////RIGHT CAMERA_CAMERAINDEX=0
        Mat view, viewGray;
        bool blink = false;
        /////////
        Mat view0;
        if( !capture.grab() ){
             cout << "Can not grab images." << endl;
             return -1;
         }
        capture.retrieve(view0);
        view0.copyTo(view);
        /////////
        imageSize = view.size();
        ///////////
        vector<Point2f> pointbuf;
//        cvtColor(view, viewGray, CV_BGR2GRAY);
        view.copyTo(viewGray);
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
        	view0.copyTo(frame_found);




//        //////////left camera camera index=1
        if( !capture_1.grab() ){
                 cout << "Can not grab images." << endl;
                 return -1;
             }
            capture_1.retrieve(view0);
            view0.copyTo(view);
//		/////////
		imageSize = view.size();
		///////////
		view.copyTo(viewGray);
		/////////
		bool found_1=false;
		found_1 = findChessboardCorners( view, boardSize, pointbuf,
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
            drawChessboardCorners( view, boardSize, Mat(pointbuf), found_1 );
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
        namedWindow( "ImageView_left_1", 1 );
        imshow("ImageView_left_1", view);
        ///////
		Mat frame_found_1;
		if (found_1)
			view0.copyTo(frame_found_1);


		///////saving  images
        if (found && found_1){
        	sprintf(filename, "stereo_calib_data/right%.2d.jpg", n_r++);
			imwrite(filename, frame_found);
			cout << "Saved " << filename << endl;
			////
        	sprintf(filename, "stereo_calib_data/left%.2d.jpg", n_l++);
			imwrite(filename, frame_found_1);
			cout << "Saved " << filename << endl;
        }//if


		////////wait
        int key = 0xff & waitKey(1);
		///////exit
        if( (key & 255) == 27 )
            break;


        if(capture_1.isOpened() && capture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }


    }//for


    return 0;
}
