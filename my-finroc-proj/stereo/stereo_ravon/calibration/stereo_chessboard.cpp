/** \brief StereoVisioProcessing is an application for processing stereo images to estimate terrain traversability using stereo camera to detect traversable surfaces.
  *
  * \author Aras Dargazany
  */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// debugging
#include <iostream> //cout

// using namespace
using namespace std;
using namespace cv; //opencv

class StereoVisionProcessing
{
private:
  string left_images;
  string right_images;
  int images_idx;
  Mat frame_0;
  Mat frame_1;
  string input_intrinsic_filename;
  string input_extrinsic_filename ;

public:
  StereoVisionProcessing(const string input_intrinsic_filename, const string input_extrinsic_filename) {} // constructor

  ~StereoVisionProcessing()
  {
  }

  void
  stereo_capture(const int frame_number)
  {

    VideoCapture capture;
    VideoCapture capture_1;

    // camera index 0
    capture.open(0);  //right
    capture_1.open(1);

    // CAMERA_CAMERAINDEX=0
    Mat frame_found_0;
    if (!capture.grab())
    {
      cout << "Can not grab images." << endl;
    }// if
    capture.retrieve(frame_found_0);
    frame_found_0.copyTo(frame_0);

    //////////camera camera index=1
    Mat frame_found_1;
    if (!capture_1.grab())
    {
      cout << "Can not grab images." << endl;
    } // if
    capture_1.retrieve(frame_found_1);
    frame_found_1.copyTo(frame_1);


//    //display the results
//    namedWindow("frame_left", 0);
//    imshow("frame_left", frame_1);
//    namedWindow("frame_right", 0);
//    imshow("frame_right", frame_0);
//    // saving the frames in right and left folders
//    char filename[100];
//    sprintf(filename, "left/left%.2d.png", frame_number);
//    imwrite(filename, frame_1);
//    cout << "Saved " << filename << endl;
//    sprintf(filename, "right/right%.2d.png", frame_number);
//    imwrite(filename, frame_0);
//    cout << "Saved " << filename << endl;


//        std::stringstream ss;
//        ss << dir_name_ << "/" << file_name_ << "_" << boost::posix_time::to_iso_string (boost::posix_time::microsec_clock::local_time ()) << ".pcd";

//    // TODO test
//    std::stringstream ss;
//    ss << left_images << "/" << "left_ss-test" << "_" << frame_number << ".png";
////    char filename_test [100] = (char) ss.str();
////    imwrite( (char*)ss.str(), frame_0);
//    cout << "Saved test ....." << ss.str() << endl;

  } // stereo_capture

  void
  run_camera()
  {

    //saving frames
    int n_r = 1;
    int n_l = 1;
    char filename[200];


    for (images_idx = 0;; images_idx++)
    {

      //stereo capture
      stereo_capture(images_idx);

      /* ------- chessboard detection --------------*/
      //1-variable initalization
      enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
      enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
      //chessboard
      Size boardSize, imageSize;
      boardSize.width = 9;
      boardSize.height = 6;
      Mat cameraMatrix, distCoeffs;
      //////////
      int delay = 1;
      clock_t prevTimestamp = 0;
      int mode = DETECTION;
      vector<vector<Point2f>> imagePoints;
      Pattern pattern = CHESSBOARD;
      bool blink = false;
      vector<Point2f> pointbuf;



      //2-chessboard detection module ---> right camera
      Mat view, view_chessboard, viewGray;
      frame_0.copyTo(view);
      ///
      /////////
      imageSize = view.size();
      ///////////
//      vector<Point2f> pointbuf;
      view.copyTo(viewGray); //gray scale camera
      //cvtColor(view, viewGray, CV_BGR2GRAY); //color camera
      /////////
      bool found = false;
      found = findChessboardCorners(view, boardSize, pointbuf,
                                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE); //stereo_calib
      // improve the found corners' coordinate accuracy
      if (pattern == CHESSBOARD && found)
        cornerSubPix(viewGray, pointbuf, Size(11, 11),
                     Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01)); //0.1 default
      ////////
      if (mode == CAPTURING && found &&
          (clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC))  //!capture.isOpened() ||
      {
        imagePoints.push_back(pointbuf);
        prevTimestamp = clock();
//        blink = capture.isOpened();
      }
      //////////
      view.copyTo(view_chessboard);
      cvtColor(view, view, CV_GRAY2BGR);
      if (found)
        drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
      ///////save the image with the found chessboard  //aras
      namedWindow("ImageView_right_0", 1);
      imshow("ImageView_right_0", view);
      ///////
      Mat frame_found;
      if (found)
        view_chessboard.copyTo(frame_found);

      //3-chessboard detection --->left camera
      //////////left camera camera index=1
      Mat viewl, viewGrayl, viewl_chessboard;
      frame_1.copyTo(viewl);
      //     bool blink = false;
      imageSize = viewl.size();
      //    ///////////
      viewl.copyTo(viewGrayl); //grayscale camera
//          cvtColor(viewl, viewGray, CV_BGR2GRAY); //colorful camera
      /////////
      bool found_1 = false;
      found_1 = findChessboardCorners(viewl, boardSize, pointbuf,
                                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK);
      // improve the found corners' coordinate accuracy
      if (pattern == CHESSBOARD && found_1) cornerSubPix(viewGrayl, pointbuf, Size(11, 11),
            Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
      ////////
      if (mode == CAPTURING && found_1 &&
          (clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC))   //!capture_1.isOpened() ||
      {
        imagePoints.push_back(pointbuf);
        prevTimestamp = clock();
//            blink = capture_1.isOpened();
      }//if
      //////////
      viewl.copyTo(viewl_chessboard);
      cvtColor(viewl, viewl, CV_GRAY2BGR);
      if (found_1)
        drawChessboardCorners(viewl, boardSize, Mat(pointbuf), found_1);
      ///////save the image with the found chessboard  //aras
      namedWindow("ImageView_left_1", 1);
      imshow("ImageView_left_1", viewl);
      ///////
      Mat frame_found_1;
      if (found_1)
        viewl_chessboard.copyTo(frame_found_1);


      ///////saving  images
      char key = waitKey(1);
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


      // key to exit or continiue //used with stereo_rectify
      cout << "number of images captured: " << images_idx << endl;
//      char key = waitKey(1);
      if (key == 27) //esc
        break;


    } // while_old -for_new
  }// run

};//class

int
main(int argc, char** argv)
{

  cout << "---------------> all the left and right images willl be saved in a folder called --> calib <-- folder in a jpeg format .........................." << endl;
  cout << "---------------> all the left and right images willl be saved in a folder called --> calib <-- folder in a jpeg format .........................." << endl;
  cout << "---------------> all the left and right images willl be saved in a folder called --> calib <-- folder in a jpeg format .........................." << endl;
  cout << "---------------> all the left and right images willl be saved in a folder called --> calib <-- folder in a jpeg format .........................." << endl;

  string input_intrinsic_filename, input_extrinsic_filename;

  // Process and display
  StereoVisionProcessing stereo_vision_processing(input_intrinsic_filename, input_extrinsic_filename);
  stereo_vision_processing.run_camera();

  return 0;
}// main
