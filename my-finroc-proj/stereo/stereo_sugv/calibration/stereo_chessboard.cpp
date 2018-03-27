/** \brief StereoVisioProcessing is an application for processing stereo images to estimate terrain traversability using stereo camera to detect traversable surfaces.
  *
  * \author Aras Dargazany
  */

// system files
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <FlyCapture2.h>

// debugging
#include <iostream>
#include <vector>
#include <stdio.h>

// using namespace
using namespace std;
using namespace cv; //opencv
using namespace FlyCapture2;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

struct chessboardDataOutput
{
  Mat view, view_chessboard;
  bool found = false;
};

class StereoVisionProcessing
{
private:

  BusManager bus_manager;
  PGRGuid pointgrey_guid;
  GigECamera camera;
  Image flycap_img;
  GigECamera camera_2;
  Image flycap_img_2;
  Image colorImage;
  bool bInitialized = false;

  // changing the resolution of images
  GigEImageSettings imageSettings;


  //saving frames
  int images_idx;
  int n_l = 1, n_r = 1;
  char filename[200];

  /* ------- chessboard detection --------------*/
  Size boardSize, imageSize;
  Mat cameraMatrix, distCoeffs;
  int delay = 1;
  clock_t prevTimestamp = 0;
  int mode = DETECTION;
  vector<vector<Point2f>> imagePoints;
  Pattern pattern = CHESSBOARD;
  bool blink = false;
  vector<Point2f> pointbuf;
  bool found = false;
  chessboardDataOutput data_1, data_2;


public:
  StereoVisionProcessing()
  {

    //changing the resolution
    //GigEImageSettings imageSettings;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.height = 900; //480;  //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
    imageSettings.width = 1200; //640; //VIDEOMODE_640x480Y8, /**< 640x480 8-bit. */
    imageSettings.pixelFormat = PIXEL_FORMAT_MONO8;

    //RunSingleCamera(pointgrey_guid);
    bus_manager.GetCameraFromIndex(0, &pointgrey_guid); //left eth1
    camera.Connect(&pointgrey_guid);
    camera.SetGigEImageSettings(&imageSettings);
    camera.StartCapture();

    bus_manager.GetCameraFromIndex(1, &pointgrey_guid); //right eth2
    camera_2.Connect(&pointgrey_guid);
    camera_2.SetGigEImageSettings(&imageSettings);
    camera_2.StartCapture();

    boardSize.width = 9;
    boardSize.height = 6; //every cell i tested is 5*5 cm -- the same as the size of the my cells in grid map :D

  } // constructor

  ~StereoVisionProcessing()
  {
    // Stop capturing images
    camera.StopCapture();
    camera_2.StopCapture();

    // Disconnect the camera
    camera.Disconnect();
    camera_2.Disconnect();
  }

  IplImage* ConvertImageToOpenCV(Image* pImage)
  {
    IplImage* cvImage = NULL;
    bool bColor = true;
    CvSize mySize;
    mySize.height = pImage->GetRows();
    mySize.width = pImage->GetCols();

    switch (pImage->GetPixelFormat())
    {
    case PIXEL_FORMAT_MONO8:
      cvImage = cvCreateImageHeader(mySize, 8, 1);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 1;
      bColor = false;
      break;
    case PIXEL_FORMAT_411YUV8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_422YUV8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_444YUV8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_RGB8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_MONO16:
      cvImage = cvCreateImageHeader(mySize, 16, 1);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 1;
      bColor = false;
      break;
    case PIXEL_FORMAT_RGB16:
      cvImage = cvCreateImageHeader(mySize, 16, 3);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_S_MONO16:
      cvImage = cvCreateImageHeader(mySize, 16, 1);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 1;
      bColor = false;
      break;
    case PIXEL_FORMAT_S_RGB16:
      cvImage = cvCreateImageHeader(mySize, 16, 3);
      cvImage->depth = IPL_DEPTH_16U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_RAW8:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_RAW16:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_MONO12:
      printf("Not supported by OpenCV");
      bColor = false;
      break;
    case PIXEL_FORMAT_RAW12:
      printf("Not supported by OpenCV");
      break;
    case PIXEL_FORMAT_BGR:
      cvImage = cvCreateImageHeader(mySize, 8, 3);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 3;
      break;
    case PIXEL_FORMAT_BGRU:
      cvImage = cvCreateImageHeader(mySize, 8, 4);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 4;
      break;
    case PIXEL_FORMAT_RGBU:
      cvImage = cvCreateImageHeader(mySize, 8, 4);
      cvImage->depth = IPL_DEPTH_8U;
      cvImage->nChannels = 4;
      break;
    default:
      printf("Some error occured...\n");
      return NULL;
    }

    if (bColor)
    {
      if (!bInitialized)
      {
        colorImage.SetData(new unsigned char[pImage->GetCols() * pImage->GetRows() * 3], pImage->GetCols() * pImage->GetRows() * 3);
        bInitialized = true;
      }

      pImage->Convert(PIXEL_FORMAT_BGR, &colorImage); //needs to be as BGR to be saved

      cvImage->width = colorImage.GetCols();
      cvImage->height = colorImage.GetRows();
      cvImage->widthStep = colorImage.GetStride();

      cvImage->origin = 0; //interleaved color channels

      cvImage->imageDataOrigin = (char*)colorImage.GetData(); //DataOrigin and Data same pointer, no ROI
      cvImage->imageData         = (char*)(colorImage.GetData());
      cvImage->widthStep    = colorImage.GetStride();
      cvImage->nSize = sizeof(IplImage);
      cvImage->imageSize = cvImage->height * cvImage->widthStep;
    }
    else
    {
      cvImage->imageDataOrigin = (char*)(pImage->GetData());
      cvImage->imageData         = (char*)(pImage->GetData());
      cvImage->widthStep         = pImage->GetStride();
      cvImage->nSize             = sizeof(IplImage);
      cvImage->imageSize         = cvImage->height * cvImage->widthStep;

      //at this point cvImage contains a valid IplImage
    }
    return cvImage;
  }

  void
  stereo_capture()
  {

    for (;;)
    {

      camera.RetrieveBuffer(&flycap_img);
      camera_2.RetrieveBuffer(&flycap_img_2);
      IplImage* opencv_img   = ConvertImageToOpenCV(&flycap_img);
      IplImage* opencv_img_2 = ConvertImageToOpenCV(&flycap_img_2);
      //cvShowImage("left", opencv_img);
      //cvShowImage("right", opencv_img_2);
      Mat left_img(opencv_img), right_img(opencv_img_2);
      namedWindow("left_img", 0);
      imshow("left_img", left_img);
      namedWindow("right_img", 0);
      imshow("right_img", right_img);
      waitKey(1);
      cvReleaseImageHeader(&opencv_img);
      cvReleaseImage(&opencv_img);
      cvReleaseImageHeader(&opencv_img_2);
      cvReleaseImage(&opencv_img_2);

    }//for

  } // stereo_capture

  chessboardDataOutput chessboard_detection(const IplImage* opencv_img_input)
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
  }//chessboard_detection

  void
  stereo_chessboard_detection()
  {

    for (images_idx = 0;; images_idx++)
    {

      // capturing the left image or camera 0
      camera.RetrieveBuffer(&flycap_img);
      IplImage* opencv_img   = ConvertImageToOpenCV(&flycap_img);
      data_1 = chessboard_detection(opencv_img);
      namedWindow("left and camera 0", 1);
      imshow("left and camera 0", data_1.view);

      // capturing the right image or camera 1
      camera_2.RetrieveBuffer(&flycap_img_2);
      IplImage* opencv_img_2 = ConvertImageToOpenCV(&flycap_img_2);
      data_2 = chessboard_detection(opencv_img_2);
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

  }// run

};//class

int
main(int argc, char** argv)
{

  cout << "---------------> all the left and right images will be saved in a folder called --> calib <-- folder in a jpeg format .........................." << endl;


  // Process and display
  StereoVisionProcessing stereo_vision_processing;
  //stereo_vision_processing.stereo_capture();
  stereo_vision_processing.stereo_chessboard_detection();

  return 0;
}// main
