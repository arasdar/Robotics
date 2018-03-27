
/*
 * stereoCapture.cpp
 *
 *  Created on: May 12, 2014
 *      Author: aras
 */

#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.h"

using namespace finroc::icarus::sensor_processing::stereo_sugv::capture;

void
mStereo::stereoCapture()
{

  camera.RetrieveBuffer(flycap_img);
  camera_2.RetrieveBuffer(flycap_img_2);
  IplImage* left_IplImage   = ConvertImageToOpenCV(flycap_img);
  IplImage* right_IplImage = ConvertImageToOpenCV(flycap_img_2);

  /*
    //only for testing
    cvShowImage("left IplImage opencv", left_IplImage);
    cvShowImage("right IplImage opencv", right_IplImage);
  //  waitKey(1);
  */

  /*
        char key = waitKey(1);
        if (key == 'q'){
          // Stop capturing images
          camera.StopCapture();
          camera_2.StopCapture();

          // Disconnect the camera
          camera.Disconnect();
          camera_2.Disconnect();
        }
  */

  Mat left_img_opencv(left_IplImage),
      right_img_opencv(right_IplImage);//IplImage * ipl = ...; //cv::Mat m = cv::cvarrToMat(ipl);  // default additional arguments: don't copy data.

  cvReleaseImageHeader(&left_IplImage);
  cvReleaseImage(&left_IplImage);
  cvReleaseImageHeader(&right_IplImage);
  cvReleaseImage(&right_IplImage);

  //pass the images to stereo_rectify
  left_img_opencv.copyTo(input_images_left);
  right_img_opencv.copyTo(input_images_right);

  // saving the frames in right and left folders
  //TODO also creating left and right folders automatically
  char filename[100];
  sprintf(filename, "left/left%.4d.png", images_idx);
  imwrite(filename, left_img_opencv);
  cout << "Saved " << filename << endl;
  sprintf(filename, "right/right%.4d.png", images_idx);
  imwrite(filename, right_img_opencv);
  cout << "Saved " << filename << endl;

} // stereo_capture







