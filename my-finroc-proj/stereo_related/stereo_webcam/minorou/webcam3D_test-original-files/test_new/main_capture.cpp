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


int main_img2pcd()
{
  int w = 640, h = 480;
  Camera cl("/dev/video1", w, h, 15); //left camera object using libv4lcam2
  Camera cr("/dev/video0", w, h, 15); //right camera object

  IplImage *l = cvCreateImage(cvSize(w, h), 8, 3); //left image
  IplImage *r = cvCreateImage(cvSize(w, h), 8, 3); //right image

  char key = 'a';

  cvNamedWindow("left", 0);
  cvNamedWindow("right", 0);

  int k = 0;

  while (key != 27)
  {
    cl.Update(&cr);
    cl.toIplImage(l);
    cr.toIplImage(r);

    cvShowImage("left", l);
    cvShowImage("right", r);

    key = cvWaitKey(1);
  }// while

  cvDestroyAllWindows();
}

int main(int ac, char** av)
{
  main_img2pcd();

  return 0;
}
