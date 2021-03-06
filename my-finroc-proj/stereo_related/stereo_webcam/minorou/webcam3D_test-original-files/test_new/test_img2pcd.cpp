// new one -- img2pcd, more resolution
#include <stdio.h>
#include <stdlib.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"

//#include </home/aras/libv4l2cam/libv4l2cam/libcam.h>
//#include <libcam.h>
#include "libcam.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //pointer to pointcloud

int main_img2pcd()
{
  int w = 640, h = 480;
  Camera cl("/dev/video1", w, h, 15); //left camera object using libv4lcam2
  Camera cr("/dev/video0", w, h, 15); //right camera object

  CvMat *mxl = (CvMat *) cvLoad("mx1.xml"); //map for remapping along X direction in the left image
  CvMat *mxr = (CvMat *) cvLoad("mx2.xml"); //map for remapping along X direction in the right image
  CvMat *myl = (CvMat *) cvLoad("my1.xml");
  CvMat *myr = (CvMat *) cvLoad("my2.xml");
  CvMat *Q   = (CvMat *) cvLoad("Q.xml");   //the Q matrix

  IplImage *l = cvCreateImage(cvSize(w, h), 8, 3); //left image
  IplImage *r = cvCreateImage(cvSize(w, h), 8, 3); //right image

  IplImage *lg = cvCreateImage(cvSize(w, h), 8, 1); //greyscale left image
  IplImage *rg = cvCreateImage(cvSize(w, h), 8, 1); //greyscale right image

  CvMat *lr = cvCreateMat(h, w, CV_8U);  //remapped left image
  CvMat *rr = cvCreateMat(h, w, CV_8U);  //remapped right image

  CvMat *disp = cvCreateMat(h, w, CV_32F);  //diaprity image
  CvMat *vdisp = cvCreateMat(h, w, CV_8U);  //normalized diaprity image
  CvMat *xyz = cvCreateMat(h, w, CV_32FC3); //matrix holding X Y Z values

  CvStereoBMState *BMState = cvCreateStereoBMState(); //the BMState object
  assert(BMState != 0);
  BMState->preFilterSize = 41;
  BMState->preFilterCap = 31;
  BMState->SADWindowSize = 41;
  BMState->minDisparity = 0;
  BMState->numberOfDisparities = (((cvGetSize(l).width / 8) + 15) & -16);
  //BMState->numberOfDisparities=128;
  BMState->textureThreshold = 10;
  BMState->uniquenessRatio = 15;

  char key = 'a';

  cloud->height = h;
  cloud->width = w;
  cloud->is_dense = false;
  cloud->points.resize(cloud->height * cloud->width);

  pcl::visualization::CloudViewer viewer("Point Cloud");

  cvNamedWindow("left", 0);
  cvNamedWindow("right", 0);
  cvNamedWindow("disparity", 0);

  int k = 0;

  while (key != 27)
  {
    cl.Update(&cr);
    cl.toIplImage(l);
    cr.toIplImage(r);

    cvCvtColor(l, lg, CV_BGR2GRAY);
    cvCvtColor(r, rg, CV_BGR2GRAY);

    cvShowImage("left", l);
    cvShowImage("right", r);

//    cvRemap(lg, lr, mxl, myl);
//    cvRemap(rg, rr, mxr, myr);
//
//    cvFindStereoCorrespondenceBM(lr, rr, disp, BMState);
//    cvNormalize(disp, vdisp, 0, 256, CV_MINMAX);
//
//    cvShowImage("disparity", vdisp);
//
//    cvReprojectImageTo3D(disp, xyz, Q);
//
//    for (int i = 0; i < (xyz->rows); i++)
//    {
//      const float *dataptr = (const float*)(xyz->data.ptr + (i * xyz->step));
//      for (int j = 0; j < (xyz->cols); j++, k++)
//      {
//        cloud->points[k].x = dataptr[0];
//        cloud->points[k].y = dataptr[1];
//        cloud->points[k].z = dataptr[2];
//        dataptr += 3;
//      }
//    }
//
//    //set centre point to (0,0,0) as camera reference
//    cloud->points[(cloud->width >> 1) * (cloud->height + 1)].x = 0;
//    cloud->points[(cloud->width >> 1) * (cloud->height + 1)].y = 0;
//    cloud->points[(cloud->width >> 1) * (cloud->height + 1)].z = 0;
//
//    k = 0;
//
//    viewer.showCloud(cloud);

    key = cvWaitKey(10);
  }// while

  cvDestroyAllWindows();
}

int main(int ac, char** av)
{
  main_img2pcd();

  return 0;
}
