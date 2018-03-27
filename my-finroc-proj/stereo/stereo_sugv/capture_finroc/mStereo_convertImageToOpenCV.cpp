/*
 * tStereoProc_convertImageToOpencv.cpp
 *
 *  Created on: May 12, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/stereo_sugv/capture_finroc/mStereo.h"

using namespace finroc::icarus::sensor_processing::stereo_sugv::capture;

IplImage*
mStereo::ConvertImageToOpenCV(Image* pImage)
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

  /*
    // convertImageToOpenCV
    Image colorImage;
    bool bInitialized = false;

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
  */
  if (!bColor)
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








