#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace cv;

using namespace finroc::stereo_traversability_experiments::mlr;
///////////////////////
// Functions

/*static*/ Mat tMLR::toGrayscale(/*INPUT*/const Mat& src) // for imshow or show as an image and visualization and debugging
{
  // only allow one channel
  if (src.channels() != 1)
  {
    CV_Error(/*Error::*/CV_StsBadArg, "Only Matrices with one channel are supported");
  }
  // create and return normalized image // non-linear function
  Mat dst;
  cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC1);  // 8 UNSIGNED CHANNEL 1 -- INT WITH ONE CHANNEL -- 0-255 for every pixel value

  return dst;
}

void tMLR::toNormalize(/*INPUT*/const cv::Mat& src, /*OUTPUT*/cv::Mat& dst, /*INPUT*/const eDATA_TYPE& data_type)
{
  // only allow one channel --> make sure not multi-channel or not multidimensional array
  if (src.channels() != 1)
  {
    CV_Error(/*Error::*/CV_StsBadArg, "Only Matrices with one channel are supported");
  }// if number of channels > 1
  if (data_type == eDATA_TYPE::eIMAGE_DATA)
  {
    cv::normalize(src, dst, 0, /*1 -> 31 PCs*/ 10/*10 -> 32 PCs*/ /*100 ~ 255 real value -> 32 PCs*/ /*1000 -> 32 PCs*/ , NORM_MINMAX, CV_64FC1 /*64 bit DOUBLE*/);
  }// image
  if (data_type == eDATA_TYPE::eDISTANCE_DATA)
  {
    cv::normalize(src, dst, 0, /*1 ~ 0.8 real value -> 1 PCs*/ /*10 -> 3 PCs*/ /*100 -> 4 PCs*/ 1000/*1000 -> 5 PCs*/ /*10000 -> 5 PCs*/ /*100000 -> 5PCs*/ /*1e5*/, NORM_MINMAX, CV_64FC1 /*64 bit DOUBLE*/);
  }// dist
  //  if (data_type == eDATA_TYPE::eLOCALIZATION_DATA)  // I dont know if this makes SENSE at all -- need more digging and research
  //  {
  //    cv::normalize(src, dst, 0, /*1 -> 1 PC*/ 10 /*10 -> 3 PCs*/ /*100 -> 3 PCs*/ /*1000 -> 3 PCs*/, NORM_MINMAX, CV_64FC1 /*64 bit DOUBLE*/);
  //  }// loc
  //  if (data_type == eDATA_TYPE::eALL_DATA)  // I dont know if this makes SENSE at all -- need more digging and research
  //  {
  //    cv::normalize(src, dst, 0, /*1 -> 2 PCs*/ /*10 -> 23 PCs*/ /*100 -> 28 PCs*/ 1000 /*1000 -> 32 PCs*/ /*10000 -> 32 PCs*/, NORM_MINMAX, CV_64FC1 /*64 bit DOUBLE*/);
  //  }// loc
}// toNormalize()

void tMLR::toNormalize(/*INPUT*/const vector<Mat>& src, /*OUTPUT*/vector<Mat>& dst, /*INPUT*/const eDATA_TYPE& data_type)
{

  for (vector<Mat>::const_iterator iter = src.begin(); iter < src.end(); ++iter)
  {
    Mat I_RAW(*iter);

    //if (data_type == eDATA_TYPE::eIMAGE_DATA || data_type == eDATA_TYPE::eDISTANCE_DATA)
    {
      string I_RAW_name = "input data RAW"; //format("%s/input_image_%d.png"/*, output_folder.c_str(), input_data_number*/);
      namedWindow(I_RAW_name, WINDOW_NORMAL);
      //LOG_PRINT(I_RAW, is_active);
      imshow(I_RAW_name, I_RAW);

      cv::Mat I_norm;
      toNormalize(I_RAW, I_norm, data_type); // normalizing the data for learning PCs
      dst.push_back(I_norm);
      /*char key =*/ waitKey(/*delay*/1);
      //if (key == "q" || key == 27 /*ESC*/){}
    }// block
  }// for each iteration
}// toNormlize()

///*static*/ Mat tMLR::toGrayscale(InputArray _src)
//{
//  Mat src = _src.getMat();
//  // only allow one channel
//  if (src.channels() != 1)
//  {
//    CV_Error(/*Error::*/CV_StsBadArg, "Only Matrices with one channel are supported");
//  }
//  // create and return normalized image
//  Mat dst;
//  cv::normalize(_src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
//  return dst;
//}
//
//// Normalizes a given image into a value range between 0 and 255.
//Mat tMLR::norm_0_255(const Mat& src)
//{
//  // Create and return normalized image:
//  Mat dst;
//  switch (src.channels())
//  {
//  case 1:
//    cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
//    break;
//  case 3:
//    cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC3);
//    break;
//  default:
//    src.copyTo(dst);
//    break;
//  }
//  return dst;
//}
