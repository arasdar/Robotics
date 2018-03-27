//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"

/*! mask_tmpl.cpp */
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>

using namespace std;
using namespace cv;

const unsigned int cPATCH_WINDOW_SIZE_RECT = 11;  // be carefull this is window will be devided by two so that ceneter should be a pixel
const unsigned int X = cPATCH_WINDOW_SIZE_RECT, Y = cPATCH_WINDOW_SIZE_RECT; // width and height of the patch
const Size patch_size(X, Y);
// cell used to rounding up the value & floor to round down the value
const Size patch_center((patch_size / 2).width + 1, (patch_size / 2).height + 1); // just in case the patch size was odd not even number
const Point2i start(patch_center); //((patch_size.height)/2, (patch_size.width)/2); //starting point for the sampling


//int main(int argc, const char** argv)
void /*extract_patches*/subsampling(const vector<Mat>& images, Size& conv_img_size, vector<Mat>& images_patches, unsigned int& num_patches_per_image) // passing to the function should be done by memory address or referencing
//    /*convolved_image_size*/ /*this can also be know as the image which has been scanned for extracting patches*/)
{


  cout << "cPATCH_WINDOW_SIZE_RECT: " << cPATCH_WINDOW_SIZE_RECT << endl;
  cout << "patch_size: " << patch_size << endl;
  cout << "patch_center: " << patch_center << endl;
  cout << "starting point in image: " << start << endl;


  // sub-sampling or sampling or re-sampling - sampling: gathering or collecting samples
  const Size input_img_size(images[0].size());
  const Point2i end(input_img_size - patch_center/*(patch_size / 2)*/);
  /*const unsigned int*/ num_patches_per_image = (end - start).x * (end - start).y;
  const unsigned int total_num_patches = num_patches_per_image * images.size();

  cout << "input_img_size: " << input_img_size << endl;
  cout << "end: " << end << endl;
  cout << "number of patches (sub-samples) per image: " << num_patches_per_image << endl;
  cout << "number of input images (samples): " << images.size() << endl;
  cout << "total_number of patches(sub-samples) from samples: or number of sub-smaples in our pooling or pool before learning: " << total_num_patches << endl;

  conv_img_size.width = (end - start).x;
  conv_img_size.height = (end - start).y;

  cout << "size of the convolved image: width or columns or cols or j or X: " << conv_img_size.width << endl
       << "size of the convolved image: height or rows or i: " << conv_img_size.height << endl;

  cout << "conv_img_size: (width, height) or (cols, rows) or (X, Y) or (j, i): " << conv_img_size << endl;

  //patchType – Depth of the extracted pixels. By default, they have the same depth as src .
  const int patch_type = images[0].type(); // this is default??

  // this is done for image show and also printed log and generally speaking for debugging purposes
  unsigned int delay = 0; // stop as default
  char key = 's'; // this is for string "s"; // stop as default
  /*"string" && 'char'*/

  // iterating through images or input samples
  for (vector<Mat>::const_iterator iter = images.begin(); iter < images.end(); ++iter)
  {
    // loading the image patches or reading sub-samples in an mat array
    //warning: comparison between signed and unsigned integer expressions
    for (/*unsigned */int y = start.y; y < end.y; ++y) //image height or mat rows or ith row or y
    {
      for (/*unsigned*/ int x = start.x; x < end.x; ++x) //image width or mat columns or jth column or x
      {

        Point2f center(x, y);
        Mat img = *iter;
        Mat extracted_patch;
        //extracted_patch.create(patch_size, img.type());

        //Retrieves a pixel rectangle from an image with sub-pixel accuracy.
        //C++: void getRectSubPix(InputArray image, Size patchSize, Point2f center, OutputArray patch, int patchType=-1 )
        getRectSubPix(img, patch_size, center, extracted_patch, patch_type/*CV_8UC1*//*CV_32F*/);

        cout << "before normalization & channel check extracted_patch.type(): " << extracted_patch.type() << endl;
        Mat src = extracted_patch.clone();
        // only allow one channel
        if (src.channels() != 1)
        {
          CV_Error(Error::StsBadArg, "Only Matrices with one channel are supported");
        }
        // create and return normalized image // non-linear function
        Mat dst;
        cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC1); // 0-255 and one channel -- complete grayscale image & normalized
        extracted_patch = dst.clone();
        cout << "after normalization & channel check extracted_patch.type(): " << extracted_patch.type() << endl;


        namedWindow("extracted_patch", WINDOW_NORMAL);
        imshow("extracted_patch", extracted_patch);

        images_patches.push_back(extracted_patch);

        cout << "num_patches_per_image: " << num_patches_per_image << endl;
        cout << "total_num_patches: " << total_num_patches << endl;

        cout << "patches_size: (width, height) or (cols, rows) or (X, Y) or (j, i) " <<  patch_size << endl;
        cout << "patches_center: (width, height) or (cols, rows) or (X, Y) or (j, i): " << patch_center << endl;

        cout << " currently extracted_patch info such as: size: (width, height) or (cols, rows) or (X, Y) or (j, i) " << extracted_patch.size() << endl
             << "type: " << extracted_patch.type() << endl
             << "depth: " << extracted_patch.depth() << endl
             << "channels; " << extracted_patch.channels() << endl
             << "elemSize: " << extracted_patch.elemSize() << endl
             << "step1: " << extracted_patch.step1() << endl
             << "isContinuous: " << extracted_patch.isContinuous() << endl
             << "total: " << extracted_patch.total() << endl;

        cout << "c to continue, s to stop or pause, q or ESC for breaking the loop and exiting..." << endl;

        // initial delay = 0 --> wait for input command
        key = waitKey(delay);
        if (key == 'q' || key == 27)
        {
          break/*exit(1)*/;
        }
        if (key == 'c' /*continue*/)
        {
          delay = 1;
        } // meaning i milisecond delay}
        if (key == 's' /*stop*/)
        {
          delay = 0; /*pause oor stop the loop*/
        }

      } //scanning every single column of the image matrix
      if (key == 'q' || key == 27)
      {
        break/*exit(1)*/;
      }
    } // scanning every single row of the image matrix
    if (key == 'q' || key == 27)
    {
      break/*exit(1)*/;
    }
  }// iterating through images


  cout << "images_patches.size(): STL:vector<Mat> " << images_patches.size() << endl;

  // size the new  concolved images
  // only the number of rows needed  // these two should be equal but they are NOT!!!!!!!!!!
  unsigned int num_rows /*for the convolved images*/ = images[0].rows /*one sample of the input images*/ - (cPATCH_WINDOW_SIZE_RECT - 1);
  unsigned int num_cols /*for the convolved images*/ = images[0].cols /*one sample of the input images*/ - (cPATCH_WINDOW_SIZE_RECT - 1);
  cout << "images[0].rows - (cPATCH_WINDOW_SIZE_RECT - 1): " << num_rows << endl;
  cout << "images[0].cols - (cPATCH_WINDOW_SIZE_RECT - 1): " << num_cols  << endl;
  cout << "conv_img_size: (width, height) = (w, h) = (cols, rows) = (j, i) = (X, Y)  " << conv_img_size << endl;
  waitKey(1); // just to make sure that we can see these info


}












int main(int argc, const char** argv)
{

  /*! This part acts like a memory or a data base */
  // Read the original images from the database & memory
  vector<Mat> images;
  images = sampling_csv(argc, argv); // this is using original CSV filem or TXT file

//  for (;;)
//  {}

//    //sub-sampling here and extracting the sub-samples or window patches
//    Size size;
//    vector<Mat> images_patches;
//    unsigned int num_patches_per_img;
//    /*extract_patches*/subsampling(images, size, images_patches, num_patches_per_img);

  // Let do the learning here
//    images.clear(); // to use for the conv images
  ////*vector<Mat> convolved_samples =*/ learning(images_patches/*, img_patch_mean*/, size, images, num_patches_per_img); // images - mean??

  /*vector<Mat> convolved_samples =*/ learning(images/*, img_patch_mean*/, size, images, num_patches_per_img); // images - mean??


  return 0; //exit
}
