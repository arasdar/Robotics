//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"

/*! mask_tmpl.cpp */
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>

#include <boost/filesystem.hpp>

//#include "projects/stereo_traversability_experiments/def/read_csv.h"
#include "projects/stereo_traversability_experiments/def/main_pca.h"

using namespace std;
using namespace cv;

int main(int argc, const char** argv)
{

  // First of all we need to read in the data
  // Check for valid command line arguments, print usage
  // if no arguments were given.
  if (argc < 2)
  {
    cout << "usage: " << argv[0] << " <input folder for reading from> <output_folder for writing into> " << endl;
    exit(1);
  }
  /*const */string output_folder; // = "";
  if (argc == 3)
  {
    output_folder = string(argv[2]);
  }

  // Get list of images in a directory -- mass reading and iterating through a directory
  std::vector<std::string> images_path;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    images_path.push_back(itr->path().string());
  }
  sort(images_path.begin(), images_path.end());

  // These vectors hold the images and corresponding labels.
  /*const*/ vector<Mat> images; // these are input images and that s why they are defined as constant to make sure they are not changed any where!!
  /*const*/ Size input_img_size;

  // reading the images path from the vector and loading image vector
  for (vector<string>::iterator iter = images_path.begin(); iter != images_path.end(); ++iter)
    //for (unsigned int it = 0;it < images.path.size(); it++){}
  {
    string path = *iter;
    if (!path.empty() /*&& !classlabel.empty()*/)
    {
      //C++: Mat imread(const string& filename, int flags=1 )¶
      unsigned int flags = CV_LOAD_IMAGE_GRAYSCALE; //CV_LOAD_IMAGE_GRAYSCALE - If set, always convert image to the grayscale one
      Mat img = imread(path, flags);
      input_img_size = img.size();
      images.push_back(/*imread(path, 0)*/img);
      cout << path << " loadeddddddddddd" << endl;
      imshow(/*format("images_%d", t)*/"loaded image", img/*.getMat(t)*/); // getMat for InputArray or InputArrayOfArray or OutputArray

      cout << "img.size: " << img.size() << endl
           << "type: " << img.type() << endl
           << "depth: " << img.depth() << endl
           << "channels; " << img.channels() << endl
           << "elemSize: " << img.elemSize() << endl
           << "step1: " << img.step1() << endl
           << "isContinuous: " << img.isContinuous() << endl
           << "total: " << img.total() << endl;

      char key = waitKey(1);
      if (key == 'q')
      {
        exit(1);
      }
    }

  }


//  // sub-sampling or sampling or re-sampling - sampling: gathering or collecting samples
////  const unsigned int X = 100, Y = 100;
//  const unsigned int X = 80, Y = 80; // width and height of the patch
////  const unsigned int X = 20, Y = 20;
//// const unsigned int X = 10, Y = 10;
////  const unsigned int X = 5, Y = 5;
////  const unsigned int X = 3, Y = 3;
//  Size patch_size(X, Y);
//  // cell used to rounding up the value & floor to round down the value
//  Size patch_center(patch_size / 2/*cell(patch_size/2)*/); // just in case the patch size was odd not even number
//  Point2i start(patch_center); //((patch_size.height)/2, (patch_size.width)/2); //starting point for the sampling
//  Point2i end(input_img_size - patch_center/*(patch_size / 2)*/);
//
//  //patchType – Depth of the extracted pixels. By default, they have the same depth as src .
//  const int patch_type = -1; // this is default??
//
//  unsigned int num_patches_per_image = (end - start).x * (end - start).y;
//  unsigned int total_num_patches = num_patches_per_image * images.size();
//  /*const*/ vector<Mat> images_patches;
//  images_patches.resize(num_patches_per_image);
//
//  // loading the image sub-samples in an mat array
//  for (vector<Mat>::iterator iter = images.begin(); iter < images.end(); ++iter)
//  {
//    //warning: comparison between signed and unsigned integer expressions
//    for (/*unsigned */int y = start.y; y < end.y; ++y) //image height or mat rows or ith row or y
//    {
//      for (/*unsigned*/ int x = start.x; x < end.x; ++x) //image width or mat columns or jth column or x
//      {
//
//        Point2f center(x, y);
//        Mat img = *iter;
//        Mat extracted_patch;
//        extracted_patch.create(patch_size, img.type());
//
//        //Retrieves a pixel rectangle from an image with sub-pixel accuracy.
//        //C++: void getRectSubPix(InputArray image, Size patchSize, Point2f center, OutputArray patch, int patchType=-1 )
//        getRectSubPix(img, patch_size, center, extracted_patch, patch_type);
//        imshow("extracted_patch", extracted_patch);
//        namedWindow("extracted_patch");
//
//        images_patches.push_back(extracted_patch);
//
//        cout << "num_patches_per_image: " << num_patches_per_image << endl;
//        cout << "total_num_patches: " << total_num_patches << endl;
//
//        cout << "patches_size: " <<  patch_size << endl;
//        cout << "patches_center: " << patch_center << endl;
//
//        cout << " currently extracted_patch info such as: size: " << extracted_patch.size() << endl
//             << "type: " << extracted_patch.type() << endl
//             << "depth: " << extracted_patch.depth() << endl
//             << "channels; " << extracted_patch.channels() << endl
//             << "elemSize: " << extracted_patch.elemSize() << endl
//             << "step1: "<< extracted_patch.step1() << endl
//             << "isContinuous: "<< extracted_patch.isContinuous() << endl
//             << "total: " << extracted_patch.total() << endl;
//
//
//        char key = waitKey(1);
//        if (key == 'q')
//        {
//          exit(1);
//        }
//
//      } //scanning every single column of the image matrix
//    } // scanning every single row of the image matrix
//  }// iterating through images

  // Now that all the patches are extracted, lets free a bit of RAM and Cache'
  //images.clear();

  // Let do the learning here
//  // observations in row
//  //inline Mat asRowMatrix(InputArrayOfArrays src, int rtype, double alpha = 1, double beta = 0)
//  //Mat data = asRowMatrix(_src, CV_64FC1);
//  const int rtype = CV_64FC1;
//  const int alpha =1, beta = 0;
//
//  // number of samples
//  size_t n = images_patches.size(); //src.total();
//
//  // dimensionality of (reshaped) samples
//  size_t d = images_patches[0].total(); //src.getMat(0).total();
//
//  // create data matrix
//  Mat data((int)n, (int)d, rtype);
//  // now copy data
//  for (unsigned int i = 0; i < n; i++)
////    for (vector<Mat>::iterator iter = images_patches.begin();iter < images_pactehs.end(); ++iter)
//  {
//    Mat patch = images_patches[i];
//    // get a hold of the current row
//    Mat xi = data.row(i);
//    // make reshape happy by cloning for non-continuous matrices
//    if (/*src.getMat(i)*/patch.isContinuous())
//    {
//      /*src.getMat(i)*/patch.reshape(1, 1).convertTo(xi, rtype, alpha, beta);
//    }
//    else
//    {
//      /*src.getMat(i)*/patch.clone().reshape(1, 1).convertTo(xi, rtype, alpha, beta);
//    }
//  }
//
//  int _num_components(n); // number of eigen_values
//  Mat _mean, _eigenvalues, _eigenvectors;
//
//  // perform the PCA
//  PCA pca(data, Mat(), PCA::DATA_AS_ROW, _num_components);
//  // copy the PCA results
//  _mean = pca.mean.reshape(1, 1); // store the mean vector
//  _eigenvalues = pca.eigenvalues.clone(); // eigenvalues by row
//  transpose(pca.eigenvectors, _eigenvectors); // eigenvectors by column
//
//  imshow("_mean", _mean);
//  waitKey();

  main_pca(images/*_patches*/);

  return 0; //exit
}
