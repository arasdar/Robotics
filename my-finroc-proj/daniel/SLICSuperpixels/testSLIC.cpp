
/*SLIC class declaration and implementation files are provided along with a MS VC+6 workspace. The files provide the code to perform superpixel segmentation as explained in the paper:

"SLIC Superpixels Compated to State-of-the-art Superpixel Methods",
Radhakrishna Achanta, Appu Shaji, Kevin Smith, Aurelien Lucchi, Pascal Fua, and Sabine Susstrunk.
IEEE TPAMI, November2012.

The usage is quite straight forward if one wants to incorporate SLICO in his/her projects. One has to instantiate an object of the SLIC class and call the various methods on it. Here is an example main() file:

====================================================================================================
NOTE: For commercial use please contact the author Radhakrishna Achanta (firstname.lastname@epfl.ch)
====================================================================================================
*/

#include <string>
#include <cmath>
#include "projects/stereo_traversability_experiments/daniel/SLICSuperpixels/SLIC.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;


void ReadImage(string path, uint *buffer)
{
  Mat image = imread(path, CV_LOAD_IMAGE_COLOR);   // Read the file

  if (! image.data)                              // Check for invalid input
  {
    cout <<  "Could not open or find the image" << std::endl ;
    return;
  }

  uchar r, g, b;

  // translate from opencv BGR to SLIC RGB(A)
  for (int i = 0; i < image.rows; i++)
  {
    for (int j = 0; j < image.cols; j++)
    {
      Vec3b pixelValue = image.at<Vec3b>(i, j);
      b = pixelValue.val[0]; // B
      g = pixelValue.val[1]; // B
      r = pixelValue.val[2]; // B
      if (i == j && i == 100)
      {
        std::cout << "OpenCV (bgr, 100, 100): " << uint(b) << " " << uint(g)
                  << " " << uint(r) << std::endl;
      }
      // store as rgb value in buffer
      unsigned int rgba = (r << 16) | (g << 8) | b;
      buffer[i * image.cols + j] = rgba;
    }
  }
  std::cout << "poixlpos: " << image.cols << std::endl;

  std::cout << "Buffer (rgb, 100, 100): " << uint(((buffer[100 * image.cols + 100]) >> 16) & 0xFF)  << " " << uint(((buffer[100 * image.cols + 100]) >> 8) & 0xFF)
            << " " << uint(((buffer[100 * image.cols + 100]) >> 0) & 0xFF) << " " << buffer[100 * image.cols + 100] << std::endl;

}


void SaveSegmentedImage(string path, uint *buffer, int w, int h)
{
  Mat image(h, w, CV_8UC3); // rows, cols, type
  uint r, g, b;

  // translate from SLIC RGB(A) to opencv BGR
  for (int i = 0; i < image.rows; i++)
  {
    for (int j = 0; j < image.cols; j++)
    {
      unsigned int rgba = buffer[i * image.cols + j];
      r = (rgba >> 16) & 0xFF;
      g = (rgba >> 8) & 0xFF;
      b = (rgba) & 0xFF;

      // store as bgr value in Mat20.ppm"
      Vec3b pixelValue;
      pixelValue.val[0] = uchar(b); // B
      pixelValue.val[1] = uchar(g); // G
      pixelValue.val[2] = uchar(r); // R
      image.at<Vec3b>(i, j) = pixelValue;
    }
  }

  imwrite(path, image);
}

int main(int argc, char** argv)
{
  int k = 200;//Desired number of superpixels.
  double m = 10;//Compactness factor. use a value ranging from 10 to 40 depending on your needs. Default is 10
  int width(1280), height(960); // default

  //read arguments
  if (argc < 2)
  {
    printf("\n Usage   : %s image (need absoulte path) Options", argv[0]);
    printf("\n   [Options : value by default indicated in ()]");
    printf("\n            -----------------------------------------");
    printf("\n            -k (200) number of superpixels ");
    printf("\n            -m (10) compactness factor ");
    printf("\n\n\n\n\n");
    exit(-1);
  }
  string infilepath = argv[1];
  for (int i = 2; i < argc; i++)
  {
    switch (argv[i][1])
    {
    case 'k':
      k = atoi(argv[++i]);
      break;
    case 'm':
      m = atof(argv[++i]);
      break;
    case 'w':
      width = atoi(argv[++i]);
      break;
    case 'h':
      height = atoi(argv[++i]);
      break;
    }
  }

  string infilename = infilepath.substr(infilepath.rfind("/") + 1);
  string infileext = infilepath.substr(infilepath.rfind("."));
  string directory = infilepath.substr(0, infilepath.rfind("/") + 1);
  string outfilename = infilename.substr(0, infilename.rfind(".")) + ".SLICSuperpixels_" + to_string(int(round(k))) + "_" + to_string(int(round(m))) + infileext;
  string outfilepath = directory  + outfilename;

  std::cout << "\n" << infilename;
  std::cout << "\n" << infileext;
  std::cout << "\n" << directory;
  std::cout << "\n" << outfilename;
  std::cout << "\n" << outfilepath;


  // unsigned int (32 bits) to hold a pixel in ARGB format as follows:
  // from left to right,
  // the first 8 bits are for the alpha channel (and are ignored)
  // the next 8 bits are for the red channel
  // the next 8 bits are for the green channel
  // the last 8 bits are for the blue channel
  unsigned int* pbuff = new uint[width * height]; // new UINT[sz]
  ReadImage(infilepath, pbuff);//YOUR own function to read an image into the ARGB format

  //----------------------------------
  // Initialize parameters
  //----------------------------------

  int* klabels = new int[width * height];
  int numlabels(0);
  string labelfilename = infilename.substr(0, infilename.rfind(".")) + ".SLICSuperpixels_" + to_string(int(round(k))) + "_" + to_string(int(round(m))) + ".dat";
  string savepath = "/home/d_poh/test/";
  //----------------------------------
  // Perform SLIC on the image buffer
  //----------------------------------
  SLIC segment;
  segment.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(pbuff, width, height, klabels, numlabels, k, m);
  // Alternately one can also use the function PerformSLICO_ForGivenStepSize() for a desired superpixel size
  //----------------------------------
  // Save the labels to a text file
  //----------------------------------

  segment.SaveSuperpixelLabels(klabels, width, height, labelfilename, directory);
  //----------------------------------
  // Draw boundaries around segments
  //----------------------------------
  segment.DrawContoursAroundSegments(pbuff, klabels, width, height, 0xff0000);
  //----------------------------------
  // Save the image with segment boundaries.
  //----------------------------------
  SaveSegmentedImage(outfilepath, pbuff, width, height);//YOUR own function to save an ARGB buffer as an image
  //----------------------------------
  // Clean up
  //----------------------------------

  if (pbuff) delete [] pbuff;
  if (klabels) delete [] klabels;

  return 0;
}

// As a disclaimer, the author of the code or EPFL are not responsible for any damages that may result from using this code or the compiled executable. There are no warraties associated with the code or executables.


