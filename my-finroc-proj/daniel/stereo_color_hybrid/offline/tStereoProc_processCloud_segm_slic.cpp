
/*SLIC class declaration and implementation files are provided along with a MS VC+6 workspace. The files provide the code to perform superpixel segmentation as explained in the paper:

"SLIC Superpixels Compated to State-of-the-art Superpixel Methods",
Radhakrishna Achanta, Appu Shaji, Kevin Smith, Aurelien Lucchi, Pascal Fua, and Sabine Susstrunk.
IEEE TPAMI, November2012.

The usage is quite straight forward if one wants to incorporate SLICO in his/her projects. One has to instantiate an object of the SLIC class and call the various methods on it. Here is an example main() file:

====================================================================================================
NOTE: For commercial use please contact the author Radhakrishna Achanta (firstname.lastname@epfl.ch)
====================================================================================================
*/



#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"
#include "projects/stereo_traversability_experiments/daniel/SLICSuperpixels/SLIC.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;


unsigned int* ReadImage(string path, int *width, int *height)
{
  Mat image = imread(path, CV_LOAD_IMAGE_COLOR);   // Read the file

  if (! image.data)                              // Check for invalid input
  {
    cout <<  "Could not open or find the image" << std::endl ;
    return NULL;
  }

  *width = image.cols;
  *height = image.rows;
  unsigned int* buffer = new uint[image.cols * image.rows]; // new UINT[sz]
  uchar r, g, b;

  // translate from opencv BGR to SLIC RGB(A)
  for (int i = 0; i < image.rows; i++)
  {
    for (int j = 0; j < image.cols; j++)
    {
      Vec3b pixelValue = image.at<Vec3b>(i, j);
      b = pixelValue.val[0]; // B
      g = pixelValue.val[1]; // G
      r = pixelValue.val[2]; // R
      // store as rgb value in buffer
      unsigned int rgba = (r << 16) | (g << 8) | b;
      buffer[i * image.cols + j] = rgba;
    }
  }
  return buffer;
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

void ConvertBufferToPointCloud(CloudPtr cloud, uint *buffer, int w, int h)
{
  uint r, g, b;

  // translate from SLIC RGB(A) to opencv BGR
  for (int i = 0; i < h; i++)
  {
    for (int j = 0; j < w; j++)
    {
      unsigned int rgba = buffer[i * w + j];
      r = (rgba >> 16) & 0xFF;
      g = (rgba >> 8) & 0xFF;
      b = (rgba) & 0xFF;

      // store as bgr value in Mat20.ppm"
      cloud->points[i * w + j].r = r;
      cloud->points[i * w + j].g = g;
      cloud->points[i * w + j].b = b;
    }
  }
  return;
}

void tStereoProcessing::processCloud_segm_slic(string filepath)
{
  int k = 200;//Desired number of superpixels.
  double m = 30;//Compactness factor. use a value ranging from 10 to 40 depending on your needs. Default is 10

  int width, height;
  string infilepath = filepath;
  string infilename = infilepath.substr(infilepath.rfind("/") + 1);
  string infileext = infilepath.substr(infilepath.rfind("."));
  string directory = infilepath.substr(0, infilepath.rfind("/") + 1);
  //string outfilename = infilename.substr(0, infilename.rfind(".")) + ".SLICSuperpixels_" + to_string(int(round(k))) + "_" + to_string(int(round(m))) + infileext;
  //string outfilepath = directory  + outfilename;


  // unsigned int (32 bits) to hold a pixel in ARGB format as follows:
  // from left to right,
  // the first 8 bits are for the alpha channel (and are ignored)
  // the next 8 bits are for the red channel
  // the next 8 bits are for the green channel
  // the last 8 bits are for the blue channel
  unsigned int* pbuff = ReadImage(infilepath, &width, &height);//YOUR own function to read an image into the ARGB format

  //----------------------------------
  // Initialize parameters
  //----------------------------------

  int* klabels = new int[width * height];
  int numlabels(0);
  //----------------------------------
  // Perform SLIC on the image buffer
  //----------------------------------
  SLIC segment;
  segment.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(pbuff, width, height, klabels, numlabels, k, m);
  // Alternately one can also use the function PerformSLICO_ForGivenStepSize() for a desired superpixel size
  //----------------------------------
  // Save the labels to a text file
  //----------------------------------

  //segment.SaveSuperpixelLabels(klabels, width, height, labelfilename, directory);
  //----------------------------------
  // Draw boundaries around segments
  //----------------------------------
  segment.DrawContoursAroundSegments(pbuff, klabels, width, height, 0xff0000);
  //----------------------------------
  // Save the image with segment boundaries.
  //----------------------------------
  //SaveSegmentedImage(outfilepath, pbuff, width, height);//YOUR own function to save an ARGB buffer as an image
  //----------------------------------
  // transform superpixels to pcl datatypes
  pcl::PointCloud<pcl::Label> segment_labels;
  std::vector<pcl::PointIndices> segments;
  segment_labels.resize(prev_cloud->points.size());
  segment_labels.width = prev_cloud->width; // must be equal to width
  segment_labels.height = prev_cloud->height; // must be equal to height
  segments.resize(numlabels);

  std::cout << "cloudsize: " << prev_cloud->width << "X" << prev_cloud->height << "; image-size: " << width << "X" << height << std::endl;

  for (int idx = 0; idx < width * height; idx++)
  {
    segment_labels[idx].label = klabels[idx];
    segments[klabels[idx]].indices.push_back(idx);
  }

  CloudPtr cloud(new Cloud);
  *cloud = *prev_cloud;
  ConvertBufferToPointCloud(cloud, pbuff, width, height);

  prev_cloud_segm_appear = cloud;
  prev_cloud_segments_appear = segments;
  prev_cloud_segment_labels_appear = segment_labels;

  // Clean up
  //----------------------------------

  if (pbuff) delete [] pbuff;
  if (klabels) delete [] klabels;

  return;
}

// As a disclaimer, the author of the code or EPFL are not responsible for any damages that may result from using this code or the compiled executable. There are no warraties associated with the code or executables.


