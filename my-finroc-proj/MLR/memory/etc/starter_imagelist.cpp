/*
 * starter_imagelist.cpp
 *
 *  Created on: Nov 23, 2010
 *      Author: Ethan Rublee
 *
 * A starter sample for using opencv, load up an imagelist
 * that was generated with imagelist_creator.cpp
 * easy as CV_PI right?
 */
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/core/core.hpp>
//#include "opencv2/highgui/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

//hide the local functions in an unnamed namespace
//namespace
//{}
// not really neccessary when the function is added to my class with a namespace already!!

void help(const char** av)
{
  cout << "\nThis program gets you started being able to read images from a list in a file\n"
       "Usage:\n./" << av[0] << " image_list.yaml\n"
       << "\tThis is a starter sample, to get you up and going in a copy pasta fashion.\n"
       << "\tThe program reads in an list of images from a yaml or xml file and displays\n"
       << "one at a time\n"
       << "\tTry running imagelist_creator to generate a list of images.\n"
       "Using OpenCV version %s\n" << CV_VERSION << "\n" << endl;
}

bool readStringList(const string& filename, vector<string>& l)
{
  l.resize(0);
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;
  FileNode n = fs.getFirstTopLevelNode();
  if (n.type() != FileNode::SEQ)
    return false;
  FileNodeIterator it = n.begin(), it_end = n.end();
  for (; it != it_end; ++it)
    l.push_back((string)*it);
  return true;
}

int process(vector<string> images)
{
  namedWindow("image", CV_WINDOW_KEEPRATIO); //resizable window;
  for (size_t i = 0; i < images.size(); i++)
  {
    Mat image = imread(images[i], IMREAD_GRAYSCALE); // do grayscale processing?
    imshow("image", image);
    cout << "Press a key to see the next image in the list." << endl;
    waitKey(); // wait indefinitely for a key to be pressed
  }
  return 0;
}


#include "projects/stereo_traversability_experiments/def/tMLR.h"

//int main(int ac, char** av)
int finroc::stereo_traversability_experiments::def::tMLR::starter_imagelist(int ac, const char** av)
{
  cv::CommandLineParser parser(ac, av, "{help h||}{@input||}");


  /*In file included from sources/cpp/projects/stereo_traversability_experiments/def/memory/starter_imagelist.cpp:12:0:
  /usr/include/opencv2/core/core.hpp: In member function ‘int finroc::stereo_traversability_experiments::def::tMLR::starter_imagelist(int, const char**)’:
  /usr/include/opencv2/core/core.hpp:4740:10: error: ‘bool cv::CommandLineParser::has(const string&)’ is protected
   bool has(const std::string& keys);
      ^
  sources/cpp/projects/stereo_traversability_experiments/def/memory/starter_imagelist.cpp:72:24: error: within this context
   if (parser.has("help"))
            ^
  */
  //  if (parser.has("help"))
  //  {
  //    help(av);
  //    return 0;
  //  }
  std::string arg = av[1]; //parser.get<std::string>("@input");
  if (arg.empty())
  {
    help(av);
    return 1;
  }
  vector<string> imagelist;

  if (!readStringList(arg, imagelist))
  {
    cerr << "Failed to read image list\n" << endl;
    help(av);
    return 1;
  }

  return process(imagelist);
}
