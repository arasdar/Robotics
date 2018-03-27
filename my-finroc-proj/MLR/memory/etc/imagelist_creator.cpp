/*this creates a yaml or xml list of files from the command line args
 */

//#include "opencv2/core/core.hpp"
#include <opencv2/core/core.hpp>
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

using namespace cv;

static void help(const char** av)
{
  cout << "\nThis creates a yaml or xml list of files from the command line args\n"
       "usage:\n./" << av[0] << " imagelist.yaml *.png\n"
       << "Try using different extensions.(e.g. yaml yml xml xml.gz etc...)\n"
       << "This will serialize this list of images or whatever with opencv's FileStorage framework" << endl;
}

#include "projects/stereo_traversability_experiments/def/tMLR.h"
//int main(int ac, char** av)
int finroc::stereo_traversability_experiments::def::tMLR::imagelist_creator(int ac, const char** av)
{
  cv::CommandLineParser parser(ac, av, "{help h||}{@output||}");


  /*
  In file included from sources/cpp/projects/stereo_traversability_experiments/def/memory/imagelist_creator.cpp:5:0:
  /usr/include/opencv2/core/core.hpp: In member function ‘int finroc::stereo_traversability_experiments::def::tMLR::imagelist_creator(int, const char**)’:
  /usr/include/opencv2/core/core.hpp:4740:10: error: ‘bool cv::CommandLineParser::has(const string&)’ is protected
   bool has(const std::string& keys);
      ^
  sources/cpp/projects/stereo_traversability_experiments/def/memory/imagelist_creator.cpp:31:24: error: within this context
   if (parser.has("help"))
  */
  //  if (parser.has("help"))
  //  {
  //    help(av);
  //    return 0;
  //  }
  string outputname = av[1]; //parser.get<string>("@output");

  if (outputname.empty())
  {
    help(av);
    return 1;
  }

  Mat m = imread(outputname); //check if the output is an image - prevent overwrites!
  if (!m.empty())
  {
    std::cerr << "fail! Please specify an output file, don't want to overwrite you images!" << endl;
    help(av);
    return 1;
  }

  FileStorage fs(outputname, FileStorage::WRITE);
  fs << "images" << "[";
  for (int i = 2; i < ac; i++)
  {
    fs << string(av[i]);
  }
  fs << "]";
  return 0;
}
