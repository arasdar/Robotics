#include <iostream>
#include <boost/filesystem.hpp>

#include "rrlib/time/time.h"

#include "projects/stereo_traversability_experiments/def/tMLR.h"

int finroc::stereo_traversability_experiments::def::tMLR::create_dir(int argc, const char** argv)
{


  //  time_t rawtime;
  //  time(&rawtime);
  //  cout <<  "asctime(localtime(&rawtime)): " << asctime(localtime(&rawtime)) << endl;
  //  const string dir_name = cv::format("IO_data_%d", asctime(localtime(&rawtime))); //"best match based on min difference";
  //  cout << "dir_name: " << dir_name << endl;


  std::cout << " (Now is: " << rrlib::time::ToIsoString(std::chrono::system_clock::now()) << ") ";

  rrlib::time::tTimestamp now = rrlib::time::Now();
  std::string now_string = rrlib::time::ToIsoString(now);

  const string dir_name = "IO_data_" + now_string; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;


  boost::filesystem::path dir(dir_name);
  if (boost::filesystem::create_directories(dir))
  {
    std::cout << "Success" << "\n";
  }


  return 0;
}
