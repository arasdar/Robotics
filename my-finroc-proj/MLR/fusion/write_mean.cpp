

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::write_mean(/*INPUT*/const cv::Mat& mean, /*INPUT*/const char** argv, /*INPUT*/const std::string& new_dir_name)
{

  boost::filesystem::path p(argv[1]);    // p reads clearer than argv[1] in the following code
  const string dir_name = p.string() + "/" + /*"learned_PCA"*/ new_dir_name; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;

  boost::filesystem::path dir(dir_name);

  // if it already exist - remove it first with all its content
  if ((exists(dir))     // does p actually exist?
      || (is_regular_file(dir))        // is p a regular file?
      || (is_directory(dir))      // is p a directory?
     )
  {
    boost::filesystem::remove_all(dir);
    cout << dir.string() << " was already existing and now it is successfully and completely removed..............................." << endl;
  }// end if directory exist

  // Create it again but this time EMPTY
  if (boost::filesystem::create_directories(dir))
  {
    std::cout << dir_name << " is new directory and is now successfully created........................." << "\n";
  }

  // Now fill it up with the learned PCA files
  const string filename_mean = dir_name + "/" + "mean.xml";
  cv::FileStorage fs;
  fs.open(filename_mean, cv::FileStorage::WRITE);
  fs << "mean" << mean;
  cout << filename_mean << " writtennnnnnnnnnnn" << endl;
  fs.release();
} // end of writing learned ES into memory
