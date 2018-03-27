

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"


using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::read_mean(/*INPUT*/const char** argv, /*INPUT*/const std::string& new_dir_name, /*OUTPUT*/ cv::Mat& mean)
{

  boost::filesystem::path p(argv[1]);    // p reads clearer than argv[1] in the following code

  // Created directory ALREADY for all learned Eigen Space details
  const string dir_name = p.string() + "/" + /*"learned_PCA"*/ new_dir_name; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;
  boost::filesystem::path dir(dir_name);

  // More clear and simpler one
  if (
    (exists(dir))     // does p actually exist?
    && (is_directory(dir))      // is p a directory?
  )
  {
    cout << dir.string() << " is existing and is a directory. now ready to read the learned PCA ..............................." << endl;
  }// end if directory exist
  else
  {
    cout << dir.string() << " is not existing, Please LEARN the database first!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
  }

  // Then fisrt write mean into its own file
  const string filename_mean = dir_name + "/" + "mean.xml";
  cv::FileStorage fs;
  fs.open(filename_mean, cv::FileStorage::READ);
  fs["mean"] >> mean; //.mean;
  cout << filename_mean << " readdddddddddddddd & loaded" << endl;

  fs.release();
} // end of writing learned ES into memory
