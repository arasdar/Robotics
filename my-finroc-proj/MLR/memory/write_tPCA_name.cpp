

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::write_tPCA(/*INPUT*/const learn_PCA::tPCA& pca, /*INPUT*/const char** argv, /*INPUT*/const std::string& new_dir_name)
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
  fs << "mean" << pca.mean;
  cout << filename_mean << " writtennnnnnnnnnnn" << endl;

  const string filename_SV = dir_name + "/" + "SV_matrix_info.xml";
  fs.open(filename_SV, cv::FileStorage::WRITE);
  fs << "V_rows" << pca.V.rows; // this is the same as S.cols // values of vectors
  fs << "V_cols" << pca.V.cols; // Data Sample Dimension
  fs << "V_type" << pca.V.type();
  // S.cols == V.cols && S.rows == 1 & S.type() == V.type()  ??? make sure about this  -- both should be DOUBLE precision or CV64F
  cout << filename_SV << " writtennnnnnnnnnnn" << endl;

  // V matrix
  // Iterating the Principle Components
  for (int i = 0; i < pca.V.cols; i++) // i is either ITERATION or ROWS(HEIGHT, Y) -- in this case ROWS
  {
    // S and V
    const string index = format("_%.4d", i);
    const string filename_PC = dir_name + "/" + "PC" + index + ".xml";
    fs.open(filename_PC, cv::FileStorage::WRITE);
    fs << "V" << pca.V.col(i); // i == row == y == height and also ITERATING THROUGH ROWS & also INDEX == i
    fs << "S" << pca.S.col(i);
    cout << filename_PC << " writtennnnnnnnnnnn" << endl;
  }// for iterating through S

  cout << "learned PCA is written to: " << dir_name << endl;

  fs.release();
} // end of writing learned ES into memory
