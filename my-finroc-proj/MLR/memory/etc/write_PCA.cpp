

#include "projects/stereo_traversability_experiments/def/tMLR.h"


void finroc::stereo_traversability_experiments::def::tMLR::write_PCA(const cv::PCA pca_ /*constant because it is an INPUT*/)
{

  // Create a directory for all learned Eigen Space details
  const string dir_name = "learned_PCA"; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;

  boost::filesystem::path dir(dir_name);
  if (boost::filesystem::create_directories(dir))
  {
    std::cout << dir_name << " Successfully created........................." << "\n";
  }

  // Then fisrt write mean into its own file
  const string filename_mean = dir_name + "/" + "mean.xml";
  cv::FileStorage fs;
  fs.open(filename_mean, cv::FileStorage::WRITE);
//  fs << "mean.size" << pca_.mean.size; // they all have the same type
//  fs << "mean.type()" << pca_.mean.type();
  fs << "mean" << pca_.mean;
  cout << filename_mean << " writtennnnnnnnnnnn" << endl;

  //Mat eigenvalues (eigenvalues.rows, 1, mean.type);
  // This is used to createMat header for eigenvalues matrix before loaing them
  const string filename_eigenvalues = dir_name + "/" + "eigenvalues_matrix_info.xml";
  fs.open(filename_eigenvalues, cv::FileStorage::WRITE);
  fs << "eigenvalues_rows" << pca_.eigenvalues.rows;
  //fs << "eigenvalues.type()" << pca_.eigenvalues.type();
  cout << filename_eigenvalues << " writtennnnnnnnnnnn" << endl;

  //Mat eigenvalues (eigenvalues.rows, eigenvectors.cols, mean.type);
  // This is used to createMat header for eigenvectors matrix before loading it
  const string filename_eigenvectors = dir_name + "/" + "eigenvectors_matrix_info.xml";
  fs.open(filename_eigenvectors, cv::FileStorage::WRITE);
  fs << "eigenvectors_cols" << pca_.eigenvectors.cols;
  //fs << "eigenvectors.type()" << pca_.eigenvectors.type();
  cout << filename_eigenvectors << " writtennnnnnnnnnnn" << endl;

  // eigenvectors matrix

  // Iterating the Principle Components
  for (int i = 0; i < pca_.eigenvalues.rows; i++) // i is either ITERATION or ROWS(HEIGHT, Y) -- in this case ROWS
  {
    // eigenvalues and eigenvectors
    const string index = format("_%d", i);
    const string filenam_eigenspace = dir_name + "/" + "EigenSpace" + index + ".xml";
    fs.open(filenam_eigenspace, cv::FileStorage::WRITE);
    fs << "eigenvectors" << pca_.eigenvectors.row(i); // i == row == y == height and also ITERATING THROUGH ROWS & also INDEX == i
    fs << "eigenvalues" << pca_.eigenvalues.row(i);
    cout << filenam_eigenspace << " writtennnnnnnnnnnn" << endl;
  }// for iterating through eigenvalues

  cout << "learned PCA is written to: " << dir_name << endl;

  fs.release();
} // end of writing learned ES into memory
