

#include "projects/stereo_traversability_experiments/def/tMLR.h"


void finroc::stereo_traversability_experiments::def::tMLR::read_PCA(cv::PCA pca_ /*not constant because it is an OUTPUT*/)
{

  // Create a directory for all learned Eigen Space details
  const string dir_name = "learned_PCA"; // write the director at that  specific spot which the program is being executed
  cout << "dir_name: " << dir_name << endl;

  //    boost::filesystem::path dir(dir_name);
  //    if (boost::filesystem::create_directories(dir))
  //    {
  //      std::cout << dir_name << " Successfully created........................." << "\n";
  //    }

  // Then fisrt write mean into its own file
  const string filename_mean = dir_name + "/" + "mean.xml";
  cv::FileStorage fs;
  fs.open(filename_mean, cv::FileStorage::READ);
  fs["mean"] >> pca_.mean;
  //  pca_.mean = fs["mean"];

  cout << filename_mean << " readdddddddddddddd" << endl;

  //Mat eigenvalues (eigenvalues.rows, 1, mean.type);
  // This is used to createMat header for eigenvalues matrix before loaing them
  const string filename_eigenvalues = dir_name + "/" + "eigenvalues_matrix_info.xml";
  fs.open(filename_eigenvalues, cv::FileStorage::READ);
  int eigenvalues_rows;
  fs["eigenvalues_rows"] >> eigenvalues_rows;
  //fs << "eigenvalues.type()" << pca_.eigenvalues.type();
  cout << filename_eigenvalues << " readdddddddddddd" << endl;
  pca_.eigenvalues.create(eigenvalues_rows, 1, pca_.mean.type());
//  Mat eigenvalues(eigenvalues_rows, 1, pca_.mean.type());

  //Mat eigenvalues (eigenvalues.rows, eigenvectors.cols, mean.type);
  // This is used to createMat header for eigenvectors matrix before loading it
  const string filename_eigenvectors = dir_name + "/" + "eigenvectors_matrix_info.xml";
  fs.open(filename_eigenvectors, cv::FileStorage::READ);
  /*size_t == unsigned int == size type*/int eigenvectors_cols;
  fs["eigenvectors_cols"] >> eigenvectors_cols;
  //fs << "eigenvectors.type()" << pca_.eigenvectors.type();
  cout << filename_eigenvectors << " readddddddddddddd" << endl;
  pca_.eigenvectors.create(eigenvalues_rows, eigenvectors_cols, pca_.mean.type());
//  Mat eigenvectors(eigenvalues_rows, eigenvectors_cols, pca_.mean.type());

  // Iterating the Principle Components
  for (int i = 0; i < pca_.eigenvalues.rows; i++) // i is either ITERATION or ROWS(HEIGHT, Y) -- in this case ROWS
  {
    // eigenvalues and eigenvectors
    const string index = format("_%d", i);
    const string filenam_eigenspace = dir_name + "/" + "EigenSpace" + index + ".xml";
    fs.open(filenam_eigenspace, cv::FileStorage::READ);
    Mat eigenvector_row_i;
    fs["eigenvectors"] >> eigenvector_row_i;
    eigenvector_row_i.convertTo(pca_.eigenvectors.row(i), pca_.eigenvectors.type()); // i == row == y == height and also ITERATING THROUGH ROWS & also INDEX == i
    Mat eigenvalue_row_i;
    fs["eigenvalues"] >> eigenvalue_row_i;
    eigenvalue_row_i.convertTo(pca_.eigenvalues.row(i), pca_.eigenvalues.type());
    //pca_.eigenvalues.at(i) = eigenvalues_row_i;
    cout << filenam_eigenspace << " readdddddddddddddddd" << endl;
  }// for iterating through eigenvalues

  cout << "learned PCA is read from: " << dir_name << endl;

  fs.release();
} // end of writing learned ES into memory
