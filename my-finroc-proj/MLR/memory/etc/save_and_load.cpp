

#include "projects/stereo_traversability_experiments/def/tMLR.h"


void finroc::stereo_traversability_experiments::def::tMLR::save(const std::string &file_name, cv::PCA pca_)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs << "mean" << pca_.mean;
  fs << "eigenvectors" << pca_.eigenvectors;
  fs << "eigenvalues" << pca_.eigenvalues;
  fs.release();
}

//void load(const std::string &file_name, cv::PCA pca_)  // this one looks like having some issues
//{
//  cv::FileStorage fs(file_name, cv::FileStorage::READ);
//  fs["mean"] >> pca_.mean ;
//  fs["eigenvectors"] >> pca_.eigenvectors ;
//  fs["eigenvalues"] >> pca_.eigenvalues ;
//  fs.release();
//}
