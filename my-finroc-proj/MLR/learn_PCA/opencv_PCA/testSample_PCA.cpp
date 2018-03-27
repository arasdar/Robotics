/*
 * test_PCA.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: aras
 */


/*

As far as I know, there is no generic way of saving PCA objects to a file.
You will need to save eigenvectors, eigenvalues and mean to a file, and then put them into a new PCA after loading.
You have to remember to use a format that doesn't lose precision, especially for mean.

Here is some example code:
*/

#include <opencv2/core/core.hpp>
#include <iostream>

#include "projects/stereo_traversability_experiments/def/sampling_csv.h"

//void save(const std::string &file_name, cv::PCA pca_)
//{
//  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
//  fs << "mean" << pca_.mean;
//  fs << "e_vectors" << pca_.eigenvectors;
//  fs << "e_values" << pca_.eigenvalues;
//  fs.release();
//}
//
//void load(const std::string &file_name, cv::PCA pca_)
//{
//  cv::FileStorage fs(file_name, cv::FileStorage::READ);
//  fs["mean"] >> pca_.mean ;
//  fs["e_vectors"] >> pca_.eigenvectors ;
//  fs["e_values"] >> pca_.eigenvalues ;
//  fs.release();
//}

//void learn(const std::vector<cv::Mat>& I /*input_data*/);
//void recongize(const std::vector<Mat>& I_new /*new input data*/);

int main(int argc /*argument counter*/, const char** argv /*argument variable*/)
{

  cv::PCA pca1;
  cv::PCA pca2;

  std::vector<cv::Mat> images = sampling_csv(argc, argv);
  cv::Mat eigenval, eigenvec, mean;
  cv::Mat inputData(images[0]);
  cv::Mat outputData1, outputData2;

  //input data has been populated with data to be used
  pca1(inputData, cv::Mat()/*dont have previously computed mean*/,
       CV_PCA_DATA_AS_ROW /*depends of your data layout*/);//pca is computed
  pca1.project(inputData, outputData1);

  //here is how to extract matrices from pca
  mean = pca1.mean.clone();
  eigenval = pca1.eigenvalues.clone();
  eigenvec = pca1.eigenvectors.clone();

  //here You can save mean,eigenval and eigenvec matrices
  const string filename("def/learned_PCA_model.xml"); // it is initialized using () but = is assignment operator
  save(filename, pca1); // save the learned model to memory or memorizing it

//  load(filename, pca2);
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  fs["mean"] >> /*reading operator*/ pca2.mean;
  fs["e_vectors"] >> pca2.eigenvectors;
  fs["e_values"] >> pca2.eigenvalues;
  fs.release();


  cout << "pca2.mean.size(): " << pca2.mean.size() << endl;

  pca2.project(inputData, outputData2);

  cv::Mat diff;//here some proof that it works
  cv::absdiff(outputData1, outputData2, diff);

  std::cerr << sum(diff)[0] << std::endl; //assuming You are using one channel data, there
  //is data only in first cell of the returned scalar
  return 0;
}

