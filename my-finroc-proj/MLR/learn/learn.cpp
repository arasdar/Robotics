#include "projects/stereo_traversability_experiments/mlr/tMLR.h"
#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;
///////////////////////
void tMLR::learn(/*INPUT*/const Mat& I_vec, /*INPUT*/const Mat& I_img_sample, /*INPUT*/const eDATA_TYPE& data_type, /*INPUT*/const char** argv)
{

  // perform PCA using learn_PCA for tested implemented EVD  -- Feature Extraction
  learn_PCA::tPCA pca;
  pca.compute_EVD(I_vec); //I_vec == INPUT DATA_AS_COL
  //  pca.compute_opencv_SVD(I_vec);
  //  pca.compute_opencv_PCA(I_vec);
  //  pca.compute_eignelib_SVD(I_vec);

  // copy the PCA results
  Mat mean = pca.mean.clone(); // isContinuous is one REASON
  LOG_PRINT(mean, "mean: ", eLOG_PRINT_STATE::eIS_ACTIVE);
  if (data_type == eDATA_TYPE::eIMAGE_DATA) // only reshape when input vector is IMAGE type
  {
    mean = mean.reshape(0 /*no change*/, I_img_sample.rows);
  }
  mean = toGrayscale(mean);  // this is converting or casting the dataTypes from DOUBLE to CHAR and scaling it
  namedWindow("mean", WINDOW_NORMAL);
  imshow("mean", mean);

  Mat S = pca.S.clone(); // S by col
  LOG_PRINT(S, "S: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  Mat V = pca.V.clone(); // square matrix if all EigenValues are kept
  LOG_PRINT(V, "V: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  Mat U = pca.U.clone(); // square matrix if all EigenValues are kept
  LOG_PRINT(U, "U: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  // number of PCs (Principle Components) to keep
  double retainedVariance(0); // this is based on percentage
  unsigned int maxComponents(0);
  int delay(0);

  // Display Principle Components  & Feature Selection
  for (/*unsigned*/ int i = 0; i < S.cols; i++)
  {
    // show the singular values
    //  const std::string data_filename = cv::format("data_%d.xml", this->data_index);
    //const std::string data_filename = cv::format("data_%.4d.xml", this->data_index);
    //string msg = format("Singular values DOUBLE precision(CV_64FC1)# %d = %20.f", i + 1, S.at<double>(i)); //2pow^64 ~ 2pow20 ==> 20 digits
    string msg = format("Singular values DOUBLE precision(CV_64FC1)# %.4d = %0.60f", i + 1, S.at<double>(i));
    cout << msg << endl;

    cv::Mat eigenFeature = V.col(i).clone();

    // Reading the singular vectors
    if (data_type == eDATA_TYPE::eIMAGE_DATA)
    {
      eigenFeature = eigenFeature.reshape(0 , I_img_sample.rows); // now it s only a colVector -> needs rows & channels for original shape
    }// if data_type image

    eigenFeature = toGrayscale(eigenFeature);// Gray type (type casting from DOUBLE to CHAR) and scale from 0-255 for IMAGES

    // Displaying it with its order with respect to WRT its eigenvalues
    string name = format("eigenFeature_%d", i);
    namedWindow(name, WINDOW_NORMAL);
    imshow(name, eigenFeature);
    char key = waitKey(delay);

    retainedVariance = double(i + 1) / double(S.cols); // only keep NON-Zero eigenvalues among all of them
    maxComponents = i + 1;
    //    cout << "retainedVariance = double(i+1) / double(eigenvectors.cols): " << retainedVariance << endl;
    //    cout << "maxComponents = i+1: " << maxComponents << endl;

    // MANUAL selection of PC -- at the moment we consider all the singular values and singular vectors
    if (key == 'q' || key == 27)
    {
      break; /* selecting the specific NUMBER of principle components for recognition*/
      //delay = 1; // select ALL
    }
  }// for displaying PCs

  cv::destroyAllWindows();

  // For debugging puposes and make sure it broke the loop
  cout << "final retainedVariance or the final number of kept principle components for re-computing the PCA & perform recognition ..........................." << endl;
  cout << "retainedVariance = double(i+1) / double(eigenvectors.cols): " << retainedVariance << endl;
  cout << "maxComponents = i+1: " << maxComponents << endl;
  cout << "maximum number of eigenvalues: " << S.cols << endl;
  cout << " re-computing the PCA ......................................................................." << endl;

  //   apply the chosen number of PCs and change the S, V & maybe U
  //   pca.recompute(num or variance);
  pca.recompute(maxComponents);

  //  // release some RAM, Cache -- deallocate some memory and heap probably
  //  eigenvectors.release();
  //  eigenvalues.release();
  //  mean.release();
  // Release RAM and Cache -- deallocate some memory & heap prolly
  //delete &pca; // ~PCA -- completely work as deconstructor
  //  /*// operator delete example
  //  #include <iostream>     // std::cout
  //
  //  struct MyClass {
  //  MyClass() {std::cout <<"MyClass constructed\n";}
  //  ~MyClass() {std::cout <<"MyClass destroyed\n";}
  //  };
  //
  //  int main () {
  //  MyClass * pt = new (std::nothrow) MyClass;
  //  delete pt;    // implicitly calls ::operator delete(pt)
  //
  //  return 0;
  //  }
  //  Edit & Run
  //  Output:
  //  MyClass constructed
  //  MyClass destroyed*/

  /* maxComponents – maximum number of components that PCA should retain; by default, ALL the components are retained.
   * retainedVariance – Percentage of variance that PCA should retain.*/
  this->LOG_PRINT(pca.S, "S", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  this->LOG_PRINT(pca.U, "U", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  this->LOG_PRINT(pca.V, "V", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);


  // writing learned PCA
  if (data_type == eDATA_TYPE::eIMAGE_DATA)
  {
    write_tPCA(pca, argv, "learned_PCA_img"); // images
  }
  if (data_type == eDATA_TYPE::eDISTANCE_DATA)
  {
    write_tPCA(pca, argv, "learned_PCA_dist");
  }
  if (data_type == eDATA_TYPE::eLOCALIZATION_DATA)
  {
    write_tPCA(pca, argv, "learned_PCA_loc");
  }
  if (data_type == eDATA_TYPE::eALL_DATA)
  {
    write_tPCA(pca, argv, "learned_PCA_all");
  }

}// end of function
