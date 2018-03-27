/*
 * display_PCs.cpp
 *
 *  Created on: Mar 29, 2016
 *      Author: aras
 */

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"


using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

// the implementation is there but implt for what? for tMLR
void tMLR::display_PCs(/*INPUT*/ const learn_PCA::tPCA& pca, /*INPUT*/const Mat& I_scaled_sample) //there is no output since it is only displaying PCs -- debugging purposes
{
  // make sure mean and S, V are loaded correctly  - by showing them
  // copy the PCA results
  Mat mean = pca.mean.clone(); // isContinuous is one REASON
  LOG_PRINT(mean, "mean: ", eLOG_PRINT_STATE::eIS_ACTIVE);
  mean = mean.reshape(0 /*no change in number of CHANNELS*/, I_scaled_sample.rows);
  mean = toGrayscale(mean);
  namedWindow("mean", WINDOW_NORMAL);
  imshow("mean", mean);

  Mat S = pca.S.clone(); // S by col
  LOG_PRINT(S, "S: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  Mat V = pca.V.clone(); // square matrix if all EigenValues are kept
  LOG_PRINT(V, "V: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  Mat U = pca.U.clone(); // square matrix if all EigenValues are kept
  LOG_PRINT(U, "U: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  // Display Principle Components  & Feature Selection
  for (/*unsigned*/ int i = 0; i < S.cols; i++)
  {
    // show the singular values
    //string msg = format("Singular values DOUBLE precision(CV_64FC1)# %d = %20.f", i + 1, S.at<double>(i)); //2pow^64 ~ 2pow20 ==> 20 digits
    string msg = format("Singular values DOUBLE precision(CV_64FC1)# %d = %0.60f", i + 1, S.at<double>(i));
    cout << msg << endl;

    // Reading the singular vectors
    cv::Mat eigenFeature = V.col(i).clone();
    eigenFeature = eigenFeature.reshape(0 /*no change in number of CHANNELS*/, I_scaled_sample.rows);
    eigenFeature = toGrayscale(eigenFeature);

    // Displaying it with its order with respect to WRT its eigenvalues
    string name = format("eigenFeature_%d", i);
    namedWindow(name, WINDOW_NORMAL);
    imshow(name, eigenFeature);
    char key = waitKey(0);

    // MANUAL selection of PC  --> feature selection
    if (key == 'q' || key == 27)
    {
      break;
    }

  }// for displaying PCs
}// display_PCs (Principle Components) --> composition, decomposition, component, compartment and analysis
