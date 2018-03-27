/*
 * temp_2.cpp
 *
 *  Created on: Feb 28, 2016
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/MLR/control/mRecognition.h"

using namespace finroc::stereo_traversability_experiments::control;

void mRecognition::load()
{
  mlr->read_tPCA(this->dir_path.Get(), *(this->pca));
  // when recognizer is loaded
  this->is_recognizer_loaded = true;
  mlr->LOG_PRINT(pca->mean, "pca->mean",  mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  mlr->LOG_PRINT(pca->S, "pca->S",  mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  mlr->LOG_PRINT(pca->V, "pca->V",  mlr::eLOG_PRINT_STATE::eIS_ACTIVE);

  //also calculating the S_diag_inv
  // instantiate, create and initialize with zero
  this->S_diag_inv = cv::Mat::zeros(pca->S.cols /*rows*/, pca->S.cols /*columns*/, pca->S.type());
  for (int j = 0; j < S_diag_inv.cols; j++)
  {
    S_diag_inv.at<double>(j, j) = double(1.0) / pca->S.at<double>(j); // in order to preserve the double precision while inversing
  }
  mlr->LOG_PRINT(S_diag_inv, "S_diag_inv", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_ACTIVE);

  // all the data filenames & file path in the database folder are loaded as well
  mlr->load_filenames(this->dir_path.Get(), filenames, file_parent_path);
  this->are_filenames_loaded = true;
}// void mRecognition::recognizer_load()
