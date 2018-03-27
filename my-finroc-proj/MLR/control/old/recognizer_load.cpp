/*
 * temp_2.cpp
 *
 *  Created on: Feb 28, 2016
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/MLR/control/mRecognition.h"

using namespace finroc::stereo_traversability_experiments::control;

void mRecognition::recognizer_load()
{
  mlr->read_tPCA(this->dir_path.Get(), *(this->pca));
  // when recognizer is loaded
  this->is_recognizer_loaded = true;
  mlr->LOG_PRINT(pca->mean, "pca->mean",  mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  mlr->LOG_PRINT(pca->S, "pca->S",  mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  mlr->LOG_PRINT(pca->V, "pca->V",  mlr::eLOG_PRINT_STATE::eIS_ACTIVE);
  //mlr->display_PCs(pca, I_img_sample);
  FINROC_LOG_PRINT(DEBUG, "Recognizer is LOADED AND READY to recognize");
  mlr->load_filenames(this->dir_path.Get(), filenames, file_parent_path);
  // all the data filenames & file path in the database folder are loaded as well
  this->are_filenames_loaded = true;
}// void mRecognition::recognizer_load()
