/*
 * tTestStereoMatch_compute.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

using namespace finroc::stereo_traversability_experiments::aras::libstereo;

//////////////////////////////////////////////////////////////////////////////
void
tTestStereoMatching::compute(unsigned char* ref_img, unsigned char* trg_img, int width, int height)
{

//  //Check that a suitable value of max_disp has been selected
//  if (max_disp_ <= 0)
//  {
//    PCL_ERROR(
//      "[pcl::StereoMatching::compute] Error. A positive max_disparity value has not be correctly inserted. Aborting..\n"
//    );
//    return;
//  }

  delete [] disp_map_;
  disp_map_ = NULL;

  delete [] pp_ref_img_;
  delete [] pp_trg_img_;
  pp_ref_img_ = NULL;
  pp_trg_img_ = NULL;

  disp_map_ = new short int[width * height];

  width_ = width;
  height_ = height;

  pp_ref_img_ = new unsigned char[width_ * height_];
  pp_trg_img_ = new unsigned char[width_ * height_];

  ref_img_ = new unsigned char[width_ * height_];
  trg_img_ = new unsigned char[width_ * height_];

  //later on used for getPointCloud --> 3D reconstruction and color triplet
  ref_img_ = ref_img;
  trg_img_ = trg_img; //this one is not used yet!!!!!

  memset(disp_map_, 0, sizeof(short int)*height_ * width_);

  if (is_pre_proc_)
  {
    preProcessing(ref_img, pp_ref_img_);
    preProcessing(trg_img, pp_trg_img_);
    compute_impl(pp_ref_img_, pp_trg_img_); //original
  }
  else
  {
    compute_impl(ref_img, trg_img); //original
  }

  /*at the end, x_offset (*16) needs to be added to all computed disparities,
  so that each fixed point value of the disparity map represents the true disparity value multiplied by 16*/
  for (unsigned j = 0; j < height_; j++)
    for (unsigned i = 0; i < width_; i++)
      if (disp_map_[j * width_ + i] > 0)
        disp_map_[j * width_ + i] += static_cast<short int>(x_off_ * 16);

}






