//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/stereo_traversability_experiments/mlr/tMLR.h
 *
 * \author  Aras Dargazany
 *
 * \date    2016-02-19
 *
 * \brief   Contains tMLR
 *
 * \b tMLR
 *
 * This is the class for memory (read or write), learning and recognition.
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__stereo_traversability_experiments__mlr__tMLR_h__
#define __projects__stereo_traversability_experiments__mlr__tMLR_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <sstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include "projects/stereo_traversability_experiments/mlr/learn_PCA/tPCA.hpp"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace mlr
{

using namespace cv;
using namespace std;
using namespace boost::filesystem;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

enum class eLOG_PRINT_STATE
{
  eIS_ACTIVE, eIS_NOT_ACTIVE
};

enum class eDATA_TYPE
{
  eIMAGE_DATA, eDISTANCE_DATA, eLOCALIZATION_DATA, eALL_DATA
};

struct recursive_directory_range
{
  typedef recursive_directory_iterator iterator;
  recursive_directory_range(path p) : p_(p) {}

  iterator begin()
  {
    return recursive_directory_iterator(p_);
  }
  iterator end()
  {
    return recursive_directory_iterator();
  }
  path p_;
};

struct RecordedData
{
  std::string time_stamp_string_label; /*= fs["time_stamp_string_label"];*/
  std::string dir_name;
  /*unsigned*/ int data_index;/*= fs["data_index"];*/
  //  string camera_image_filename; //= fs["camera_image_filename"];
  //  string camera_image_filepath; //= p.parent_path().string() + "/" + camera_image_filename;
  cv::Mat camera_image;
  std::vector<double> ir_distance; /*= {fs["ir_distance_front_value "],
                                             fs["ir_distance_lll_value"], fs["ir_distance_ll_value"], fs["ir_distance_l_value"],
                                             fs["ir_distance_rear_value"],
                                             fs["ir_distance_r_value"], fs["ir_distance_rr_value"], fs["ir_distance_rrr_value"]
                                            };*/ // from front -- anti clockwise to rrr
  std::vector<double> pose; /*= {fs["pose_x"], fs["pose_y"], fs["pose_yaw"]};*/
  double desired_velocity; // = fs["desired_velocity"];
  double desired_angular_velocity; //= fs["desired_angular_velocity"];
  double desired_fork_position; /*= fs["desired_fork_position"];*/
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This is the class for memory (read or write), learning and recognition.
 */
class tMLR
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tMLR() {} /*int argc, const char** argv*/ // read in the input arguments

  ~tMLR() {}  //You need this destructor if you allocated memory on the heap that must be free'd. Delete otherwise!

  //Here is the right place for your public methods. Replace this line by your declarations!

  /*! Reading and writing IO data into memory and from memory */
  /*! Reading and writing IO data into memory and from memory */
  /*! Reading and writing IO data into memory and from memory */
  /*! Reading and writing IO data into memory and from memory */
  /*! Read RAW input data for learning from the memory */
  void read_directory_recursively(/*INPUT*/int argc, /*INPUT*/const char** argv, /*OUTPUT*/std::vector<cv::Mat>& all_image_data_in_directory);
  std::vector<std::vector<cv::Mat>> read_directory_recursively(/*INPUT*/int argc, /*INPUT*/const char** argv);
  void write_tPCA(/*INPUT*/const learn_PCA::tPCA& pca, /*INPUT*/const char** argv, /*INPUT*/const std::string& new_dir_name);
  void write_tPCA(/*INPUT*/const learn_PCA::tPCA& pca, /*INPUT*/const char** argv);
  void read_tPCA(/*INPUT*/const char** argv, /*OUTPUT*/mlr::learn_PCA::tPCA& pca);
  void read_tPCA(/*INPUT*/const char** argv, /*OUTPUT*/mlr::learn_PCA::tPCA& pca, /*INPUT*/const std::string& new_dir_name);
  void read_tPCA(/*INPUT*/const string& dir_path, /*OUTPUT*/mlr::learn_PCA::tPCA& pca);
  void load_filenames(/*INPUT*/const std::string& dir_path, /*OUTPUT*/std::vector<std::string>& filenames, /*OUTPUT*/ std::string& file_parent_path);

  /*! Pre-processing the loaded IO data */
  /*! Pre-processing the loaded IO data */
  /*! Pre-processing the loaded IO data */
  /*! Pre-processing the loaded IO data */
  /*! Scaled the RAW input Data */
  // Normalizes a given image into a value range between 0 and 255.
  /*static*/ Mat toGrayscale(/*INPUT*/const Mat& input_data);
  void toNormalize(/*INPUT*/const cv::Mat& src, /*OUTPUT*/cv::Mat& dst, /*INPUT*/const eDATA_TYPE& data_type);
  void toNormalize(/*INPUT*/const vector<Mat>& src, /*OUTPUT*/vector<Mat>& dst, /*INPUT*/const eDATA_TYPE& data_type);
  /*static*/ Mat vectorize_input_data(/*INPUT*/const vector<Mat>& input_data_scaled);
  /*static*/ Mat vectorize_input_data(/*INPUT*/const cv::Mat& input_data_scaled);

  /*! Utilities and helpers for debugging, showing, printing and visualizing */
  /*! Utilities and helpers for debugging, showing, printing and visualizing */
  /*! Utilities and helpers for debugging, showing, printing and visualizing */
  /*! These are some utilities and functions for debugging and saving the files to memory */
  void LOG_PRINT(/*INPUT*/const Mat& mat, /*INPUT*/const std::string& name, /*INPUT*/const eLOG_PRINT_STATE& state);
  void display_PCs(/*INPUT*/ const learn_PCA::tPCA& pca, /*INPUT*/const Mat& I_scaled_sample);


  /*! The Learn and recognition part is here */
  /*! The Learn and recognition part is here */
  /*! The Learn and recognition part is here */
  void learn(/*INPUT*/ const Mat& I_vec, /*INPUT*/const Mat& I_img_sample, /*INPUT*/const eDATA_TYPE& data_type, /*INPUT*/const char** argv);
  void learn(/*INPUT*/const Mat& I_vec, /*INPUT*/const eDATA_TYPE& data_type, /*INPUT*/const char** argv);


  /*! Recognition part for generating output */
  /*! Recognition part for generating output */
  /*! Recognition part for generating output */
  void recognize(/*INPUT*/const Mat& I_scaled_vec, /*INPUT*/const Mat& /*images*/I_new_scaled_vec, /*INPUT*/const Mat& I_scaled_sample, /*INPUT*/const char** argv);
  void compare(/*INPUT*/const Mat& I, /*INPUT*/const Mat& I_new, /*INPUT*/const learn_PCA::tPCA& pca, /*INPUT*/const cv::Mat& S_diag_inv,
                        /*OUTPUT*/std::vector<double>& vec_diff_norms/*comparison_results*/, /*OUTPUT*/std::vector<double>& vec_sim_values/*comparison_results*/);

  /* Fusing all the sensor data */
  void fuse_data(/*INPUT*/const char** argv, /*OUTPUT*/cv::Mat& I_all);
  std::vector<cv::Mat> read_data(/*INPUT*/const char** argv, /*INPUT*/ const eDATA_TYPE& data_type);
  std::vector<std::vector<cv::Mat>> read_data(/*INPUT*/const char** argv);
  void compute_mean(/*INPUT*/const cv::Mat& I, /*OUTPUT*/cv::Mat& mean);
  void write_mean(/*INPUT*/const cv::Mat& mean, /*INPUT*/const char** argv, /*INPUT*/const std::string& new_dir_name);
  void read_mean(/*INPUT*/const char** argv, /*INPUT*/const std::string& new_dir_name, /*OUTPUT*/ cv::Mat& mean);
  void recognize_test(/*INPUT*/const Mat& I_all, /*INPUT*/const Mat& /*images*/I_all_img, /*INPUT*/const Mat& I_img_sample, /*INPUT*/const char** argv, /*INPUT*/const eDATA_TYPE& data_type);
  void recognize_test(/*INPUT*/const Mat& I_all, /*INPUT*/const Mat& /*images*/I_all_img, /*INPUT*/const Mat& I_img_sample, /*INPUT*/const char** argv);
  void compare(/*INPUT*/const Mat& I, /*INPUT*/const Mat& I_new, /*INPUT*/const cv::Mat& mean,
                        /*OUTPUT*/std::vector<double>& vec_diff_norms/*comparison_results*/, /*OUTPUT*/std::vector<double>& vec_sim_values/*comparison_results*/);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}


#endif
