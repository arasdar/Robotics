#include <iostream> // cout , cin
#include <boost/filesystem.hpp> // pretty obvious
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/highgui/highgui.hpp> // imread and imshow ...

using namespace boost::filesystem;
using namespace std;

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"
using namespace finroc::stereo_traversability_experiments::mlr;

#include "projects/stereo_traversability_experiments/MLR/control/mRecognition.h"
using namespace finroc::stereo_traversability_experiments::control;

const unsigned int /*size_t*/ cNUMBER_OF_COMP(8); // number of comparison measurements

void mRecognition::search(/*INPUT*/const cv::Mat& I_new)
{

  // preparing for optimizing the distance function or comparison function
  /*void mRecognition::optimize_test*/
  std::vector<double> final_comparison_results;
  std::vector<RecordedData> vec_recorded_data;
  std::vector<std::string> vec_XML_filepath;

  // resizing the vectors
  vec_recorded_data.resize(cNUMBER_OF_COMP);
  vec_XML_filepath.resize(cNUMBER_OF_COMP);
  final_comparison_results.clear();

  // start searching and iterating through the DATABASE to find the best MATCH
  for (auto iter = filenames.begin(); iter < filenames.end(); ++iter)
  {
    const string XML_filepath(*iter);  // it has to be CONSTANT in one iteration
    cv::FileStorage fs(XML_filepath, cv::FileStorage::READ);

    // First of all let s check if we can open the file
    if (!fs.isOpened())
    {
      cerr << "Fail to open: " << XML_filepath << endl;
      //return 1;
      exit(1);
    }

    const std::string time_stamp_string_label = fs["time_stamp_string_label"]; // not really used
    const std::string dir_name = fs["dir_name"];
    const /*unsigned*/ int data_index = fs["data_index"];
    const std::string camera_image_filename = fs["camera_image_filename"];
    const std::vector<double> ir_distance = {fs["ir_distance_front_value"],
                                             fs["ir_distance_lll_value"], fs["ir_distance_ll_value"], fs["ir_distance_l_value"],
                                             fs["ir_distance_rear_value"],
                                             fs["ir_distance_r_value"], fs["ir_distance_rr_value"], fs["ir_distance_rrr_value"]
                                            }; // from front -- anti clockwise to rrr
    const std::vector<double> pose = {fs["pose_x"], fs["pose_y"], fs["pose_yaw"]};
    const double desired_velocity = fs["desired_velocity"];
    const double desired_angular_velocity = fs["desired_angular_velocity"];
    const double desired_fork_position = fs["desired_fork_position"];
    fs.release();


    // start loading the recorded camera images in database -- I_img
    const string camera_image_filepath = /*p.parent_path().string()*/ file_parent_path + "/" + camera_image_filename;
    const path camera_image_path(camera_image_filepath);

    //////////////ONLY THE INPUT DATA FOR LEARNING
    // This is the final step for loading the all input data all together -- there should similar size and should be all loaded as they exist and
    // and they have no problem so that all with the same labels (date, index) have values and are not EMPTY
    // loading the image data vector for learning
    if (
      exists(camera_image_path)    // does p actually exist?
      && (is_regular_file(camera_image_path))        // is p a regular file?
      && (!camera_image_path.empty())  // if the XML file is not empty
      && (pose.size() > 0 && !pose.empty())
      && (ir_distance.size() > 0  /*meaning the vector is not empty*/ && !ir_distance.empty())
    )
    {
      /*const - is just READ ONLY!! */cv::Mat camera_image = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_GRAYSCALE);
      if (camera_image.rows > 0 && camera_image.cols > 0  && !camera_image.empty() /*camera image not empty*/)
      {
        /*const */string window_name = "searching database of camera images";
        cv::namedWindow(window_name, WINDOW_NORMAL);
        cv::imshow(window_name, camera_image);
        //cv::waitKey(1);  // THERE is already one EXISTING


        /*! Here is the right spot for doing the actual COMPARISON TO FIND THE BEST MATCH  -- the ACTUAL RECOGNITION */
        // rename it first to match other names ---> I_new_img
        cv::Mat I_img(camera_image);
        //camera_image.release(); ==> has been loaded in recorded data
        //        /*const string */window_name = "I_img";
        //        cv::namedWindow(window_name, WINDOW_NORMAL);
        //        cv::imshow(window_name, I_img);

        // now I_img in database should be scaled first
        cv::Mat I_img_scaled = mlr->toGrayscale(I_img);
        I_img.release();
        //        /*const string */window_name = "I_img_scaled";
        //        cv::namedWindow(window_name, WINDOW_NORMAL);
        //        cv::imshow(window_name, I_img_scaled);

        // vectorize it then
        cv::Mat I = mlr->vectorize_input_data(I_img_scaled); // this is one image at the time from DATABASE
        I_img_scaled.release();
        mlr->LOG_PRINT(I, "I:", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

        if (I.rows == pca->V.rows && I.cols == 1)
        {
          // comparing I and I new --> d(I, I_new) distance function
          std::vector<double> vec_diff_norms, vec_sim_values;
          mlr->compare(I, I_new, *this->pca, this->S_diag_inv, vec_diff_norms, vec_sim_values);
          //          // OUTPUT --> comparison results of diff and sim
          //          vec_diff_norms.resize(4); // 0,1,2,3
          //          vec_sim_values.resize(4); // 0,1,2,3
          //
          //          vec_diff_norms[0] = diff_norm;
          //          vec_diff_norms[1] = diff_mu_norm;
          //          vec_diff_norms[2] = diff_mu_V_norm;
          //          vec_diff_norms[3] = diff_mu_V_S_norm;
          //
          //          vec_sim_values[0] = sim_value;
          //          vec_sim_values[1] = sim_mu_value;
          //          vec_sim_values[2] = sim_mu_V_value;
          //          vec_sim_values[3] = sim_mu_V_S_value;

          std::vector<double> comparison_results;
          comparison_results.resize(cNUMBER_OF_COMP);
          comparison_results[0] = vec_diff_norms[0] /*= diff_norm*/;
          comparison_results[1] = vec_diff_norms[1] /*= diff_mu_norm*/;
          comparison_results[2] = vec_diff_norms[2] /*= diff_mu_V_norm*/;
          comparison_results[3] = vec_diff_norms[3] /*= diff_mu_V_S_norm*/;

          comparison_results[4] = vec_sim_values[0] /*= sim_value*/;
          comparison_results[5] = vec_sim_values[1] /*= sim_mu_value*/;
          comparison_results[6] = vec_sim_values[2] /*= sim_mu_V_value*/;
          comparison_results[7] = vec_sim_values[3] /*= sim_mu_V_S_value*/;

          // optimizing the distance function in the database (minimum difference and maximum similarity)
          // compare with the previous comparison results && load the OUTPUT if better
          if (final_comparison_results.empty() && final_comparison_results.size() == 0) // if it is first time
          {
            // this is for all & initializing
            final_comparison_results = comparison_results;

            for (unsigned int index = 0; index < 8; index++) // vector or array index --> array[index]
            {
              //              vec_recorded_data[index]/*diff_norm*/.camera_image = /*diff_norm*/cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR); // has been already used
              //              cv::Mat img_min_diff = vec_recorded_data[index]/*diff_norm*/.camera_image;
              //              const string window_name = cv::format("img_min_diff_%d", index);
              //              cv::namedWindow(window_name, WINDOW_NORMAL);
              //              cv::imshow(window_name, img_min_diff);

              //final_comparison_results[index] = comparison_results[index]; // /*= diff_norm*/

              //XML_filepath_diff_norm = XML_filepath; // loaded XML_file
              vec_XML_filepath[index] = XML_filepath;

              //diff_norm = data_index; // index stored
              vec_recorded_data[index]/*diff_norm*/.data_index = data_index; //== > this already has been used

              // Load the FINAL recorded output data for CONTROL
              vec_recorded_data[index]/*diff_norm*/.time_stamp_string_label = time_stamp_string_label;
              vec_recorded_data[index]/*diff_norm*/.dir_name = dir_name;
              vec_recorded_data[index]/*diff_norm*/.ir_distance = ir_distance;
              vec_recorded_data[index]/*diff_norm*/.pose = pose;
              vec_recorded_data[index]/*diff_norm*/.desired_angular_velocity = desired_angular_velocity; // DISTANCE
              vec_recorded_data[index]/*diff_norm*/.desired_velocity = desired_velocity; // ANGLE
              vec_recorded_data[index]/*diff_norm*/.desired_fork_position = desired_fork_position;
            }//for index 0-4
          }
          else // if not the first time
          {
            //          comparison_results[index] = vec_diff_norms[index] /*= diff_norm*/;
            //          comparison_results[index] = vec_diff_norms[index] /*= diff_norm*/;
            //          comparison_results[index] = vec_diff_norms[index] /*= diff_norm*/;
            for (unsigned int index = 0; index < 4; index++) // vector or array index --> array[index]
            {
              if (comparison_results[index] < final_comparison_results[index]) /*= diff_norm*/ //minimum difference
              {
                vec_recorded_data[index]/*diff_norm*/.camera_image = /*diff_norm*/cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR); // has been already used
                cv::Mat img_min_diff = vec_recorded_data[index]/*diff_norm*/.camera_image;
                const string window_name = cv::format("img_min_diff_%d", index);
                cv::namedWindow(window_name, WINDOW_NORMAL);
                cv::imshow(window_name, img_min_diff);

                final_comparison_results[index] = comparison_results[index]; // /*= diff_norm*/

                //XML_filepath_diff_norm = XML_filepath; // loaded XML_file
                vec_XML_filepath[index] = XML_filepath;

                //diff_norm = data_index; // index stored
                vec_recorded_data[index]/*diff_norm*/.data_index = data_index; //== > this already has been used

                // Load the FINAL recorded output data for CONTROL
                vec_recorded_data[index]/*diff_norm*/.time_stamp_string_label = time_stamp_string_label;
                vec_recorded_data[index]/*diff_norm*/.dir_name = dir_name;
                vec_recorded_data[index]/*diff_norm*/.ir_distance = ir_distance;
                vec_recorded_data[index]/*diff_norm*/.pose = pose;
                vec_recorded_data[index]/*diff_norm*/.desired_angular_velocity = desired_angular_velocity; // DISTANCE
                vec_recorded_data[index]/*diff_norm*/.desired_velocity = desired_velocity; // ANGLE
                vec_recorded_data[index]/*diff_norm*/.desired_fork_position = desired_fork_position;
              }//if comparison_reults < final_comparison_results
            }//for index 0-4

            //          comparison_results[index] = vec_sim_values[0] /*= sim_value*/;
            //          comparison_results[index] = vec_sim_values[0] /*= sim_value*/;
            //          comparison_results[index] = vec_sim_values[0] /*= sim_value*/;
            for (unsigned int index = 4; index < 8; index++) // vector or array index --> array[index] //
            {
              if (comparison_results[index] > final_comparison_results[index]) /*= sim_value*/ // maximum similarity
              {
                vec_recorded_data[index]/*sim_value*/.camera_image = /*sim_value*/cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR); // has been already used
                cv::Mat img_max_sim = vec_recorded_data[index]/*sim_value*/.camera_image;
                const string window_name = cv::format("img_max_sim_%d", index); //"the best match";
                cv::namedWindow(window_name, WINDOW_NORMAL);
                cv::imshow(window_name, img_max_sim);

                final_comparison_results[index] = comparison_results[index]; // /*= diff_norm*/

                //XML_filepath_sim_value = XML_filepath; // loaded XML_file
                vec_XML_filepath[index] = XML_filepath;

                //sim_value = data_index; // index stored
                vec_recorded_data[index]/*sim_value*/.data_index = data_index; //== > this already has been used

                // Load the FINAL recorded output data for CONTROL
                vec_recorded_data[index]/*sim_value*/.time_stamp_string_label = time_stamp_string_label;
                vec_recorded_data[index]/*sim_value*/.dir_name = dir_name;
                vec_recorded_data[index]/*sim_value*/.ir_distance = ir_distance;
                vec_recorded_data[index]/*sim_value*/.pose = pose;
                vec_recorded_data[index]/*sim_value*/.desired_angular_velocity = desired_angular_velocity; // DISTANCE
                vec_recorded_data[index]/*sim_value*/.desired_velocity = desired_velocity; // ANGLE
                vec_recorded_data[index]/*sim_value*/.desired_fork_position = desired_fork_position;
              }// compare sim
            }// for index 4-8

            // showing the resulting images after optimizing the distance function in each iteration
            cv::waitKey(1);
          }// else if final_comparison_results not empty
        } //if (I.rows == pca->V.rows && I.cols == 1)
      }// if camera image exist, not empty and have size
    }// main if all have size and not empty
  }// for iterating through the loaded filenames in DATABASE

  // After optimizing the distance function
  //(minimum difference (MSSD - Minimum Square-root of Squared Differences) and maximum similarity (MNCC - Maximum Normalized Cross Correlation))
  // we should do some decision making for determining the controller output data for the actual control of the robot
  // LOG-PRINTING the output
  for (unsigned int index = 0; index < 8; index++) // vector or array index --> array[index]
  {
    std::string msg = cv::format("final_comparison_results[%d]", index);
    std::cout << msg << ": " << final_comparison_results[index] << std::endl;
    /*std::string */msg = cv::format("vec_XML_filepath[%d]", index);
    cout << msg << ": " << vec_XML_filepath[index] << endl;
  }// for index

  // Making sense out of the comparison results and generating an output for control
  output(vec_recorded_data);

}// end of the function
