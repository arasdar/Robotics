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


// This is gonna be changed to this one
//void tMLR::search_database(/*INPUT*/const cv::Mat I_new_img_vec_projected, /*INPUT*/const string& dir_path,
//                                    /*INPUT*/cv::PCA& pca, /*OUTPUT*/RecordedData recorded_data);


void mRecognition::recognizer_search_database(/*INPUT*/const cv::Mat& I_new_img_vec_projected, /*OUTPUT*/mlr::RecordedData& O_new_recorded_data) // I_new -> O_new flow
{

//      /*INPUT*/const std::vector<std::string>& loaded_filenames = this->filenames,
//      /*INPUT*/ const std::string& file_parent_path = this->file_parent_path,
//      /*INPUT*//*cv::PCA& pca*/const mlr::learn_PCA::tPCA pca = *this->pca,
//      /*OUTPUT*/ RecordedData recorded_data = *this->recorded_data;

  // preperation for the final output & comparisons
  std::vector<double> prev_comparison_results; // OUTPUT
  prev_comparison_results.clear(); // make sure it is empty BEFORE searching in database
  /*unsigned*/ int min_index_diff(0), min_index_diff_scaled(0), max_index_sim(0), max_index_sim_scaled(0);
  /*unsigned*/ cv::Mat img_min_index_diff, img_min_index_diff_scaled, img_max_index_sim, img_max_index_sim_scaled;
  RecordedData recorded_data_min_index_diff;
  //std::string XML_filepath_min_index_diff;
  std::vector<std::string> vec_XML_filepath;
  vec_XML_filepath.clear();
  const unsigned int /*size_t*/ cNUMBER_OF_MTRICS(4);
  vec_XML_filepath.resize(cNUMBER_OF_MTRICS);

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
        //        std::vector<cv::Mat> vec_I_img_scaled;
        //        vec_I_img_scaled.push_back(I_img_scaled);
        //        cv::Mat I_img_vec = this->vectorize_input_data(vec_I_img_scaled);
        //        I_img_scaled.release();
        //        vec_I_img_scaled.clear();
        cv::Mat I_img_vec = mlr->vectorize_input_data(I_img_scaled); // this is one image at the time from DATABASE
        I_img_scaled.release();
        mlr->LOG_PRINT(I_img_vec, "I_img_vec:", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

        if (I_img_vec.rows == pca->V.rows && I_img_vec.cols == 1)
        {
          cv::Mat I_img_vec_projected = pca->project(I_img_vec); // col, j, x, X, width, W
          mlr->LOG_PRINT(I_img_vec_projected, "I_img_vec_projected:", eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

          //            for(int i = 0; i < I_img_vec_projected.rows; i++)
          //            {
          //                /*Parameters:
          //        i – Index along the dimension 0
          //        j – Index along the dimension 1
          //        k – Index along the dimension 2
          //        pt – Element position specified as Point(j,i) .
          //        idx – Array of Mat::dims indices.*/
          //              const cv::Point point(/*j-col-x,w*/0, /*row,height, y*/i);
          //              if (I_img_vec_projected.at(i) == I_img_vec_projected_test.at(i)){}
          ////                  if (I_img_vec_projected.at(i, 0) == I_img_vec_projected_test.at(i, 0))
          ////              {std::cout << "EQUALLLLLLLLLLLLLLLLLLLLLLLLLLLLL"<< std::endl;}
          //            }

          // passed to the compare && get the output comparison results
          std::vector<double> comparison_results; // OUTPUT
          this->recognizer_compare(I_img_vec_projected, I_new_img_vec_projected,/* pca->S,*/ comparison_results);
          //          std::cout << "comparison_results.size(): " << comparison_results.size() << std::endl;
          //          std::cout << "comparison_results[0]: " << comparison_results[0] << std::endl;
          //          std::cout << "comparison_results[1]: " << comparison_results[1] << std::endl;
          //          std::cout << "comparison_results[2]: " << comparison_results[2] << std::endl;
          //FINROC_LOG_PRINT(DEBUG, "comparison_results", comparison_results);

          // compare with the previous comparison results && load the OUTPUT if better
          if (prev_comparison_results.empty() && prev_comparison_results.size() == 0) // if it is first time
          {
            prev_comparison_results = comparison_results;
          }
          else // if not the first time
          {
            //            comparison_results[0] = diff_norm;
            //            comparison_results[1] =  diff_scaled_norm;
            //            comparison_results[2] = sim_cosine;
            //            comparison_results[3] = sim_scaled_cosine;
            if (/*vec_diff_norm[i]*/ comparison_results[0] < prev_comparison_results[0]/*vec_diff_norm[min_index_diff]*/) // Found a smaller min for difference
            {

              img_min_index_diff = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR);
              const string window_name = "img_min_index_diff"; //"the best match";
              cv::namedWindow(window_name, WINDOW_NORMAL);
              cv::imshow(window_name, /*O_new_recorded_data.*/img_min_index_diff);
              //cv::waitKey(1);  // THERE is already one EXISTING

              //              std::cout << "============================================================================================" << std::endl;
              //              std::cout << "current comparison_results[0]: " << comparison_results[0] << std::endl;
              //              std::cout << "prev_comparison_results[0]: " << prev_comparison_results[0] << std::endl;
              //              cout << "previous min_index_diff: " << min_index_diff << endl;
              //              cout << "current data_index: " << data_index << endl;
              //              std::cout << "============================================================================================" << std::endl;


              min_index_diff = data_index; // index stored
              prev_comparison_results[0] = comparison_results[0]; // the min diff value is also stored

              //XML_filepath_min_index_diff = XML_filepath; // loaded XML_file
              vec_XML_filepath[0] = XML_filepath;
              // Load the FINAL recorded output data for CONTROL
              recorded_data_min_index_diff.time_stamp_string_label = time_stamp_string_label;
              recorded_data_min_index_diff.dir_name = dir_name;
              recorded_data_min_index_diff.data_index = data_index; //== > this already has been used
              recorded_data_min_index_diff.camera_image = img_min_index_diff; // has been already used
              recorded_data_min_index_diff.ir_distance = ir_distance;
              recorded_data_min_index_diff.pose = pose;
              recorded_data_min_index_diff.desired_angular_velocity = desired_angular_velocity; // DISTANCE
              recorded_data_min_index_diff.desired_velocity = desired_velocity; // ANGLE
              recorded_data_min_index_diff.desired_fork_position = desired_fork_position;
              //              cout << "recorded_data_min_index_diff.desired_angular_velocity: " << recorded_data_min_index_diff.desired_angular_velocity << endl;
              //              cout << "recorded_data_min_index_diff.desired_velocity: " << recorded_data_min_index_diff.desired_velocity << endl;
              //              cout << "recorded_data_min_index_diff.desired_fork_position: " << recorded_data_min_index_diff.desired_fork_position << endl;
              //              cout << "desired_angular_velocity: " << desired_angular_velocity << endl;
              //              cout << "desired_velocity: " << desired_velocity << endl;
              //              cout << "desired_fork_position: " << desired_fork_position << endl;

            }
            //            comparison_results[0] = diff_norm;
            //            comparison_results[1] =  diff_scaled_norm;
            //            comparison_results[2] = sim_cosine;
            //            comparison_results[3] = sim_scaled_cosine;
            if (/*vec_diff_scaled_norm[i]*/ comparison_results[1] < prev_comparison_results[1]/*vec_diff_scaled_norm[min_index_diff_scaled]*/) // Found a smaller min for difference
            {
              img_min_index_diff_scaled = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR);
              const string window_name = "img_min_index_diff_scaled"; //"the best match";
              cv::namedWindow(window_name, WINDOW_NORMAL);
              cv::imshow(window_name, /*O_new_recorded_data.*/img_min_index_diff_scaled);
              //cv::waitKey(1);  // THERE is already one EXISTING

              //              std::cout << "============================================================================================" << std::endl;
              //              std::cout << "current comparison_results[1]: " << comparison_results[1] << std::endl;
              //              std::cout << "prev_comparison_results[1]: " << prev_comparison_results[1] << std::endl;
              //              cout << "previous min_index_diff_scaled: " << min_index_diff_scaled << endl;
              //              cout << "current data_index: " << data_index << endl;
              //              std::cout << "============================================================================================" << std::endl;

              min_index_diff_scaled = data_index; // index stored
              prev_comparison_results[1] = comparison_results[1]; // min diff scaled value also stored

              vec_XML_filepath[1] = XML_filepath;
            }
            //            comparison_results[0] = diff_norm;
            //            comparison_results[1] =  diff_scaled_norm;
            //            comparison_results[2] = sim_cosine;
            //            comparison_results[3] = sim_scaled_cosine;
            if (/*vec_sim_norm[i]*/comparison_results[2] > prev_comparison_results[2]/*vec_sim_norm[max_index_sim]*/) // Found a bigger max for similarity
            {
              img_max_index_sim = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR);
              const string window_name = "img_max_index_sim"; //"the best match";
              cv::namedWindow(window_name, WINDOW_NORMAL);
              cv::imshow(window_name, /*O_new_recorded_data.*/img_max_index_sim);
              //cv::waitKey(1);  // THERE is already one EXISTING

              //              std::cout << "============================================================================================" << std::endl;
              //              std::cout << "current comparison_results[2]: " << comparison_results[2] << std::endl;
              //              std::cout << "prev_comparison_results[2]: " << prev_comparison_results[2] << std::endl;
              //              cout << "previous max_index_sim: " << max_index_sim << endl;
              //              cout << "current data_index: " << data_index << endl;
              //              std::cout << "============================================================================================" << std::endl;

              max_index_sim = data_index; // index stored
              prev_comparison_results[2] = comparison_results[2]; // max sim value also stored

              vec_XML_filepath[2] = XML_filepath;
            }
            //            comparison_results[0] = diff_norm;
            //            comparison_results[1] =  diff_scaled_norm;
            //            comparison_results[2] = sim_cosine;
            //            comparison_results[3] = sim_scaled_cosine;
            if (/*vec_sim_norm[i]*/comparison_results[3] > prev_comparison_results[3]) // Found a bigger max for similarity scaled
            {
              img_max_index_sim_scaled = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_COLOR);
              const string window_name = "img_max_index_sim_scaled"; //"the best match";
              cv::namedWindow(window_name, WINDOW_NORMAL);
              cv::imshow(window_name, /*O_new_recorded_data.*/img_max_index_sim_scaled);
              //cv::waitKey(1);  // THERE is already one EXISTING

              //              std::cout << "============================================================================================" << std::endl;
              //              std::cout << "current comparison_results[2]: " << comparison_results[2] << std::endl;
              //              std::cout << "prev_comparison_results[2]: " << prev_comparison_results[2] << std::endl;
              //              cout << "previous max_index_sim: " << max_index_sim << endl;
              //              cout << "current data_index: " << data_index << endl;
              //              std::cout << "============================================================================================" << std::endl;

              max_index_sim_scaled = data_index; // index stored
              prev_comparison_results[3] = comparison_results[3]; // max sim value also stored

              vec_XML_filepath[3] = XML_filepath;
            }
            // One waitKey for all imshows (all FOUR metrics)
            cv::waitKey(1);

          }// previous comparison results
        } //        if (I_img_vec.rows == eigenvectors.cols && I_img_vec.cols == 1)
      }// if camera image exist and not empty and have size
    }// main if all have size and not empty

  }// for iterating through the loaded filenames  --- end of searching and iterating through DATABASE

  cout << "final comparison results ..............................................." << std::endl;
  std::cout << "prev_comparison_results.size()(MUST BE ONLY FOUR METRICS): " << prev_comparison_results.size() << std::endl; // make sure there is only 4 metrics and there is no but
  std::cout << "prev_comparison_results[0](min_index_diff): " << prev_comparison_results[0] << std::endl;
  std::cout << "prev_comparison_results[1](min_index_diff_scaled): " << prev_comparison_results[1] << std::endl;
  std::cout << "prev_comparison_results[2](max_index_sim): " << prev_comparison_results[2] << std::endl;
  std::cout << "prev_comparison_results[3](max_index_sim_scaled): " << prev_comparison_results[3] << std::endl;
  cout << "XML_filepath_min_index_diff (the chosen one): " << /*XML_filepath_min_index_diff*/vec_XML_filepath[0] << endl;
  cout << "XML_filepath_min_index_diff_scaled (the chosen one): " << /*XML_filepath_min_index_diff*/vec_XML_filepath[1] << endl;
  cout << "XML_filepath_max_index_sim (the chosen one): " << /*XML_filepath_min_index_diff*/vec_XML_filepath[2] << endl;
  cout << "XML_filepath_max_index_sim_scaled (the chosen one): " << /*XML_filepath_min_index_diff*/vec_XML_filepath[3] << endl;

  if (
    std::abs(min_index_diff - min_index_diff_scaled) < 5
    ||
    std::abs(min_index_diff - max_index_sim) < 5
    ||
    std::abs(min_index_diff - max_index_sim_scaled) < 5
    ||
    std::abs(min_index_diff_scaled - max_index_sim) < 5
    ||
    std::abs(min_index_diff_scaled - max_index_sim_scaled) < 5
    ||
    std::abs(max_index_sim - max_index_sim_scaled) < 5
  )
  {
    cout << "  TRUEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE" << endl;
    cout << "std::abs(min_index_diff - min_index_diff_scaled) < 5 -->" << std::abs(min_index_diff - min_index_diff_scaled) << endl;
    cout << "std::abs(min_index_diff - max_index_sim) < 5 -->" << std::abs(min_index_diff - max_index_sim) << endl;
    cout << "std::abs(min_index_diff - max_sim_index_scaled) < 5 -->" << std::abs(min_index_diff - max_index_sim_scaled) << endl;
    cout << "std::abs(min_index_diff_scaled - max_sim_index) < 5 -->" << std::abs(min_index_diff_scaled - max_index_sim) << endl;
    cout << "std::abs(min_index_diff_scaled - max_sim_index_scaled) < 5 -->" << std::abs(min_index_diff_scaled - max_index_sim_scaled) << endl;
    cout << "std::abs(max_index_sim - max_sim_index_scaled) < 5 -->" << std::abs(max_index_sim - max_index_sim_scaled) << endl;

    // make sure recorded data is not empty -- TESTING THE OUTPUT BEFORE PASSING IT
    O_new_recorded_data = recorded_data_min_index_diff;
    std::cout << "THIS IS THE MAIN CONTROLLING OUTPUTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT" << std::endl;
    cout << "O_new_recorded_data.time_stamp_string_label: " << O_new_recorded_data.time_stamp_string_label << endl;
    cout << "O_new_recorded_data.dir_name: " << O_new_recorded_data.dir_name << endl;
    cout << "O_new_recorded_data.data_index: " << O_new_recorded_data.data_index << endl;
    cout << "O_new_recorded_data.ir_distance.size(): " << O_new_recorded_data.ir_distance.size() << endl;
    cout << "O_new_recorded_data.pose.size(): " << O_new_recorded_data.pose.size() << endl;
    //    cout << "O_new_recorded_data.desired_angular_velocity: " << O_new_recorded_data.desired_angular_velocity << endl;
    //    cout << "O_new_recorded_data.desired_velocity: " << O_new_recorded_data.desired_velocity << endl;
    //    cout << "O_new_recorded_data.desired_fork_position: " << O_new_recorded_data.desired_fork_position << endl;
  }// abs(min_index_diff - max_index_sim) < 2
  else // if the best match was NOT found
  {
    // OUTPUT to controllers or controlling data or controlling commands should be ALL ZERO or set to SERO
    O_new_recorded_data.desired_velocity = 0.;
    O_new_recorded_data.desired_angular_velocity = 0.;
    O_new_recorded_data.desired_fork_position = 0.;
  }
  cout << "O_new_recorded_data.desired_velocity: " << O_new_recorded_data.desired_velocity << endl;
  cout << "O_new_recorded_data.desired_angular_velocity: " << O_new_recorded_data.desired_angular_velocity << endl;
  cout << "O_new_recorded_data.desired_fork_position: " << O_new_recorded_data.desired_fork_position << endl;


}// end of the function
