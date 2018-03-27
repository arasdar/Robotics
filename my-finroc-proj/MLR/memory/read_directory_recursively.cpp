/*
 * path_info.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: aras
 */



#include <iostream> // cout , cin
#include <boost/filesystem.hpp> // pretty obvious
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/highgui/highgui.hpp> // imread and imshow ...
//#include "projects/stereo_traversability_experiments/control/tRecordedData.h"

using namespace boost::filesystem;
using namespace std;

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"
using namespace finroc::stereo_traversability_experiments::mlr;

// This is the time template is needed why it is not really obvious what will be the output of the function
//std::vector<cv::Mat> finroc::stereo_traversability_experiments::def::tMLR::read_directory_recursively(int argc, const char** argv)
std::vector<std::vector<cv::Mat>> tMLR::read_directory_recursively(/*INPUT*/int argc, /*INPUT*/const char** argv)
{

  // Get list of input_data in a directory -- mass reading and iterating through a directory
  //std::vector<std::string> all_XML_data_path;  // if I wanna go through them and sort them BUT why??
  //std::vector< /*a data structure containing all the data types*/> all_RAW_loaded_data;

  // at the meantime let s start with testing the images for LEARNING and then we will go on on with the REST  -- all used for LEARNING
  std::vector<cv::Mat> all_image_data_in_directory;
  std::vector<cv::Mat> all_distance_data_in_directory;
  std::vector<cv::Mat> all_localization_data_in_directory;
  std::vector<std::vector<cv::Mat>> all_input_data_in_directory;  // for learning we need all the RAW input data all AT ONCE

  // This is for RECOGNITION -- it is only needed to iterate through all the data once - ONE BY ONE
  //std::vector<finroc::stereo_traversability_experiments::control::tRecordedData> all_recorded_data_in_directory;

  //How do I iterate over every file, directory, and sub-directory and store the file names in a vector in C++?
  //boost filesystem example for iterate directory and sub-directories
  std::string dir_path(argv[1]);

  for (auto it : recursive_directory_range(dir_path))
  {
    //    std::cout << it << std::endl;

    path p(it);    // p reads clearer than argv[1] in the following code

    if (exists(p))    // does p actually exist?
    {
      if (is_regular_file(p))        // is p a regular file?
      {
        //cout << p << " size is " << file_size(p) << '\n';

        if (p.has_extension() && p.has_stem() && p.has_filename())
        {
          //cout  <<  "  has_filename()-------: " <<   (p.has_filename()) << '\n';
          //cout  <<  "  has_stem()-----------: " <<   (p.has_stem()) << '\n';
          //cout  <<  "  has_extension()------: " <<   (p.has_extension()) << '\n';

          if (p.extension() == ".xml")
          {
            cout  <<  "  relative_path()------: " << p.relative_path() << '\n';
            cout  <<  "  parent_path()--------: " << p.parent_path() << '\n';
            cout  <<  "  filename()-----------: " << p.filename() << '\n';
            cout  <<  "  stem()---------------: " << p.stem() << '\n';
            cout  <<  "  extension()----------: " << p.extension() << '\n';

            cout  <<  "  has_relative_path()--: " << (p.has_relative_path()) << '\n';
            cout  <<  "  has_parent_path()----: " << (p.has_parent_path()) << '\n';
            cout  <<  "  has_filename()-------: " << (p.has_filename()) << '\n';
            cout  <<  "  has_stem()-----------: " << (p.has_stem()) << '\n';
            cout  <<  "  has_extension()------: " << (p.has_extension()) << '\n';


            if (!p.empty())  // if the XML file is not empty
            {

              //cout  <<  "  empty()--------------: " <<   (p.empty()) << '\n';
              //cout << "path.string(): " << p.string() << endl;

              // push the xml file path to one vector
              //all_XML_data_path.push_back(p.string());
              //cout << "vector size: " << all_XML_data_path.size() << endl;

              // now let s open the XML file and look into it
              //read
              const string XML_filepath(p.string());  // it has to be CONSTANT in one iteration
              cout << "XML_filepath: " << XML_filepath << endl;

              // The recorded data structure to pass allt he recorded data at once
              //finroc::stereo_traversability_experiments::control::tRecordedData recorded_data;

              cv::FileStorage fs(XML_filepath, cv::FileStorage::READ);
              cout << endl << "Opened for Reading: " << endl;

              // First of all let s check if we can open the file
              if (!fs.isOpened())
              {
                cerr << "Fail to open: " << XML_filepath << endl;
                //return 1;
                exit(1);
              }

              //              // first method: use (type) operator on FileNode.
              //              int frameCount = (int)fs2["frameCount"];
              //
              //              std::string date;
              //              // second method: use FileNode::operator >>
              //              fs2["calibrationDate"] >> date;
              //
              //              Mat cameraMatrix2, distCoeffs2;
              //              fs2["cameraMatrix"] >> cameraMatrix2;
              //              fs2["distCoeffs"] >> distCoeffs2;
              //
              //              cout << "frameCount: " << frameCount << endl
              //                   << "calibration date: " << date << endl
              //                   << "camera matrix: " << cameraMatrix2 << endl
              //                   << "distortion coeffs: " << distCoeffs2 << endl;
              //
              //              FileNode features = fs2["features"];
              //              FileNodeIterator it = features.begin(), it_end = features.end();
              //              int idx = 0;
              //              std::vector<uchar> lbpval;


              //fs << "time_stamp_string_label" << this->time_stamp_string_label; // this is dir label - date
              const string time_stamp_string_label = fs["time_stamp_string_label"];
              cout << "time_stamp_string_label: " << time_stamp_string_label << endl;
              //recorded_data.time_stamp_string_label = time_stamp_string_label;

              //              fs << "data_index" << this->data_index; // this is file label inside dir - index
              const /*unsigned*/ int data_index = fs["data_index"];
              cout << "data_index: " << data_index << endl;
              //recorded_data.data_index = data_index;

              //              fs << "camera_image_filename" << camera_image_filename;
              const string camera_image_filename = fs["camera_image_filename"];
              cout << "camera_image_filename: " << camera_image_filename << endl;
              // camera image filename is not needed since we read the actual image

              // let s show the image
              const string camera_image_filepath = p.parent_path().string() + "/" + camera_image_filename;
              const path camera_image_path(camera_image_filepath);
              cout << "camera_image_filepath: " << camera_image_filepath << endl;



              //ir_distance.push_back(fs["ir_distance_front_value"]);
              //              fs << "ir_distance_front_value" << this->ir_distance_front.Get().Value();
              //              fs << "ir_distance_l_value" << this->ir_distance_l.Get().Value();
              //              fs << "ir_distance_ll_value" << this->ir_distance_ll.Get().Value();
              //              fs << "ir_distance_lll_value" << this->ir_distance_lll.Get().Value();
              //              fs << "ir_distance_rear_value" << this->ir_distance_rear.Get().Value();
              //              fs << "ir_distance_rrr_value" << this->ir_distance_rrr.Get().Value();
              //              fs << "ir_distance_rr_value" << this->ir_distance_rr.Get().Value();
              //              fs << "ir_distance_r_value" << this->ir_distance_r.Get().Value();
              const std::vector<double> ir_distance = {fs["ir_distance_front_value	"],
                                                       fs["ir_distance_lll_value"], fs["ir_distance_ll_value"], fs["ir_distance_l_value"],
                                                       fs["ir_distance_rear_value"],
                                                       fs["ir_distance_r_value"], fs["ir_distance_rr_value"], fs["ir_distance_rrr_value"]
                                                      }; // from front -- anti clockwise to rrr
              cout << "ir_distance.size(): " << ir_distance.size() << endl;

              for (std::vector<double>::const_iterator iter = ir_distance.begin(); iter < ir_distance.end(); ++iter)
              {
                cout << "*iter(ir_distance in order from front to ....): " << *iter << endl;
              }

              //              //fs << "localization data" ;
              //              fs << "pose_x" << this->pose_x.Get();
              //              fs << "pose_y" << this->pose_y.Get();
              //              fs << "pose_yaw" <<  this->pose_yaw.Get();
              const std::vector<double> pose = {fs["pose_x"], fs["pose_y"], fs["pose_yaw"]};
              cout << "pose.size(): " << pose.size() << endl;
              for (std::vector<double>::const_iterator iter = pose.begin(); iter < pose.end(); ++iter)
              {
                cout << "*iter (pose values from X, Y, Yaw): " << *iter << endl;
              }

              /*! This is the output data -- NOT USED FOR LEARNING but only for RECOGNITION*/
              //              // output == controlling data
              //              fs << "desired_velocity" << this->desired_velocity.Get().Value();
              //              fs << "desired_angular_velocity" << this->desired_angular_velocity.Get().Value().Value();
              //              fs << "desired_fork_position" << this->desired_fork_position.Get();
              const double desired_velocity = fs["desired_velocity"];
              cout << "desired_velocity: " << desired_velocity << endl;
              const double desired_angular_velocity = fs["desired_angular_velocity"];
              cout << "desired_angular_velocity: " << desired_angular_velocity << endl;
              const double desired_fork_position = fs["desired_fork_position"];
              cout << "desired_fork_position: " << desired_fork_position << endl;

              cout << endl << "Closed and released after reading: " << endl;
              fs.release();


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
                const cv::Mat camera_image = cv::imread(camera_image_filepath/*, DEFAULT IST color*/); // cv::Mat and std::vector they are almost having the same data structure
                if (camera_image.size().width > 0 && camera_image.size().height > 0  && !camera_image.empty() /*camera image not empty*/)
                {
                  /*const */string window_name = "camera_image"; //camera_image_filename + "___" + time_stamp_string_label + "___" + cv::format("_%d", data_index);
                  cv::namedWindow(window_name, CV_WINDOW_NORMAL);
                  imshow(window_name, camera_image);

                  const cv::Mat camera_image_grayscale = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_GRAYSCALE);
                  window_name = "camera_image_GRAYSCALE";
                  cv::namedWindow(window_name, WINDOW_NORMAL);
                  cv::imshow(window_name, camera_image_grayscale);
                  cv::waitKey(1);
                  all_image_data_in_directory.push_back(camera_image_grayscale);


                  cv::Mat ir_distance_mat(ir_distance);
                  all_distance_data_in_directory.push_back(ir_distance_mat);

                  cv::Mat pose_mat(pose);
                  all_localization_data_in_directory.push_back(pose_mat);
                }// if camera image exit and not empty and have size
              }// main if all have size and not empty



            }// if xml file not empty
          } // if xml

        }// if has extension
      }// if regular file
    } // if exists

  }// for

  // the 1st one images
  cout << "all_image_data_in_directory.size(): " << all_image_data_in_directory.size() << endl;
  if (all_image_data_in_directory.size() > 0 && !all_image_data_in_directory.empty())
  {
    all_input_data_in_directory.push_back(all_image_data_in_directory);
  }

  cout << "all_distance_data_in_directory.size(): " << all_distance_data_in_directory.size() << endl;
  if (all_distance_data_in_directory.size() > 0 && !all_distance_data_in_directory.empty())
  {
    all_input_data_in_directory.push_back(all_distance_data_in_directory);
  }

  // the 3rd one in the array
  cout << "all_localization_data_in_directory.size(): " << all_localization_data_in_directory.size() << endl;
  if (all_localization_data_in_directory.size() > 0 && !all_localization_data_in_directory.empty())
  {
    all_input_data_in_directory.push_back(all_localization_data_in_directory);
  }

  cout << "all_input_data_in_directory: " << all_input_data_in_directory.size() << endl;  // this is should be basically only THREE IMG, DIST AND LOC

  // DEBUGGGING
  if (all_input_data_in_directory.size() != 3 || all_input_data_in_directory[0].size() != all_input_data_in_directory[1].size() || all_input_data_in_directory[0].size() != all_input_data_in_directory[2].size() || all_input_data_in_directory[1].size() != all_input_data_in_directory[2].size())
  {
    cout << "BUGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    //all_input_data_in_directory.size() is THREE
    cout << "all_input_data_in_directory.size(): " << all_input_data_in_directory.size() << endl;
    //all_input_data_in_directory[0].size() == all_input_data_in_directory[1].size() == all_input_data_in_directory[2].size()
    cout << "all_input_data_in_directory[0].size(): " << all_input_data_in_directory[0].size() << endl;
    cout << "all_input_data_in_directory[1].size(): " << all_input_data_in_directory[1].size() << endl;
    cout << "all_input_data_in_directory[2].size(): " << all_input_data_in_directory[2].size() << endl;
  }// sizes didnt match
  //  return all_image_data_in_directory;
  //return all_distance_data_in_directory; // this can be shown in histograms -- be careful this is DOUBLE PRECISION DATA
  //return all_localization_data_in_directory;
  return all_input_data_in_directory;
}// end of the function
