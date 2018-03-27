#include <iostream> // cout , cin
#include <boost/filesystem.hpp> // pretty obvious
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/highgui/highgui.hpp> // imread and imshow ...

using namespace boost::filesystem;
using namespace std;

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"
using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::load_filenames(/*INPUT*/const string& dir_path, /*OUTPUT*/std::vector<std::string>& filenames, /*OUTPUT*/ std::string& file_parent_path)
{
  for (auto it : recursive_directory_range(dir_path))
  {
    path p(it);    // p reads clearer than argv[1] in the following code

    if (exists(p))    // does p actually exist?
    {
      if (is_regular_file(p))        // is p a regular file?
      {
        if (p.has_extension() && p.has_stem() && p.has_filename())
        {
          if (p.extension() == ".xml")
          {
            if (!p.empty())  // if the XML file is not empty
            {
              const string XML_filepath(p.string());  // it has to be CONSTANT in one iteration
              cv::FileStorage fs(XML_filepath, cv::FileStorage::READ);

              // First of all let s check if we can open the file
              if (!fs.isOpened())
              {
                cerr << "Fail to open: " << XML_filepath << endl;
                //return 1;
                exit(1);
              }

              //const string time_stamp_string_label = fs["time_stamp_string_label"];
              //const /*unsigned*/ int data_index = fs["data_index"];
              const string camera_image_filename = fs["camera_image_filename"];
              const std::vector<double> ir_distance = {fs["ir_distance_front_value	"],
                                                       fs["ir_distance_lll_value"], fs["ir_distance_ll_value"], fs["ir_distance_l_value"],
                                                       fs["ir_distance_rear_value"],
                                                       fs["ir_distance_r_value"], fs["ir_distance_rr_value"], fs["ir_distance_rrr_value"]
                                                      }; // from front -- anti clockwise to rrr
              const std::vector<double> pose = {fs["pose_x"], fs["pose_y"], fs["pose_yaw"]};
              //const double desired_velocity = fs["desired_velocity"];
              //const double desired_angular_velocity = fs["desired_angular_velocity"];
              //const double desired_fork_position = fs["desired_fork_position"];
              fs.release();


              // start loading the recorded camera images in database -- I_img
              const string camera_image_filepath = p.parent_path().string() + "/" + camera_image_filename;
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
                  const string window_name = "loading the database of camera images";
                  cv::namedWindow(window_name, WINDOW_NORMAL);
                  cv::imshow(window_name, camera_image);
                  cv::waitKey(1);

                  // Loading the final OUTPUT
                  filenames.push_back(XML_filepath);
                  std::cout << XML_filepath << " loadedddddddddddddddddddd........................." << std::endl;
                  file_parent_path = p.parent_path().string();
                  std::cout << file_parent_path << " loadedddddddddddd tooooooooooooooooooooooooooooooooooooooooo" << std::endl;

                }// if camera image exist and not empty and have size
              }// main if all have size and not empty
            }// if xml file not empty
          } // if xml
        }// if has extension
      }// if regular file
    } // if exists
  }// for

  cv::destroyAllWindows();

}// end of the function
