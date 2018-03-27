


#include "projects/stereo_traversability_experiments/MLR/write/mWrite.h"
#include "rrlib/coviroa/opencv_utils.h" // AccessImageAsMat
#include <boost/filesystem.hpp>

using namespace std;

void finroc::stereo_traversability_experiments::write::mWrite::write_xml()
{
  /*! All the input and output data should be accessible here for writing to memory with label */
  // IO= {IO1,...................IOt} // IO data & t label
  /*! Start writing down the data and saving them in the memory */

  /*! Accessing the image from the port and in opencv format Mat */
  data_ports::tPortDataPointer<const rrlib::coviroa::tImage> img = this->camera_image.GetPointer();
  cv::Mat img_mat;
  img_mat = rrlib::coviroa::AccessImageAsMat(*img);
  std::string win_name("img_mat from camera image");
  cv::namedWindow(win_name, CV_WINDOW_NORMAL);
  cv::imshow(win_name, img_mat);
  cv::waitKey(1); //1 mili second

  /*
    // saving the frames in right and left folders
    char filename[100];
    sprintf(filename, "left/left%.2d.png", frame_number); // this should be like name%.4digits 00> 0001, ....1000
    imwrite(filename, frame_1);
    cout << "Saved " << filename << endl;
  */

  // This is the actual creation of directory for camera images
  const std::string camera_image_dir(this->dir_name + "/" + "camera_images");
  boost::filesystem::path p(camera_image_dir);
  if (!boost::filesystem::is_directory(p)) // it already exist or already have been written to the memory -- unloade recorded_
  {
    // This is the actual creation of directory
    if (boost::filesystem::create_directories(p))
    {
      std::cout  << p.string() << " directory created for camera images .............." << "\n";
    }
  }// if camera dir doesnt exist
  //      const std::string camera_image_filename = cv::format("camera_image_%d.png", this->data_index); only one digit 1....1000
  const std::string camera_image_filename = "camera_images/" + cv::format("camera_image_%.4d.png", this->data_index); // 0001 ....1000
  const std::string camera_image_filepath = this->dir_name + "/" + camera_image_filename;
  std::cout << camera_image_filepath << std::endl;
  cv::imwrite(camera_image_filepath, img_mat /*, Compression_param*/ /*CV_IMWRITE_PNG_COMPRESSION*/);


//  const std::string data_filename = cv::format("data_%d.xml", this->data_index);
  const std::string data_filename = cv::format("data_%.4d.xml", this->data_index);
  const std::string data_filepath = this->dir_name + "/" + data_filename;
  cout << data_filepath << endl;
  cv::FileStorage fs(data_filepath, cv::FileStorage::WRITE);
  // the configuration distance sensors in anti clockwise - this might also b important - front, l, ll, lll, rear, r, rr, rrr
  fs << "time_stamp_string_label" << this->time_stamp_string_label; // this is dir label - date --> IS THIS NECCESSARY???
  fs << "dir_name" << this->dir_name; // this is the learned directory & the written directory for recorded sequence & inside there are the index sequence frames
  fs << "data_index" << this->data_index; // this is file label inside dir - index
  fs << "camera_image_filename" << camera_image_filename;
  fs << "ir_distance_front_value" << this->ir_distance_front.Get().Value();
  fs << "ir_distance_l_value" << this->ir_distance_l.Get().Value();
  fs << "ir_distance_ll_value" << this->ir_distance_ll.Get().Value();
  fs << "ir_distance_lll_value" << this->ir_distance_lll.Get().Value();
  fs << "ir_distance_rear_value" << this->ir_distance_rear.Get().Value();
  fs << "ir_distance_rrr_value" << this->ir_distance_rrr.Get().Value();
  fs << "ir_distance_rr_value" << this->ir_distance_rr.Get().Value();
  fs << "ir_distance_r_value" << this->ir_distance_r.Get().Value();
  //fs << "localization data" ;
  fs << "pose_x" << this->pose_x.Get();
  fs << "pose_y" << this->pose_y.Get();
  fs << "pose_yaw" <<  this->pose_yaw.Get();
  // output == controlling data
  fs << "desired_velocity" << this->desired_velocity.Get().Value();
  fs << "desired_angular_velocity" << this->desired_angular_velocity.Get().Value().Value();
  fs << "desired_fork_position" << this->desired_fork_position.Get();
  fs.release();                                       // explicit close

  this->data_index++;
  cout << "Written to memory................................." << endl;
}// write_xml()
