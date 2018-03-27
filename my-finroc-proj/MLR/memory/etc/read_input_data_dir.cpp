
#include "projects/stereo_traversability_experiments/def/tMLR.h"

using namespace cv;

/*int main(int argc, const char** argv)*/
vector<Mat> finroc::stereo_traversability_experiments::def::tMLR::read_input_data_dir(int argc, const char** argv)
{

  // Get list of input_data in a directory -- mass reading and iterating through a directory
  std::vector<std::string> input_data_path;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr; ++itr)
  {
    input_data_path.push_back(itr->path().string());
  }
  sort(input_data_path.begin(), input_data_path.end());

  // These vectors hold the input_data and corresponding labels.
  /*const*/ vector<Mat> input_data; // these are input input_data and that s why they are defined as constant to make sure they are not changed any where!!

  // reading the input_data from path
  for (vector<string>::iterator iter = input_data_path.begin(); iter != input_data_path.end(); ++iter)
  {
    string path = *iter;
    if (!path.empty() /*&& !classlabel.empty()*/)
    {
      unsigned int flags = 0; //CV_LOAD_IMAGE_GRAYSCALE;
      Mat sample = imread(path, flags);
      bool is_active = false;
      LOG_PRINT(sample, is_active);
      input_data.push_back(sample);
    }

  }

  return input_data; // this is RAW input data
}




/*!
    data_ports::tPortDataPointer<const rrlib::coviroa::tImage> img = camera_image.GetPointer();
    cv::Mat img_mat;
    img_mat = rrlib::coviroa::AccessImageAsMat(*img);
    string win_name("img_mat from camera image");
    cv::namedWindow(win_name, CV_WINDOW_NORMAL);
    cv::imshow(win_name, img_mat);
    cv::waitKey(1); //1 mili second

    //const string dir_path = "/home/aras/database/robot_control/recorded_IO/test/images/";   IMAGES FOLDER AND PATH
    const string camera_image_filename = cv::format("camera_image_%d.png", this->data_index);
    const string camera_image_filepath = this->dir_name + "/" + camera_image_filename;
    cout << camera_image_filepath << endl;
    cv::imwrite(camera_image_filepath, img_mat);

    const string IO_data_filename = cv::format("IO_data_%d.xml", this->data_index);
    const string IO_data_filepath = this->dir_name + "/" + IO_data_filename;
    cout << IO_data_filepath << endl;
    cv::FileStorage fs(IO_data_filepath, cv::FileStorage::WRITE);
    // the configuration distance sensors in anti clockwise - this might also b important - front, l, ll, lll, rear, r, rr, rrr
    fs << "time_stamp_string_label" << this->time_stamp_string_label; // this is dir label - date
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
    cout << "Write Done." << endl;

    */
