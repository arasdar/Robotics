#include "projects/stereo_traversability_experiments/MLR/control/mRecognition.h"
#include "rrlib/coviroa/opencv_utils.h" // AccessImageAsMat

using namespace finroc::stereo_traversability_experiments::control;

void mRecognition::recognizer_process()
{
  //main intellignet control function

  //    At least one of your input ports has changed. Do something useful with its data.
  //    However, using the .HasChanged() method on each port you can check in more detail.
  // Prepare the I_new first -- should available - the values
  /*! Accessing the image from the port and in opencv format Mat */
  /*const*/ data_ports::tPortDataPointer<const rrlib::coviroa::tImage> img = camera_image.GetPointer();
  cv::Mat I_new_img = rrlib::coviroa::AccessImageAsMat(*img);
  //    std::string win_name("I_new_img");
  //    cv::namedWindow(win_name, CV_WINDOW_NORMAL);
  //    cv::imshow(win_name, I_new_img);

  // from 3 channels to one --> RGB2GRAY
  cv::Mat I_new_img_gray;
  cv::cvtColor(I_new_img, I_new_img_gray, CV_RGB2GRAY);
  I_new_img.release();
  //    /*std::string */win_name = ("I_new_img_gray");
  //    cv::namedWindow(win_name, CV_WINDOW_NORMAL);
  //    cv::imshow(win_name, I_new_img_gray);

  // scale
  cv::Mat I_new_img_scaled = mlr->toGrayscale(I_new_img_gray);
  I_new_img_gray.release(); // release RAM and CACHE - deallocate sme memory and heap
  //    /*std::string */win_name = ("I_new_img_gray_scaled");
  //    cv::namedWindow(win_name, CV_WINDOW_NORMAL);
  //    cv::imshow(win_name, I_new_img_scaled);

  // vectorize
  //  std::vector<cv::Mat> vec_I_new_img_scaled;
  //  vec_I_new_img_scaled.push_back(I_new_img_scaled);
  //  I_new_img_scaled.release();
  //  cv::Mat I_new_img_vec = mlr->vectorize_input_data(vec_I_new_img_scaled); // TODO only for one -- implemented only needs to be replaced
  //  vec_I_new_img_scaled.clear();
  cv::Mat I_new_img_vec = mlr->vectorize_input_data(I_new_img_scaled);
  I_new_img_scaled.release();
  mlr->LOG_PRINT(I_new_img_vec, "I_new_img_vec", mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

  // project
  mlr->LOG_PRINT(pca->mean, "pca->mean", mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  mlr->LOG_PRINT(pca->S, "pca->S", mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  mlr->LOG_PRINT(pca->V, "pca->V", mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);
  if (!pca->mean.empty() && !pca->S.empty() && !pca->V.empty()) // they were all loaded
  {
    cv::Mat I_new_img_vec_projected = pca->project(I_new_img_vec.col(0));
    I_new_img_vec.release();
    mlr->LOG_PRINT(I_new_img_vec_projected, "I_new_img_vec_projected", mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);

    // Here is the actual recognition real-time -- comparison is done here every single time
    //    /*mlr->*/recognizer_search_database(I_new_img_vec_projected, this->filenames, this->file_parent_path, *this->pca, *recorded_data);
    mlr::RecordedData O_new_recorded_data;
    recognizer_search_database(I_new_img_vec_projected, O_new_recorded_data);
    //std::cout << "O_new_recorded_data.desired_velocity: " << O_new_recorded_data.desired_velocity << std::endl;
    recorded_data = &O_new_recorded_data;
  }// pca loaded

  //  //Do something each cycle independent from changing ports.
  //  // Publishing the final output AS controller data
  /*  //  this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
  // These are for recieving the final output and publishing it to the ports
  */
  data_ports::tPortDataPointer<rrlib::si_units::tVelocity<>> velocity_buffer/*_to_publish*/ = this->desired_velocity.GetUnusedBuffer();
  *velocity_buffer = recorded_data->desired_velocity;
  FINROC_LOG_PRINT(DEBUG, "*velocity_buffer: ", *velocity_buffer);
  desired_velocity.Publish(velocity_buffer/*_to_publish*/);

  data_ports::tPortDataPointer<rrlib::si_units::tAngularVelocity<>> angular_velocity_buffer/*_to_publish*/ = this->desired_angular_velocity.GetUnusedBuffer();
  *angular_velocity_buffer = recorded_data->desired_angular_velocity;
  FINROC_LOG_PRINT(DEBUG, "*angular_velocity_buffer: ", *angular_velocity_buffer);
  desired_angular_velocity.Publish(angular_velocity_buffer/*_to_publish*/);

  data_ports::tPortDataPointer<double> fork_position_buffer/*_to_publish*/ = this->desired_fork_position.GetUnusedBuffer();
  *fork_position_buffer = recorded_data->desired_fork_position;
  FINROC_LOG_PRINT(DEBUG, "*fork_position_buffer: ", *fork_position_buffer);
  desired_fork_position.Publish(fork_position_buffer/*_to_publish*/);
} // void mRecognition::recognizer_load()
