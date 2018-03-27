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

void mRecognition::output(/*INPUT*/const std::vector<RecordedData>& vec_recorded_data)
{

  //  comparison_results[0] = vec_diff_norms[0] /*= diff_norm*/;
  //  comparison_results[1] = vec_diff_norms[1] /*= diff_mu_norm*/;
  //  comparison_results[2] = vec_diff_norms[2] /*= diff_mu_V_norm*/;
  //  comparison_results[3] = vec_diff_norms[3] /*= diff_mu_V_S_norm*/;
  //
  //  comparison_results[4] = vec_sim_values[0] /*= sim_value*/;
  //  comparison_results[5] = vec_sim_values[1] /*= sim_mu_value*/;
  //  comparison_results[6] = vec_sim_values[2] /*= sim_mu_V_value*/;
  //  comparison_results[7] = vec_sim_values[3] /*= sim_mu_V_S_value*/;
  if (std::abs(vec_recorded_data[6]/*sim_mu_V_value*/.data_index - vec_recorded_data[2]/*diff_mu_V_norm*/.data_index) < 1) // the same
  {
    if (std::abs(vec_recorded_data[7]/*sim_mu_V_S_value*/.data_index - vec_recorded_data[3]/*diff_mu_V_S_norm*/.data_index) < 1) // meaning if they were equal
    {
      *this->recorded_data = vec_recorded_data[3]; //diff_mu_V_S_norm
      // make sure recorded data is not empty -- TESTING THE OUTPUT BEFORE PASSING IT
      cout << "time_stamp_string_label: " << this->recorded_data->time_stamp_string_label << endl;
      cout << "dir_name: " << this->recorded_data->dir_name << endl;
      cout << "data_index: " << this->recorded_data->data_index << endl;
      cout << "ir_distance.size(): " << this->recorded_data->ir_distance.size() << endl;
      cout << "pose.size(): " << this->recorded_data->pose.size() << endl;
    }
    else
    {
      *this->recorded_data = vec_recorded_data[2]; //diff_mu_V_norm
      cout << "time_stamp_string_label: " << this->recorded_data->time_stamp_string_label << endl;
      cout << "dir_name: " << this->recorded_data->dir_name << endl;
      cout << "data_index: " << this->recorded_data->data_index << endl;
      cout << "ir_distance.size(): " << this->recorded_data->ir_distance.size() << endl;
      cout << "pose.size(): " << this->recorded_data->pose.size() << endl;
    }
  }
  else // if the best match was NOT found
  {
    // OUTPUT to controllers or controlling data or controlling commands should be ALL ZERO or set to SERO
    this->recorded_data->desired_velocity = 0.;
    this->recorded_data->desired_angular_velocity = 0.;
    this->recorded_data->desired_fork_position = 0.;
  }

  cout << "this->recorded_data->desired_velocity: " << this->recorded_data->desired_velocity << endl;
  cout << "this->recorded_data->desired_angular_velocity: " << this->recorded_data->desired_angular_velocity << endl;
  cout << "this->recorded_data->desired_fork_position: " << this->recorded_data->desired_fork_position << endl;

}// end of the function
