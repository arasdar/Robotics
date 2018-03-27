
#include "projects/stereo_traversability_experiments/aras/libstereo_test/gpu_finroc/gpu_part.h"

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace aras
{
namespace libstereo_test
{
namespace gpu
{

gpu_part::gpu_part() {}

gpu_part::~gpu_part() {}

/************postprocess*******/
void gpu_part::GPU_postProcess(const std::string& output_file, uchar* data_ptr)
{
  cv::Mat output(numRows, numCols, CV_8UC1, (void*)data_ptr);

  Mat img_hist_equalized;
  equalizeHist(output, img_hist_equalized);
  printf("%s\n", "hi karen5");
  //cv::Mat imageOutputBGR;
  //cv::cvtColor(output, imageOutputBGR, CV_GRAY2BGR);//CV_RGBA2BGR);
  //output the image
  //cv::imwrite(output_file.c_str(), imageOutputBGR);
  cv::imwrite(output_file.c_str(), img_hist_equalized);
}

}
}
}
}
}
