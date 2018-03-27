


#include "projects/stereo_traversability_experiments/daniel/stereo_color_original/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_original::offline;

void
tStereoProcessing::run_saveCloud(const CloudConstPtr& cloud)
{
  std::stringstream ss;

  /*an alternative*/
  //char str[512];
  //sprintf(str, "text_id_%03d", text_id);

  if (images_idx < 10 && images_idx < 100 && images_idx < 1000)
  {
    ss << dir_name_ << "/" << file_name_ << "_00" << images_idx << ".pcd";
  }
  if (images_idx >= 10 && images_idx < 100 && images_idx < 1000)
  {
    ss << dir_name_ << "/" << file_name_ << "_0" << images_idx << ".pcd";
  }
  if (images_idx > 10 && images_idx >= 100 && images_idx < 1000)
  {
    ss << dir_name_ << "/" << file_name_ << "_" << images_idx << ".pcd";
  }

  cout << "Saved " << ss.str() << endl;

  if (format_ & 1)
  {
    writer_.writeBinary<PointT> (ss.str(), *cloud);
    std::cerr << "Data saved in BINARY format to: " << ss.str() << std::endl;
  }

  if (format_ & 2)
  {
    writer_.writeBinaryCompressed<PointT> (ss.str(), *cloud);
    std::cerr << "Data saved in BINARY COMPRESSED format to: " << ss.str() << std::endl;
  }
}

//void
//tStereoProcessing::run_saveCloud_left(const PointCloud<RGB>::ConstPtr& cloud)
//{
//  std::stringstream ss;
//  ss << "frames_pcd_left_RGB" << "/" << "left_pcd" << "_" << images_idx << ".pcd";
//  cout << "Saved " << ss.str() << endl;
//  writer_.writeBinaryCompressed<RGB> (ss.str(), *cloud);
//}
//
//void
//tStereoProcessing::run_saveCloud_disp(const PointCloud<RGB>::ConstPtr& cloud)
//{
//  std::stringstream ss;
//  ss << "frames_pcd_disp_pcd" << "/" << "disp_pcd" << "_" << images_idx << ".pcd";
//  cout << "Saved " << ss.str() << endl;
//  writer_.writeBinaryCompressed<RGB> (ss.str(), *cloud);
//}
