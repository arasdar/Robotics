/*
 * getPointCloud.cpp
 *
 *  Created on: Apr 30, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

using namespace finroc::stereo_traversability_experiments::aras::libstereo;

//////////////////////////////////////////////////////////////////////////////
bool
tTestStereoMatching::getPointCloud(
  float u_c, float v_c, float focal, float baseline,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  //disp map has not been computed yet..
  if (disp_map_ == NULL)
  {

    PCL_ERROR(
      "[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n"
    );

    return false;
  }

  //cloud needs to be re-allocated
  if (cloud->width != width_ || cloud->height != height_)
  {
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  if (cloud->is_dense)
    cloud->is_dense = false;

  //Loop
  pcl::PointXYZ temp_point;
  pcl::PointXYZ nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  //nan_point.intensity = std::numeric_limits<float>::quiet_NaN();

  //all disparities are multiplied by a constant equal to 16;
  //this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (unsigned j = 0; j < height_; j++)
  {
    for (unsigned i = 0; i < width_; i++)
    {
      if (disp_map_[ j * width_ + i] > 0)
      {

        temp_point.z = depth_scale / disp_map_[j * width_ + i];
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;
        //temp_point.intensity = 255;

        (*cloud)[j * width_ + i] = temp_point;
      }
      //adding NaN value
      else
      {
        (*cloud)[j * width_ + i] = nan_point;
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
tTestStereoMatching::getPointCloud(
  float u_c, float v_c, float focal, float baseline,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  //disp map has not been computed yet..
  if (disp_map_ == NULL)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n");

    return (false);
  }

  //cloud needs to be re-allocated
  if (cloud->width != width_ || cloud->height != height_)
  {
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  //Loop
  pcl::PointXYZRGB temp_point;
  /*pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.b = std::numeric_limits<unsigned char>::quiet_NaN();*/

  //all disparities are multiplied by a constant equal to 16;
  //this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (unsigned j = 0; j < height_; j++)
  {
    for (unsigned i = 0; i < width_; i++)
    {
      temp_point.r = ref_img_[j * width_ + i];
      temp_point.g = ref_img_[j * width_ + i];
      temp_point.b = ref_img_[j * width_ + i];

      if (disp_map_[ j * width_ + i] > 0)
      {
        temp_point.z = (depth_scale) / (disp_map_[ j * width_ + i]);
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;
        (*cloud)[j * width_ + i] = temp_point;
      }
      //adding NaN value
      else
      {
        temp_point.x = std::numeric_limits<float>::quiet_NaN();
        temp_point.y = std::numeric_limits<float>::quiet_NaN();
        temp_point.z = std::numeric_limits<float>::quiet_NaN();
        (*cloud)[j * width_ + i] = temp_point;
      }
    }
  }

  return (true);
}


//////////////////////////////////////////////////////////////////////////////
bool
tTestStereoMatching::getPointCloud(
  float u_c, float v_c, float focal, float baseline,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  pcl::PointCloud<pcl::RGB>::Ptr texture)
{
  //disp map has not been computed yet..
  if (disp_map_ == NULL)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n");

    return (false);
  }

  if (texture->width != width_ || texture->height != height_)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: the size of the texture cloud does not match that of the computed range map. The resulting cloud can not be computed..\n");
    return (false);
  }

  //cloud needs to be re-allocated
  if (cloud->width != width_ || cloud->height != height_)
  {
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  //Loop
  pcl::PointXYZRGB temp_point;
  /*pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.b = std::numeric_limits<unsigned char>::quiet_NaN();*/

  //all disparities are multiplied by a constant equal to 16;
  //this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (int j = 0; j < height_; j++)
  {
    for (int i = 0; i < width_; i++)
    {
      //temp_point.intensity = ( texture->at(j*width_+i).r +texture->at(j*width_+i).g + texture->at(j*width_+i).b) / 3.0f;
      temp_point.r = texture->at(j * width_ + i).r;
      temp_point.g = texture->at(j * width_ + i).g;
      temp_point.b = texture->at(j * width_ + i).b;

      if (disp_map_[ j * width_ + i] > 0)
      {
        temp_point.z = (depth_scale) / (disp_map_[ j * width_ + i]);
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;
        (*cloud)[j * width_ + i] = temp_point;
      }
      //adding NaN value
      else
      {
        temp_point.x = std::numeric_limits<float>::quiet_NaN();
        temp_point.y = std::numeric_limits<float>::quiet_NaN();
        temp_point.z = std::numeric_limits<float>::quiet_NaN();
        (*cloud)[j * width_ + i] = temp_point;
      }
    }
  }

  return (true);
}


////////////////////////////////////////////////////////////////////////////////
bool
tTestStereoMatching::getPointCloud(
  float u_c, float v_c, float focal, float baseline,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
  cv::Mat texture)
{
  //disp map has not been computed yet..
  if (disp_map_ == NULL)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: a disparity map has not been computed yet. The resulting cloud can not be computed..\n");

    return (false);
  }

  if (texture.cols != width_ || texture.rows != height_)
  {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: the size of the texture cloud does not match that of the computed range map. The resulting cloud can not be computed..\n");
    return (false);
  }

  //cloud needs to be re-allocated
  if (cloud->width != width_ || cloud->height != height_)
  {
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  //Loop
  pcl::PointXYZRGB temp_point;
  /*pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.b = std::numeric_limits<unsigned char>::quiet_NaN();*/

  //all disparities are multiplied by a constant equal to 16;
  //this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (int j = 0; j < height_; j++)
  {
    for (int i = 0; i < width_; i++)
    {
      //temp_point.intensity = ( texture->at(j*width_+i).r +texture->at(j*width_+i).g + texture->at(j*width_+i).b) / 3.0f;
      //vMap.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
      cv::Vec3b rgb = texture.at<cv::Vec3b>(j, i);
      temp_point.r = rgb[0]; //texture->at(j * width_ + i).r;
      temp_point.g = rgb[1]; //texture->at(j * width_ + i).g;
      temp_point.b = rgb[2]; //texture->at(j * width_ + i).b;


      if (disp_map_[ j * width_ + i] > 0)
      {
        temp_point.z = (depth_scale) / (disp_map_[ j * width_ + i]);
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;
        (*cloud)[j * width_ + i] = temp_point;
      }
      //adding NaN value
      else
      {
        temp_point.x = std::numeric_limits<float>::quiet_NaN();
        temp_point.y = std::numeric_limits<float>::quiet_NaN();
        temp_point.z = std::numeric_limits<float>::quiet_NaN();
        (*cloud)[j * width_ + i] = temp_point;
      }
    }
  }

  return (true);
}
