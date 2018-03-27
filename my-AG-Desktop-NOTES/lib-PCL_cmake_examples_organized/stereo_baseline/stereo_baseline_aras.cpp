/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/stereo/stereo_grabber.h>
#include <pcl/stereo/stereo_matching.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

  /** \brief stereo_baseline is a demonstration application for using stereo tools to display disparity maps and point clouds.
    *
    * \authour Federico Tombari
    * \author Aras Dargazany
    */

class HRCSBaseline
{
  private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer;

    std::vector<std::string> left_images;
    std::vector<std::string> right_images;
    int images_idx;
    int img_pairs_num;

    pcl::AdaptiveCostSOStereoMatching stereo;
    bool trigger;
    bool continuous;
    int smooth_weak;
    int smooth_strong;


  public:
    HRCSBaseline (std::vector<std::string> left_images, std::vector<std::string> right_images, const int img_pairs_num) :
      viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")),
      image_viewer (new pcl::visualization::ImageViewer ("Image Viewer"))
    {
      trigger = true;
      continuous = false;

      this->left_images = left_images;
      this->right_images = right_images;
      images_idx = 0;
      this->img_pairs_num = img_pairs_num;
      
      // Set up a 3D viewer
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      viewer->registerKeyboardCallback (&HRCSBaseline::keyboardCallback, *this, 0);
      
      // Set up the stereo matching
      stereo.setMaxDisparity(60);
      stereo.setXOffset(0);
      stereo.setRadius(5);

      smooth_weak = 20;
      smooth_strong = 100;
      stereo.setSmoothWeak(smooth_weak);
      stereo.setSmoothStrong(smooth_strong);
      stereo.setGammaC(25);
      stereo.setGammaS(10);
      
      stereo.setRatioFilter(20);
      stereo.setPeakFilter(0);
      
      stereo.setLeftRightCheck(true);
      stereo.setLeftRightCheckThreshold(1);
      
      stereo.setPreProcessing(true);
      
    }
    
    ~HRCSBaseline ()
    {
    }
    
    void
    keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.keyUp ())
      {
        switch (event.getKeyCode ())
        {
          case ' ':
            trigger = true;
            break;
          case 'c':
            continuous = !continuous;
            break;
        }
      }
    }
    
    void
    processStereoPair (const pcl::PointCloud<pcl::RGB>::Ptr& left_image, const pcl::PointCloud<pcl::RGB>::Ptr& right_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
    {
      stereo.compute (*left_image, *right_image);
      stereo.medianFilter (4);
      stereo.getPointCloud(318.112200f, 224.334900f, 368.534700f, 0.8387445f, out_cloud, left_image);
    }
    
    void run ()
    {
      while (!viewer->wasStopped ())
      {

		  // exting criteria
		  if (img_pairs_num == images_idx){
			cout << "img_pairs_num == images_idx " << images_idx << std::endl;
			break;
		  }

    	  // Process a new image
        if (trigger || continuous)
        {
          pcl::PointCloud<pcl::RGB>::Ptr left_cloud (new pcl::PointCloud<pcl::RGB>);
          pcl::PointCloud<pcl::RGB>::Ptr right_cloud (new pcl::PointCloud<pcl::RGB>);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

          pcl::PCDReader pcd;
          pcd.read (left_images[images_idx], *left_cloud);
          pcd.read (right_images[images_idx], *right_cloud);
          processStereoPair (left_cloud, right_cloud, out_cloud);
//          processCloud (out_cloud);

          // visualizing point clouds
          viewer->removePointCloud ("cloud");
    	  if (!viewer->updatePointCloud<PointT>  ( out_cloud, "cloud")){
    		  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> intensity(out_cloud);
    		  viewer->addPointCloud<pcl::PointXYZRGB> (out_cloud, intensity, "cloud");
    	  }

          ///displaying disparity map
     	  pcl::PointCloud<pcl::RGB>::Ptr vmap (new pcl::PointCloud<pcl::RGB>);
     	  stereo.getVisualMap(vmap);
     	  image_viewer->addRGBImage<pcl::RGB> (vmap);

          images_idx++;

          trigger = false;
        }

		// showing the input images
		cout << "left_images[img_index]: " << left_images[images_idx-1] << std::endl;
		cout << "right_images[img_index]: " << right_images[images_idx-1] << std::endl;
		cout << "images_idx: " << images_idx-1 << endl;
		cout << "img_pairs_num: " << img_pairs_num << endl;

        viewer->spinOnce (100);
        image_viewer->spinOnce (100);

      }//while

    }//run
    
};

int
main (int argc, char** argv)
{
  if (argc < 3)
  {
    PCL_INFO ("usage: pcl_stereo_baseline left_pcd_image_directory right_pcd_image_directory\n");
    PCL_INFO ("note: images must be in PCD format.  See stereo_png2pcd\n");
  }

	int img_number_left = 0, img_number_right = 0 ;
	int img_pairs_num = 0;

  // Get list of stereo files
  std::vector<std::string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (argv[1]); itr != end_itr; ++itr)
  {
    left_images.push_back (itr->path ().string ());
	img_number_left++;
  }
  sort (left_images.begin (), left_images.end ());
  std::vector<std::string> right_images;
  for (boost::filesystem::directory_iterator itr (argv[2]); itr != end_itr; ++itr)
  {
    right_images.push_back (itr->path ().string ());
	img_number_right++;
  }
  sort (right_images.begin (), right_images.end ());

  // showing the input images
  	cout << "img_number_left: " << img_number_left << std::endl;
  	cout << "img_number_right: " << img_number_right << std::endl;
  	if (img_number_left == img_number_right)
  		img_pairs_num = img_number_left;

  PCL_INFO ("Press space to advance to the next frame, or 'c' to enable continuous mode\n");


  // Process and display
  HRCSBaseline hrcs (left_images, right_images, img_pairs_num);
  hrcs.run ();
  
  return 0;
}
