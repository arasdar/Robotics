/** \brief StereoVisioProcessing is an application for processing stereo images to classify terrain using stereo camera.
  *
  * \author Aras Dargazany
  *
  *  * this is sense and control module for stereo vision processing (StViPr)
  *
  */

//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/png_io.h>  //why error!!
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/segmentation/ground_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

//filtering
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>

//passthrough filter
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//stereo
//#include <pcl/stereo/stereo_grabber.h> //not used //???
#include <pcl/stereo/stereo_matching.h>
//#include "projects/icarus/simulation_physics/stereo/stereo_matching.h"
//#include "projects/icarus/sensorProcessing/libstereo/stereo_matching.h"

//problems
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>


// debugging
#include <iostream>
#include <vector>
#include <stdio.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

// using namespace
using namespace std;
using namespace pcl;
using namespace cv; //opencv

// template
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
