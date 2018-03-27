#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>
#include <ctime>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace std;

namespace finroc
{
namespace mauro
{

class UPD
{

public:
  UPD():
    image_cloud(new PointCloud<PointXYZRGBA>),
    colored_cloud(new PointCloud<PointXYZRGBA>),
    r_cloud(new PointCloud<Normal>()),
    normals(new PointCloud<Normal> ())
  {}

//  UPD():
//  viewer(new visualization::cloudViewer("cloud viewer")){}

  ~UPD() {}

  void  viewerOneOff(visualization::PCLVisualizer& viewer);
  void  viewerPsycho(visualization::PCLVisualizer& viewer);

  uint32_t GiveRainbowColor(float position);
  PointCloud<PointXYZRGBA>& surf_creator();
  int  upd();
  int run(int argc, char** argv);

//  PointCloud<PointXYZRGBA>::Ptr image_cloud(new PointCloud<PointXYZRGBA>);    //define input cloud
//  PointCloud<PointXYZRGBA>::Ptr colored_cloud(new PointCloud<PointXYZRGBA>);
//  PointCloud<Normal>::Ptr r_cloud(new PointCloud<Normal> ());
//  PointCloud<Normal>::Ptr normals(new PointCloud<Normal> ());

  PointCloud<PointXYZRGBA>::Ptr image_cloud;    //define input cloud
  PointCloud<PointXYZRGBA>::Ptr colored_cloud;
  PointCloud<Normal>::Ptr r_cloud;
  PointCloud<Normal>::Ptr normals;
//  visualization::CloudViewer* viewer (new visualization::CloudViewer("cloud viewer"));
//  visualization::CloudViewer viewer;
  int user_data;

  // --------------------
  // -----Structure for the stop watch -----
  // --------------------

  int64_t timespecDiff(struct timespec *timeA_p, struct timespec *timeB_p)
  {
    return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
           ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
  }

};

}
}
