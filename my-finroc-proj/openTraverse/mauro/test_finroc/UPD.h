#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>


using namespace pcl;
using namespace std;

namespace finroc
{
namespace mauro
{
namespace test
{


//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

class UPD
{

public:
//  UPD(const vector<string> left_images, const unsigned img_pairs_num):
  UPD():
    image_cloud(new Cloud),
    colored_cloud(new Cloud),
    r_cloud(new PointCloud<Normal>()),
    normals(new PointCloud<Normal> ())
  {}

  ~UPD() {}

  void  viewerOneOff(visualization::PCLVisualizer& viewer);
  void  viewerPsycho(visualization::PCLVisualizer& viewer);

  uint32_t GiveRainbowColor(float position);
  Cloud& surf_creator();
  int  upd();
  void upd_filter();
  void upd_normals_extraction();
  void upd_trav();
  void upd_class();
  int run(int argc, char** argv);

  CloudPtr image_cloud;    //define input cloud
  CloudPtr colored_cloud;
  PointCloud<Normal>::Ptr r_cloud;
  PointCloud<Normal>::Ptr normals;
  int user_data;
  float normalRadiusSearch = 0.4;   // is the radius of the round in meter 1.0 for outside acquisition, 0.3 is good for the kinect sensor acquisition

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
}
