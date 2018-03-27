#ifndef MAP_FROM_G2O_H
#define MAP_FROM_G2O_H

#include "projects/stereo_traversability_experiments/openTraverse/igor/traversability_analyzer.h"

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace igor
{

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> Cloud;
//typedef Cloud::ConstPtr CloudConstPt;
//typedef Cloud::Ptr CloudPtr;


class Worker
{
public:
  Worker();
  void processPCLToMap(int maxAngle,
                       double cellWidth,
                       double maxTravHeight,
                       double radiusNormals,
                       double robotHeight,
                       string outputImage);

  void processCloudToMap(int maxAngle,
                         double cellWidth,
                         double maxTravHeight,
                         double radiusNormals,
                         double robotHeight,
                         CloudPtr& cloud);

//  void processCloud2Map(int maxAngle,
//          double cellWidth,
//          double maxTravHeight,
//          double radiusNormals,
//          double robotHeight,
//          CloudPtr pcd);
};

}
}
}
}


#endif
