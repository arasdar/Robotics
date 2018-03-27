


//#include "UPD.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

namespace finroc
{
namespace mauro
{
namespace test
{


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////Point cloud surface creator///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloud<PointXYZRGBA>& UPD::surf_creator()
{
  PointCloud<PointXYZRGBA>::Ptr surface(new PointCloud<PointXYZRGBA>);
  PointCloud<PointXYZRGBA>::Ptr point(new PointCloud<PointXYZRGBA>);
  point->resize(1 * 1);
  for (float x = -0.5f; x <= 0.5f; x += 0.1f)
  {
    for (float y = -0.5f; y <= 0.5f; y += 0.1f)
    {

      point->points[0].x = x;
      point->points[0].y = y;
      point->points[0].z = 0;// 2.0f - y;
      point->points[0].rgba = 0x00000000;

      *surface += *point;
    }
  }
  for (float x = -0.5f; x <= 0.5f; x += 0.1f)
  {
    for (float z = -0.5f; z <= 0.5f; z += 0.1f)
    {

      point->points[0].x = x;
      point->points[0].y = 0.51f;
      point->points[0].z = 0.5f + z; // 2.0f - y;
      point->points[0].rgba = 0x00000000;

      *surface += *point;
    }
  }/*
for (float y=-0.5f; y<=0.5f; y+=0.05f)
{
  for (float z=-0.5f; z<=0.5f; z+=0.05f)
  {

    point->points[0].x = 0.5f;
    point->points[0].y = y;
    point->points[0].z = 0.5f+z;// 2.0f - y;
    point->points[0].rgba = 0x000000FF;

    *surface+=*point;
  }
}
/**/
  surface->width = (int) surface->points.size();
  surface->height = 1;

  return (*surface);
}

}
}
}
