//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/stereo_traversability_experiments/openTraverse/mauro/tUPD.cpp
 *
 * \author  Aras Dargazany
 *
 * \date    2014-10-20
 *
 */
//----------------------------------------------------------------------
#include "projects/stereo_traversability_experiments/openTraverse/mauro/tUPD.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transforms.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace stereo_traversability_experiments
{
namespace openTraverse
{
namespace mauro
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tUPD constructors
//----------------------------------------------------------------------
tUPD::tUPD():
  image_cloud(new Cloud),
  colored_cloud(new Cloud),
  r_cloud(new PointCloud<Normal>()),
  normals(new PointCloud<Normal> ())
//  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
{}

//----------------------------------------------------------------------
// tUPD destructor
//----------------------------------------------------------------------
tUPD::~tUPD()
{}

////----------------------------------------------------------------------
//// tUPD SomeExampleMethod functions implementations
////----------------------------------------------------------------------
//void tUPD::SomeExampleMethod()
//{
////  This is an example for a method. Replace it by your own methods!
//}


int  tUPD::upd()
{


  upd_filter();

  upd_normals_extraction();

  upd_trav();
  upd_class();


  return (0);
}


void tUPD::upd_filter()
{

//    Eigen::Matrix4f PCL_axis_transformation;              //define transformation matrix
//
//    PCL_axis_transformation  <<   0, -1, 0, 0,             //rigid transformation adapt for the visualization of the bumblebee camera
//                             0, 0, 1, 0,
//                             -1, 0, 0, 0,
//                             0, 0, 0, 1;
//
//    transformPointCloud(*image_cloud, *image_cloud, PCL_axis_transformation);

  pcl::VoxelGrid<PointT>* ds(new VoxelGrid<PointT>);   //create downsampling filter
  CloudPtr image_cloud_ds(new Cloud);
  ds->setInputCloud(image_cloud);
  float leafSize = 0.25; //0.3f; //0.2f;
  ds->setLeafSize(leafSize, leafSize, leafSize);
  ds->filter(*image_cloud_ds);
  cout << "Orginal cloud size = " << image_cloud->size() << endl;
  cout << "Voxel grid downsampled cloud size = " << image_cloud_ds->size() << endl;
  *image_cloud = *image_cloud_ds;
  image_cloud_ds->clear();

  pcl::StatisticalOutlierRemoval<PointT>* sor(new StatisticalOutlierRemoval<PointT>);
  sor->setInputCloud(image_cloud);
  sor->setMeanK(100);
  sor->setStddevMulThresh(0.5);
  sor->filter(*image_cloud_ds);
  cout << "Original cloud size = " << image_cloud->size() << endl;
  cout << "SOR cloud size = " << image_cloud_ds->size() << endl;
  *image_cloud = *image_cloud_ds;
  image_cloud_ds->clear();

}

void tUPD::upd_normals_extraction()
{

  // Estimate normals
  NormalEstimation<PointT, Normal> ne;
  image_cloud->resize(image_cloud->size() * 1);
  ne.setRadiusSearch(normalRadiusSearch);
  ne.setInputCloud(image_cloud);
  ne.compute(*normals);

  //flip normals towards a viewpoint

  for (size_t j = 0; j < image_cloud->size(); ++j)
  {
    flipNormalTowardsViewpoint(image_cloud->points[j], 0.0, 0.0, 0.0,
                               normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
  }/**/
}

void tUPD::upd_trav()
{

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////
  /*        TRAVERSABILITY ANALYSIS        */

  PointT searchPoint;
  searchPoint.x = image_cloud->points[1].x;
  searchPoint.y = image_cloud->points[1].y;
  searchPoint.z = image_cloud->points[1].z;

  KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(image_cloud);

  Eigen::Vector3d r_vector;
  r_cloud->resize(image_cloud->size() * 1);


  // Neighbors within radius search
  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;

  float radius = normalRadiusSearch;//0.4;

  /*    cout << "Neighbors within radius search at (" << searchPoint.x
           << " " << searchPoint.y
           << " " << searchPoint.z
           << ") with radius=" << radius << endl;
  */
  for (size_t j = 0; j < image_cloud->size(); ++j)
  {
    r_cloud->points[j].normal_x = 0;
    r_cloud->points[j].normal_y = 0;
    r_cloud->points[j].normal_z = 0;
    r_cloud->points[j].curvature = 0;
    searchPoint.x = image_cloud->points[j].x;
    searchPoint.y = image_cloud->points[j].y;
    searchPoint.z = image_cloud->points[j].z;

    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
      {
        r_cloud->points[j].normal_x = r_cloud->points[j].normal_x + normals->points[ pointIdxRadiusSearch[i] ].normal_x;
        r_cloud->points[j].normal_y = r_cloud->points[j].normal_y + normals->points[ pointIdxRadiusSearch[i] ].normal_y;
        r_cloud->points[j].normal_z = r_cloud->points[j].normal_z + normals->points[ pointIdxRadiusSearch[i] ].normal_z;
      }
      r_vector(0) = r_cloud->points[j].normal_x;
      r_vector(1) = r_cloud->points[j].normal_y;
      r_vector(2) = r_cloud->points[j].normal_z;
      float r_norm = r_vector.norm();

      //r_cloud->points[j].normal_x = r_cloud->points[j].normal_x;///r_norm;
      //r_cloud->points[j].normal_y = r_cloud->points[j].normal_y;///r_norm;
      //r_cloud->points[j].normal_z = r_cloud->points[j].normal_z;///r_norm;
      r_cloud->points[j].curvature = r_norm / pointIdxRadiusSearch.size();
      if (r_norm < (pointIdxRadiusSearch.size() / 2)*M_SQRT2)
      {
        //cout<< "misvalue found = " << r_vector.norm()/pointIdxRadiusSearch.size () << endl;
        r_cloud->points[j].curvature = 0;//r_vector.norm()/(pointIdxRadiusSearch.size ()+1);

        //cout<< "misvalue found = " << r_cloud->points[j].curvature << endl;
      }
      //r_cloud->points[j].curvature = normals->points[ j ].curvature;
    }
  }


  r_cloud->height = image_cloud->height;
  r_cloud->width = image_cloud->width;

}

void tUPD::upd_class()
{

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///UPD CLOUD CREATING
  float rgb_value;
  //colored_cloud->clear();
  colored_cloud->resize(image_cloud->size() * 1);
  PointCloud<PointSurfel>::Ptr UPD_cloud(new PointCloud<PointSurfel>);
  UPD_cloud->resize(image_cloud->size() * 1);
  for (size_t j = 0; j < image_cloud->size(); ++j)
  {
    UPD_cloud->points[j].x = image_cloud->points[j].x;
    UPD_cloud->points[j].y = image_cloud->points[j].y;
    UPD_cloud->points[j].z = image_cloud->points[j].z;
    UPD_cloud->points[j].rgba = image_cloud->points[j].rgba;
    UPD_cloud->points[j].normal_x = r_cloud->points[j].normal_x;
    UPD_cloud->points[j].normal_y = r_cloud->points[j].normal_y;
    UPD_cloud->points[j].normal_z = r_cloud->points[j].normal_z;
    UPD_cloud->points[j].radius = r_cloud->points[j].curvature;
    //float sink = sqrt(r_cloud->points[j].normal_x*r_cloud->points[j].normal_x+
    //                  r_cloud->points[j].normal_y*r_cloud->points[j].normal_y);
    UPD_cloud->points[j].confidence = 0;//(360/3.1415926536)*acos(r_cloud->points[j].normal_y/sink);
    //sink = sqrt(r_cloud->points[j].normal_z*r_cloud->points[j].normal_z+
    //r_cloud->points[j].normal_y*r_cloud->points[j].normal_y);
    UPD_cloud->points[j].curvature = 0; // (360/3.1415926536)*acos(r_cloud->points[j].normal_y/sink);

    colored_cloud->points[j].x = image_cloud->points[j].x;
    colored_cloud->points[j].y = image_cloud->points[j].y;
    colored_cloud->points[j].z = image_cloud->points[j].z;
    if (UPD_cloud->points[j].normal_x * UPD_cloud->points[j].normal_x > UPD_cloud->points[j].normal_y * UPD_cloud->points[j].normal_y)
    {
      colored_cloud->points[j].rgba = 0x00FF0000;
    }
    else if (UPD_cloud->points[j].normal_z * UPD_cloud->points[j].normal_z > UPD_cloud->points[j].normal_y * UPD_cloud->points[j].normal_y)
    {
      colored_cloud->points[j].rgba = 0x00FF0000;
    }
    else if (UPD_cloud->points[j].curvature > 0.9 && UPD_cloud->points[j].curvature < 0.99)
    {
      rgb_value = (r_cloud->points[j].curvature - 0.9) / 0.1;
      colored_cloud->points[j].rgba = GiveRainbowColor(rgb_value);
    }
    else
    {
      rgb_value = (r_cloud->points[j].curvature - 0.7) / 0.3;
      colored_cloud->points[j].rgba = GiveRainbowColor(rgb_value);
    }
  }
  UPD_cloud->height = image_cloud->height;
  UPD_cloud->width = image_cloud->width;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////Rainbow color creator///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t tUPD::GiveRainbowColor(float position)

// this function gives 1D linear RGB color gradient
// color is proportional to position
// position  <0;1>
// position means position of color in color gradient

{
  if (position > 1)position = 1; //position-int(position);
  // if position > 1 then we have repetition of colors
  // it maybe useful
  uint8_t R, G, B;// byte
  int nmax = 6; // number of color bars
  float m = nmax * position;
  int n = int(m); // integer of m
  float f = m - n; // fraction of m
  uint8_t t = int(f * 255);


  switch (n)
  {
  case 0:
  {
    R = 0;
    G = 255;
    B = t;
    break;
  }

  case 1:
  {
    R = 0;
    G = 255 - t;
    B = 255;
    break;
  }
  case 2:
  {
    R = t;
    G = 0;
    B = 255;
    break;
  }
  case 3:
  {
    R = 255;
    G = 0;
    B = 255 - t;
    break;
  }
  case 4:
  {
    R = 255;
    G = t;
    B = 0;
    break;
  }
  case 5:
  {
    R = 255 - t;
    G = 255;
    B = 0;
    break;
  }
  case 6:
  {
    R = 0;
    G = 255;
    B = 0;
    break;
  }

  }; // case


  return (R << 16) | (G << 8) | B;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
