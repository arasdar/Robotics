


#include "UPD.h"

namespace finroc
{
namespace mauro
{

int  UPD::upd()
{

  Eigen::Matrix4f PCL_axis_transformation;              //define transformation matrix

  PCL_axis_transformation  <<   0, -1, 0, 0,             //rigid transformation adapt for the visualization of the bumblebee camera
                           0, 0, 1, 0,
                           -1, 0, 0, 0,
                           0, 0, 0, 1;

  transformPointCloud(*image_cloud, *image_cloud, PCL_axis_transformation);

  pcl::VoxelGrid<pcl::PointXYZRGBA> ds;  //create downsampling filter
  PointCloud<PointXYZRGBA>::Ptr image_cloud_ds(new PointCloud<PointXYZRGBA>);
  ds.setInputCloud(image_cloud);
  ds.setLeafSize(0.2f, 0.2f, 0.2f);
  ds.filter(*image_cloud_ds);

  //cout << "Orginal cloud size" << image_cloud->size () <<
  //    " Downsampled cloud size = " << image_cloud_ds->size() << endl;
  *image_cloud = *image_cloud_ds;
  image_cloud_ds->clear();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud(image_cloud);
  sor.setMeanK(100);
  sor.setStddevMulThresh(0.5);
  sor.filter(*image_cloud_ds);
  //cout << "Orginal cloud size" << image_cloud->size () <<
  //    " SOR cloud size = " << image_cloud_ds->size() << endl;
  *image_cloud = *image_cloud_ds;
  image_cloud_ds->clear();

  /*
       PCL_axis_transformation  <<   0, -1, 0, 0,             //rigid transformation adapt for the visualization of the bumblebee camera
                                     0, 0, 1, 0,
                                    -1, 0, 0, 0,
                                     0, 0, 0, 1;

       PCL_axis_transformation  <<   1, 0, 0, 0,             //rigid transformation adapt for the visualization of the kinect camera
                                     0, -1, 0, 0,
                                     0, 0, -1, 0,
                                     0, 0, 0, 1;
   */
  /**/

  //*image_cloud=surf_creator();

  // Estimate normals
  float normalRadiusSearch = 0.4;   // is the radius of the round in meter 1.0 for outside acquisition, 0.3 is good for the kinect sensor acquisition

  NormalEstimation<PointXYZRGBA, Normal> ne;
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
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////
  /*        TRAVERSABILITY ANALYSIS        */

  PointXYZRGBA searchPoint;
  searchPoint.x = image_cloud->points[1].x;
  searchPoint.y = image_cloud->points[1].y;
  searchPoint.z = image_cloud->points[1].z;

  KdTreeFLANN<PointXYZRGBA> kdtree;
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
  /**/
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
  }/**/


  r_cloud->height = image_cloud->height;
  r_cloud->width = image_cloud->width;

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


  return (0);
}

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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////Rainbow color creator///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t UPD::GiveRainbowColor(float position)

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




/************************************************************************************************
 ******************************************* VIEWER PROGRAM *************************************
 ************************************************************************************************/


void
UPD::viewerOneOff(visualization::PCLVisualizer& viewer)
{
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 1.0, 0.5, v1);
  viewer.setBackgroundColor(1.0, 1.0, 1.0, v1);
  visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> rgb1(image_cloud);
  viewer.addPointCloud<pcl::PointXYZRGBA> (image_cloud, rgb1, "cloud", v1);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

  int v2(0);
  viewer.createViewPort(0.0, 0.5, 1.0, 1.0, v2);
  viewer.setBackgroundColor(1.0, 1.0, 1.0, v2);
  visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> rgb2(colored_cloud);
  viewer.addPointCloud<pcl::PointXYZRGBA> (colored_cloud, rgb2, "C-cloud", v2);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "C-cloud");
  cout << "i only run once" << endl;
}

/************************************************************************************************
 ******************************************* main *************************************
 ************************************************************************************************/

void
UPD::viewerPsycho(visualization::PCLVisualizer& viewer)
{
  static unsigned count = 0;
  stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);

  //FIXME: possible race condition here:
  user_data++;
}

int  UPD::run(int argc, char** argv)
{
  //system ("clear");   //clear the shell screen

  string file_name;
  struct timespec start, end;
  uint64_t timeElapsed;


  if (console::find_argument(argc, argv, "-m") >= 0)
  {
    ifstream readSavedFile;
    if (argc > 2)
    {
      readSavedFile.open(argv[2]);
      cout << "File list read ! " << endl;
    }
    else
    {
      console::print_error(" Attention File list not read !!!! \n\n");
    }
    char SavedFileName[100];
    struct timespec start, end;

    clock_gettime(CLOCK_MONOTONIC, &start);

    if (readSavedFile.is_open())
    {
      int first_file_flag = 0;
      visualization::PCLVisualizer viewer("viewer");

      do
      {
        if (first_file_flag == 0)
        {
          readSavedFile >> file_name;//pcd_input_filename;

          if (io::loadPCDFile(file_name, *image_cloud) == true)
            cout << "Loaded " << image_cloud->size() << " data points from " << file_name << endl;

          cout << "Read = " << file_name << endl;//pcd_input_filename << std::endl;
          upd();
          first_file_flag = 1;
          //////////////////////////inserire salvataggio!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }
        else
        {
          if (!viewer.updatePointCloud(image_cloud, "cloud"))
          {
            viewerOneOff(viewer);
          }

          if (!viewer.updatePointCloud(colored_cloud, "C-cloud"))
          {
            viewerPsycho(viewer);
          }

          viewer.spinOnce(1);

          readSavedFile >> file_name;//pcd_input_filename;
          if (io::loadPCDFile(file_name, *image_cloud) == true)
            cout << "Loaded " << image_cloud->size() << " data points from " << file_name << endl;
          cout << "Read = " << file_name << endl;//pcd_input_filename << std::endl;
          upd();

        }
      }
      while (!readSavedFile.eof());

      clock_gettime(CLOCK_MONOTONIC, &end);
      timeElapsed = timespecDiff(&end, &start);
      timeElapsed = timeElapsed * 0.000001;  //time is converted in msec
      cout << "Completed --- time elapsed ==> " << timeElapsed << " ms    CIAOOO!!!!\n" << endl;
    }
    return 0;
  }

  if (console::find_argument(argc, argv, "") <= 0)
  {
    cout << "\n\E[00;31m\033[1mArgument not recognized ... \033[0m\n\n" << endl;
    cout << "\E[00;34m\33[1m-m ---> Normal analysis on a file list\n";
    cout << "-n ---> Normal analysis test \n\033[0m" << endl;
    return 0;
  }
  return 0;
}

}
}

