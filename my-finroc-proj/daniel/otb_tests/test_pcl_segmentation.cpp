#include <iostream>

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include "projects/stereo_traversability_experiments/daniel/otb_tests/segmentation_test/tGroundPlaneComparatorForTexture.h"
#include "projects/stereo_traversability_experiments/daniel/otb_tests/segmentation_test/tOrganizedConnectedComponentSegmentationForTesting.h"

const int dims_haralick = 8;


struct HaralickPointType
{
  double dim[dims_haralick];
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;   // make sure our new allocators are aligned
};// EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(HaralickPointType,            // here we assume a XYZ + "test" (as fields)
                                  (double[dims_haralick], dim, dim)
                                 );

typedef pcl::PointXYZRGB PointT;


using namespace finroc::stereo_traversability_experiments::daniel::otb_tests;

int
main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <pcd_file> ";
    std::cerr << std::endl;
    return EXIT_FAILURE;
  }

  const char* pcd_file_name   = argv[1];

  // load texture cloud
  pcl::PointCloud<HaralickPointType>::Ptr cloud(new pcl::PointCloud<HaralickPointType>);

  if (pcl::io::loadPCDFile<HaralickPointType> (std::string(pcd_file_name), *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from " << std::string(pcd_file_name)
            << std::endl;

  // normalize data; convert to Eigen vector, normalize, convert back
  std::cout << cloud->width << " " << cloud->height << std::endl;
  int ndims = sizeof(cloud->points[0].dim) / sizeof(cloud->points[0].dim[0]);
  for (int rowIdx = 0; rowIdx < (static_cast<int>(cloud->height)); rowIdx++)
  {
    for (int colIdx = 0; colIdx < (static_cast<int>(cloud->width)); colIdx++)
    {
      std::cout << rowIdx << " " << colIdx << std::endl;
      Eigen::VectorXf vec(ndims);
      for (int i = 0; i < ndims; i++)
      {
        vec[i] = cloud->at(colIdx, rowIdx).dim[i];
      }
      vec.normalize();
      for (int i = 0; i < ndims; i++)
      {
        cloud->at(colIdx, rowIdx).dim[i] = vec[i];
      }

    }
  }

  // save normalized cloud as pcd file on disk
  std::cerr << "Saving " << cloud->points.size() << " normalized data points." << std::endl;
  pcl::io::savePCDFileASCII(std::string(pcd_file_name) + "_normalized_pcd.pcd", *cloud);

  // IMPORTANT: in final pipeline, different types have to be used; PoinT instead of HaralickPointtype
  //libsegmentation::GroundPlaneComparatorForTexture<PointT, HaralickPointType>::Ptr  comp (new libsegmentation::GroundPlaneComparatorForTexture<PointT, HaralickPointType>());
  //libsegmentation::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> segm  = libsegmentation::OrganizedConnectedComponentSegmentation<PointT, pcl::Label>(comp);
  GroundPlaneComparatorForTexture<HaralickPointType, HaralickPointType>::Ptr  comp(new GroundPlaneComparatorForTexture<HaralickPointType, HaralickPointType>());
  OrganizedConnectedComponentSegmentationForTesting<HaralickPointType, pcl::Label> segm  = OrganizedConnectedComponentSegmentationForTesting<HaralickPointType, pcl::Label>(comp);


  float roughness = pcl::deg2rad(5.0f); //the tolerance in radians
  comp->setAngularThreshold(roughness); //3.0f original
  /*Set up the ground plane comparator*/
  comp->setInputTextures(cloud);
  /*Run segmentation*/
  pcl::PointCloud<pcl::Label> segment_labels;
  std::vector<pcl::PointIndices> segments;
  segm.setInputCloud(cloud);
  segm.segment(segment_labels, segments);



  return (0);

}
