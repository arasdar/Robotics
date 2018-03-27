
#ifndef __projects__stereo_traversability_experiments__daniel__segmentation_with_texture__texture__tTextureExtractor_h__
#define __projects__stereo_traversability_experiments__daniel__segmentation_with_texture__texture__tTextureExtractor_h__


#include <itkMacro.h>
#include <itkRescaleIntensityImageFilter.h>
#include <otbImage.h>
#include <otbImageFileReader.h>
#include <otbImageFileWriter.h>


// Pretty RGB output
#include <otbVectorImage.h>
#include <otbImageToVectorImageCastFilter.h>
#include <otbVectorRescaleIntensityImageFilter.h>


#include <otbScalarImageToTexturesFilter.h>
#include <itkImageRegionIteratorWithIndex.h>

//pcl includes
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>


const int dims_haralick = 8;

// pcl point type declaration
struct HaralickPointType
{
  double dim[dims_haralick];
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW;   // make sure our new allocators are aligned
};// EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(HaralickPointType,            // here we assume a XYZ + "test" (as fields)
                                  (double[dims_haralick], dim, dim)
                                 );


namespace finroc
{
namespace stereo_traversability_experiments
{
namespace daniel
{
namespace segmentation_with_texture
{
namespace texture
{

// pcl type definitions
typedef pcl::PointCloud<HaralickPointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef double PixelType;
const int Dimension = 2;
typedef otb::Image<PixelType, Dimension> ImageType;


typedef otb::ScalarImageToTexturesFilter
<ImageType, ImageType> TexturesFilterType;
typedef itk::ImageRegionIteratorWithIndex<ImageType> IteratorType;

typedef otb::ImageFileReader<ImageType> ReaderType;
typedef otb::ImageFileWriter<ImageType> WriterType;

class tTextureExtractor
{

public:

  tTextureExtractor();
  ~tTextureExtractor();

  CloudPtr compute_texture(std::string image, unsigned int radius, unsigned int xOffset, unsigned int yOffset);

private:

  /*Here is the right place for your variables. Replace this line by your declarations!*/
  //boost::shared_ptr<visualization::PCLVisualizer> viewer;

};




}
}
}
}
}

#endif
