#include "projects/stereo_traversability_experiments/daniel/segmentation_with_texture/texture/tTextureExtractor.h"



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

tTextureExtractor::tTextureExtractor()
{}
tTextureExtractor::~tTextureExtractor()
{}

CloudPtr tTextureExtractor::compute_texture(std::string imageName, unsigned int radius, unsigned int xOffset, unsigned int yOffset)
{
  // read in image, perform texture extraction
  ReaderType::Pointer reader  = ReaderType::New();
  ImageType::Pointer outputImage = ImageType::New();


  reader->SetFileName(imageName);


  TexturesFilterType::Pointer texturesFilter
    = TexturesFilterType::New();

  typedef ImageType::SizeType SizeType;
  SizeType sradius;
  sradius.Fill(radius);

  texturesFilter->SetRadius(sradius);

  typedef ImageType::OffsetType OffsetType;
  OffsetType offset;
  offset[0] =  xOffset;
  offset[1] =  yOffset;

  texturesFilter->SetOffset(offset);

  texturesFilter->SetInputImageMinimum(0);
  texturesFilter->SetInputImageMaximum(255);


  texturesFilter->SetInput(reader->GetOutput());
  texturesFilter->Update();

  // define image objects for operations with iterator
  ImageType::Pointer otbHaralickImages [dims_haralick];
  otbHaralickImages[0] = texturesFilter->GetEnergyOutput();
  otbHaralickImages[1] = texturesFilter->GetEntropyOutput();
  otbHaralickImages[2] = texturesFilter->GetCorrelationOutput();
  otbHaralickImages[3] = texturesFilter->GetInverseDifferenceMomentOutput();
  otbHaralickImages[4] = texturesFilter->GetInertiaOutput();
  otbHaralickImages[5] = texturesFilter->GetClusterShadeOutput();
  otbHaralickImages[6] = texturesFilter->GetClusterProminenceOutput();
  otbHaralickImages[7] = texturesFilter->GetHaralickCorrelationOutput();


  ImageType::RegionType largestRegion = otbHaralickImages[0]->GetRequestedRegion(); //maybe also GetLargestPossibleRegion(), but it works
  unsigned int x_size = largestRegion.GetSize()[0];
  unsigned int y_size = largestRegion.GetSize()[1];
  unsigned int dims = otbHaralickImages[0]->GetNumberOfComponentsPerPixel();
  //std::cout <<"x_size: " << x_size << " y_size: " << y_size << " dims: " << dims << std::endl;


  // normalize all single dimensions to [0..1]
  typedef itk::RescaleIntensityImageFilter <ImageType, ImageType> RescalerType;


  for (int d = 0; d < dims_haralick; d++)
  {
    RescalerType::Pointer imRescaler = RescalerType::New();
    imRescaler->SetOutputMinimum(0.0);
    imRescaler->SetOutputMaximum(1.0);
    imRescaler->SetInput(otbHaralickImages[d]);
    imRescaler->Update();
    otbHaralickImages[d] = imRescaler->GetOutput();
  }

  // create empty pcl point cloud
  CloudPtr cloud(new Cloud);

  cloud->width    = x_size;
  cloud->height   = y_size;
  cloud->points.resize(x_size * y_size);
  cloud->is_dense = true; //all points are valid, no NaN

  for (int d = 0; d < dims_haralick; d++)
  {
    IteratorType imageIt(otbHaralickImages[d], otbHaralickImages[d]->GetRequestedRegion());
    for (imageIt.GoToBegin(); !imageIt.IsAtEnd(); ++imageIt)
    {
      ImageType::IndexType idx = imageIt.GetIndex();
      cloud->at(idx[0], idx[1]).dim[d] =  imageIt.Get();
      if (d == 2)
      {
        if (imageIt.Get() != imageIt.Get())
        {
          cloud->at(idx[0], idx[1]).dim[d] =  0; // replace NaN by 0, so we can apply vector computation
        }
      }
    }
  }

  // check content

  int testPixels_x [] = {20, 50, 100, 200};
  int testPixels_y [] = {20, 50, 100, 200};
  for (int i = 0; i < 4; i++) // (sizeof(a)/sizeof(*a))
  {
    ImageType::RegionType::IndexType pixel;
    pixel[0] = testPixels_x[i];
    pixel[1] = testPixels_y[i];
    std::cout << " x: " << testPixels_x[i] << " y: " << testPixels_y[i] << "\n";
    for (int d = 0; d < dims_haralick; d++)
    {
      std::cout << "imageVal[" << d << "]: " << otbHaralickImages[d]->GetPixel(pixel)
                << " cloudVal[" << d << "]: " << cloud->at(testPixels_x[i], testPixels_y[i]).dim[d] << "\n";
    }
    std::cout << "\n";

  }
  std::cout << "\n";

  // normalize data; convert to Eigen vector, normalize, convert back
  int ndims = sizeof(cloud->points[0].dim) / sizeof(cloud->points[0].dim[0]);
  for (int rowIdx = 0; rowIdx < (static_cast<int>(cloud->height)); rowIdx++)
  {
    for (int colIdx = 0; colIdx < (static_cast<int>(cloud->width)); colIdx++)
    {
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

  return cloud;
}



}
}
}
}
}
