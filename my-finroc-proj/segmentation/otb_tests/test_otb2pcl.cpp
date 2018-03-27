/*=========================================================================

  Program:   ORFEO Toolbox
  Language:  C++
  Date:      $Date$
  Version:   $Revision$


  Copyright (c) Centre National d'Etudes Spatiales. All rights reserved.
  See OTBCopyright.txt for details.


     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/


// otb includes

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




int main(int argc, char * argv[])
{
  // Parse command line parameters
  if (argc != 6)
  {
    std::cerr << "Usage: " << argv[0] << " <inputImage> ";
    std::cerr << " <outputRescaled (.tif will be appended)> ";
    std::cerr << " <radius> <xOffset> <yOffset> ";
    std::cerr << std::endl;
    return EXIT_FAILURE;
  }

  const char* infname   = argv[1];
  const char* outprettyfname  = argv[2];

  const unsigned int radius  =  static_cast<unsigned int>(atoi(argv[3]));
  const unsigned int xOffset =  static_cast<unsigned int>(atoi(argv[4]));
  const unsigned int yOffset =  static_cast<unsigned int>(atoi(argv[5]));

  typedef double PixelType;
  const int Dimension = 2;
  typedef otb::Image<PixelType, Dimension> ImageType;


  typedef otb::ScalarImageToTexturesFilter
  <ImageType, ImageType> TexturesFilterType;
  typedef itk::ImageRegionIteratorWithIndex<ImageType> IteratorType;

  typedef otb::ImageFileReader<ImageType> ReaderType;
  typedef otb::ImageFileWriter<ImageType> WriterType;


  // read in image, perform texture extraction

  ReaderType::Pointer reader  = ReaderType::New();
  ImageType::Pointer outputImage = ImageType::New();


  reader->SetFileName(infname);


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
  std::cout << "Image size:\nx: " << x_size << " y: " << y_size << " dims: " << dims << "\n";

  // save unprocessed otb images as txt file on disk
  PixelType pMax [dims_haralick];
  PixelType pMin [dims_haralick];
  std::cerr << "Saving " << dims_haralick << " unprocessed texture images as .txt files." << std::endl;
  IteratorType imageIts[dims_haralick];
  for (int d = 0; d < dims_haralick; d++)
  {
    imageIts[d] = IteratorType(otbHaralickImages[d], otbHaralickImages[d]->GetRequestedRegion());
    imageIts[d].GoToBegin();
    pMax[d] = pMin[d] = imageIts[d].Get();
  }
  std::ofstream otbImFile(std::string(outprettyfname) + "_otbData_raw.txt");

  for (imageIts[0].GoToBegin(); !imageIts[0].IsAtEnd();)
  {
    ImageType::IndexType idx = imageIts[0].GetIndex();
    otbImFile << "x: " << idx[0] << " y: " << idx[1] << " data:";
    for (int d = 0; d < dims_haralick; d++)
    {
      otbImFile << " " << imageIts[d].Get();
      if (imageIts[d].Get() > pMax[d]) pMax[d] = imageIts[d].Get();
      if (imageIts[d].Get() < pMin[d]) pMin[d] = imageIts[d].Get();
      ++imageIts[d];
    }
    otbImFile << std::endl;
  }
  otbImFile.close();

  for (int d = 0; d < dims_haralick; d++)
  {
    std::cout << "d: " << d << " min: " << pMin[d] << " max: " << pMax[d] << std::endl;
  }

  // normalize all single dimensions to [0..1]
  typedef itk::RescaleIntensityImageFilter <ImageType, ImageType> RescalerdType;


  for (int d = 0; d < dims_haralick; d++)
  {
    RescalerdType::Pointer imRescaler = RescalerdType::New();
    imRescaler->SetOutputMinimum(0.0);
    imRescaler->SetOutputMaximum(1.0);
    imRescaler->SetInput(otbHaralickImages[d]);
    imRescaler->Update();
    otbHaralickImages[d] = imRescaler->GetOutput();
  }


  // define nanImage for error visualization: white region -> valid value;
  //                      black region -> invalid (NaN)
  ImageType::Pointer nanImage = ImageType::New();
  ImageType::RegionType nanRegion;
  ImageType::RegionType::IndexType nanStart;
  ImageType::RegionType::SizeType  nanSize;
  nanStart[0] = 0;
  nanStart[1] = 0;
  nanSize[0] = x_size;
  nanSize[1] = y_size;
  nanRegion.SetSize(nanSize);
  nanRegion.SetIndex(nanStart);
  nanImage->SetRegions(nanRegion);
  nanImage->Allocate();

  // create empty pcl point cloud
  pcl::PointCloud<HaralickPointType> cloud;
  cloud.width    = x_size;
  cloud.height   = y_size;
  cloud.points.resize(x_size * y_size);
  cloud.is_dense = true; //all points are valid, no NaN

  int nan_count [] = {0, 0, 0, 0, 0, 0, 0, 0};
  // perform image representation transformation
  for (int d = 0; d < dims_haralick; d++)
  {
    IteratorType imageIt(otbHaralickImages[d], otbHaralickImages[d]->GetRequestedRegion());
    IteratorType nanImageIt(nanImage, otbHaralickImages[2]->GetRequestedRegion());
    for (imageIt.GoToBegin(), nanImageIt.GoToBegin(); !imageIt.IsAtEnd(); ++imageIt, ++nanImageIt)
    {
      ImageType::IndexType idx = imageIt.GetIndex();
      cloud.at(idx[0], idx[1]).dim[d] =  imageIt.Get();
      if (d == 2)
      {
        if (imageIt.Get() != imageIt.Get())
        {
          cloud.at(idx[0], idx[1]).dim[d] =  0; // replace NaN by 0, so we can apply vector computation
          nan_count[d] ++;
          nanImageIt.Set(0);
        }
        else
        {
          nanImageIt.Set(1);
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
                << " cloudVal[" << d << "]: " << cloud.at(testPixels_x[i], testPixels_y[i]).dim[d] << "\n";
    }
    std::cout << "\n";

  }
  std::cout << "\n";
  for (int d = 0; d < dims_haralick; d++)
  {
    std::cout << "image " << d << " NaN: " << nan_count[d] << "\n";
  }

  // save cloud as pcd file on disk
  std::cerr << "Saving " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
  pcl::io::savePCDFileASCII(std::string(outprettyfname) + "_pcd.pcd", cloud);

  // save normalized otb images as txt file on disk
  std::cerr << "Saving " << dims_haralick << " normalized texture images as .txt files." << std::endl;
  for (int d = 0; d < dims_haralick; d++)
  {
    imageIts[d] = IteratorType(otbHaralickImages[d], otbHaralickImages[d]->GetRequestedRegion());
    imageIts[d].GoToBegin();
  }
  std::ofstream otbImFile2(std::string(outprettyfname) + "_otbData_norm.txt");

  for (imageIts[0].GoToBegin(); !imageIts[0].IsAtEnd();)
  {
    ImageType::IndexType idx = imageIts[0].GetIndex();
    otbImFile2 << "x: " << idx[0] << " y: " << idx[1] << " data:";
    for (int d = 0; d < dims_haralick; d++)
    {
      otbImFile2 << " " << imageIts[d].Get();
      ++imageIts[d];
    }
    otbImFile2 << std::endl;
  }
  otbImFile2.close();

  /************** pretty images for printing *********/
  std::cerr << "Saving " << dims_haralick << " texture images." << std::endl;
  typedef otb::Image <unsigned char, 2>     OutputImageType;
  typedef itk::RescaleIntensityImageFilter <ImageType, OutputImageType> RescalerType;
  typedef otb::ImageFileWriter<OutputImageType> OutputWriterType;

  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetOutputMinimum(0);
  rescaler->SetOutputMaximum(255);

  OutputWriterType::Pointer outWriter = OutputWriterType::New();

  for (int d = 0; d < dims_haralick; d++)
  {
    rescaler->SetInput(otbHaralickImages[d]);
    rescaler->Update(); // intermediate update avoids tiles in output due to streamed processing
    outWriter->SetFileName(std::string(outprettyfname) + "_" + std::to_string(d) + ".tif");
    outWriter->SetInput(rescaler->GetOutput());
    outWriter->Update();
  }
  rescaler->SetInput(nanImage);
  rescaler->Update(); // intermediate update avoids tiles in output due to streamed processing
  outWriter->SetFileName(std::string(outprettyfname) + "_nan.tif");
  outWriter->SetInput(rescaler->GetOutput());
  outWriter->Update();

  return EXIT_SUCCESS;
}
