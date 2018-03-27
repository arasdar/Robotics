/*=========================================================================

  Program:   ORFEO Toolbox
  Language:  C++
  Date:      $Date$
  Version:   $Revision$


  Copyright (c) Centre National d'Etudes Spatiales. All rights reserved.
  See OTBCopyright.txt for details.

  Some parts of this code are derived from ITK. See ITKCopyright.txt
  for details.


     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/


#include <iostream>
using namespace std;

#include <itkVectorGradientAnisotropicDiffusionImageFilter.h>
#include <itkVectorGradientMagnitudeImageFilter.h>
#include <itkWatershedImageFilter.h>

#include <otbImage.h>
#include <otbImageFileReader.h>
#include <otbImageFileWriter.h>
#include <itkVectorCastImageFilter.h>
#include <itkUnaryFunctorImageFilter.h>
#include <itkScalarToRGBPixelFunctor.h>

int main(int argc, char *argv[])
{
  if (argc < 7)
  {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr <<
              " inputImage conductanceTerm diffusionIterations lowerThreshold outputScaleLevel gradientMode "
              << std::endl;
    return 1;
  }
  string  inName            = argv[1];
  int fileNameLen = inName.rfind("/") != string::npos ? inName.rfind(".") - inName.rfind("/") - 1 : inName.rfind(".") - 1;
  int fileNameStart = inName.rfind("/") != string::npos ? inName.rfind("/") + 1 : 0;
  std::ostringstream ostr;
  ostr << inName.substr(fileNameStart, fileNameLen)  << "_watershed_" << argv[2] << "_" << argv[3] << "_" << argv[4]
       << "_" << argv[5] << "_" << argv[6] << ".jpg";
  string  outName      = ostr.str();

  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef otb::Image<RGBPixelType, 2>    RGBImageType;
  typedef itk::Vector<float, 3>          VectorPixelType;
  typedef itk::Image<VectorPixelType, 2> VectorImageType;
  typedef itk::Image<unsigned long, 2>   LabeledImageType;
  typedef itk::Image<float, 2>           ScalarImageType;

  typedef otb::ImageFileReader<RGBImageType> FileReaderType;
  typedef itk::VectorCastImageFilter<RGBImageType, VectorImageType>
  CastFilterType;
  typedef itk::VectorGradientAnisotropicDiffusionImageFilter < VectorImageType,
          VectorImageType >
          DiffusionFilterType;
  typedef itk::VectorGradientMagnitudeImageFilter < VectorImageType, float,
          ScalarImageType >
          GradientMagnitudeFilterType;
  typedef itk::WatershedImageFilter<ScalarImageType> WatershedFilterType;


  typedef otb::ImageFileWriter<RGBImageType> FileWriterType;

  FileReaderType::Pointer reader = FileReaderType::New();
  reader->SetFileName(argv[1]);

  CastFilterType::Pointer caster = CastFilterType::New();


  DiffusionFilterType::Pointer diffusion = DiffusionFilterType::New();
  diffusion->SetNumberOfIterations(atoi(argv[3]));
  diffusion->SetConductanceParameter(atof(argv[2]));
  diffusion->SetTimeStep(0.125);
  diffusion->SetUseImageSpacing(false);

  GradientMagnitudeFilterType::Pointer
  gradient = GradientMagnitudeFilterType::New();
  gradient->SetUsePrincipleComponents(atoi(argv[6]));
  gradient->SetUseImageSpacingOff();

  WatershedFilterType::Pointer watershed = WatershedFilterType::New();
  watershed->SetLevel(atof(argv[5]));
  watershed->SetThreshold(atof(argv[4]));

  typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long>
  ColorMapFunctorType;
  typedef itk::UnaryFunctorImageFilter < LabeledImageType,
          RGBImageType,
          ColorMapFunctorType > ColorMapFilterType;
  ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();

  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(outName);

  caster->SetInput(reader->GetOutput());
  diffusion->SetInput(caster->GetOutput());
  gradient->SetInput(diffusion->GetOutput());
  watershed->SetInput(gradient->GetOutput());
  colormapper->SetInput(watershed->GetOutput());
  writer->SetInput(colormapper->GetOutput());

  try
  {
    writer->Update();
  }
  catch (itk::ExceptionObject& e)
  {
    std::cerr << e << std::endl;
  }

  return EXIT_SUCCESS;
}

