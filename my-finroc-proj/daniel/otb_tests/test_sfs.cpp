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
#include <itkMacro.h>

#include <otbImage.h>
#include <otbImageFileReader.h>
#include <otbImageFileWriter.h>
#include <itkRescaleIntensityImageFilter.h>


#include <otbSFSTexturesImageFilter.h>
// Software Guide : EndCodeSnippet

using namespace std;
int main(int argc, char * argv[])
{
  typedef double PixelType;
  const unsigned int Dimension = 2;
  if (argc != 7)
  {
    std::cerr << "Usage: " << argv[0] << " <inputImage>";
    std::cerr << " <spectralThreshold> <spatialThreshold> <dirNb> <maxConsideration> <alpha> ";
    std::cerr << std::endl;
    return EXIT_FAILURE;
  }

  string  inName            = argv[1];
  PixelType    spectThresh       = atof(argv[2]);
  unsigned int spatialThresh     = atoi(argv[3]);
  unsigned int dirNb             = atoi(argv[4]);
  unsigned int maxConsideration  = atoi(argv[5]);
  double       alpha             = atof(argv[6]);
  std::ostringstream ostr;
  ostr << inName.substr(0, inName.rfind("."))  << "_sfs_" << spectThresh << "_" << spatialThresh << "_" << dirNb
       << "_" << maxConsideration << "_" << alpha;
  string  outName_base      = ostr.str();
  string  outNameLength     = outName_base + "_length.jpg";
  string  outNameWidth      = outName_base + "_width.jpg";
  string  outNameWMean      = outName_base + "_wmean.jpg";
  string  outNameRatio      = outName_base + "_ratio.jpg";
  string  outNameSD         = outName_base + "_sd.jpg";
  string  outNamePsi        = outName_base + "_psi.jpg";



  typedef otb::Image<PixelType, Dimension> ImageType;
  typedef otb::ImageFileReader<ImageType>  ReaderType;
  typedef otb::ImageFileWriter<ImageType>  WriterType;
  typedef otb::SFSTexturesImageFilter<ImageType, ImageType> SFSFilterType;


  SFSFilterType::Pointer filter    = SFSFilterType::New();
  ReaderType::Pointer    reader       = ReaderType::New();

  reader->SetFileName(inName);


  filter->SetSpectralThreshold(spectThresh);
  filter->SetSpatialThreshold(spatialThresh);
  filter->SetNumberOfDirections(dirNb);
  filter->SetRatioMaxConsiderationNumber(maxConsideration);
  filter->SetAlpha(alpha);

  filter->SetFeatureStatus(SFSFilterType::PSI, true);

  filter->SetInput(reader->GetOutput());

  /*
  WriterType::Pointer    writerLength = WriterType::New();
  WriterType::Pointer    writerWidth  = WriterType::New();
  WriterType::Pointer    writerWMean  = WriterType::New();
  WriterType::Pointer    writerRatio  = WriterType::New();
  WriterType::Pointer    writerSD     = WriterType::New();
  WriterType::Pointer    writerPsi    = WriterType::New();


  writerLength->SetFileName(outNameLength);
  writerLength->SetInput(filter->GetLengthOutput());
  writerLength->Update();

  writerWidth->SetFileName(outNameWidth);
  writerWidth->SetInput(filter->GetWidthOutput());
  writerWidth->Update();

  writerWMean->SetFileName(outNameWMean);
  writerWMean->SetInput(filter->GetWMeanOutput());
  writerWMean->Update();

  writerRatio->SetFileName(outNameRatio);
  writerRatio->SetInput(filter->GetRatioOutput());
  writerRatio->Update();

  writerSD->SetFileName(outNameSD);
  writerSD->SetInput(filter->GetSDOutput());
  writerSD->Update();

  writerPsi->SetFileName(outNamePsi);
  writerPsi->SetInput(filter->GetPSIOutput());
  writerPsi->Update();

  */

  /************** pretty images for printing *********/
  typedef otb::Image < unsigned char,
          2 >                                     OutputImageType;
  typedef itk::RescaleIntensityImageFilter < ImageType,
          OutputImageType > RescalerType;
  typedef otb::ImageFileWriter<OutputImageType>
  OutputWriterType;

  RescalerType::Pointer rescaler = RescalerType::New();
  rescaler->SetOutputMinimum(0);
  rescaler->SetOutputMaximum(255);

  OutputWriterType::Pointer outWriter = OutputWriterType::New();
  outWriter->SetInput(rescaler->GetOutput());

  rescaler->SetInput(filter->GetLengthOutput());
  outWriter->SetFileName(outNameLength);
  outWriter->Update();

  rescaler->SetInput(filter->GetWidthOutput());
  outWriter->SetFileName(outNameWidth);
  outWriter->Update();

  rescaler->SetInput(filter->GetWMeanOutput());
  outWriter->SetFileName(outNameWMean);
  outWriter->Update();

  rescaler->SetInput(filter->GetRatioOutput());
  outWriter->SetFileName(outNameRatio);
  outWriter->Update();

  rescaler->SetInput(filter->GetSDOutput());
  outWriter->SetFileName(outNameSD);
  outWriter->Update();

  rescaler->SetInput(filter->GetPSIOutput());
  outWriter->SetFileName(outNamePsi);
  outWriter->Update();

  return EXIT_SUCCESS;
}
