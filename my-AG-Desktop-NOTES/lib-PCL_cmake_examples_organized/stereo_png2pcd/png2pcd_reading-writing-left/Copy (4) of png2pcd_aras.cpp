#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>
#include <opencv2/highgui/highgui.hpp>
/////////debugging
#include <iostream>
#include <vector>
#include <stdio.h>
////////namespace
using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*                      PNG 2 PCD CONVERTER - Usage Guide                  *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << argv[0] << " [Options] input.png output.pcd" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                                               Show this help." << std::endl;
  std::cout << "     -format 0 | 1:                                    Set the format of the output pcd file." << std::endl;
  std::cout << "     -mode DEFAULT | FORCE_COLOR | FORCE_GRAYSCALE:    Set the working mode of the converter." << std::endl;
  std::cout << "       --intensity_type: FLOAT | UINT8_T               Set the desired intensity type" << std::endl;
}

template<typename PointInT> void
saveCloud (const std::string &filename, const PointCloud<PointInT> &cloud, bool format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  savePCDFile (filename, cloud, format);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

int
main (int argc, char** argv)
{

  // Get list of input png images
  vector<string> left_images;
  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (argv[1]); itr != end_itr; ++itr)
  {
	left_images.push_back (itr->path ().string ());
  }
  sort (left_images.begin (), left_images.end ());

  int img_index = 0;

  for (;;)
  {

	   string filename = left_images[img_index];
	   char *fileName = (char*)filename.c_str();
	   cout << "input png image : " << fileName << endl;

	  // Parse the command line arguments for .vtk and .ply files
	//  std::vector<int> png_file_indices = parse_file_extension_argument (argc, argv, ".png");
	  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");

	//  if (png_file_indices.size () != 1 || pcd_file_indices.size () != 1)
	//  {
	//    print_error ("Need one input PNG file and one output PCD file.\n");
	//    return (-1);
	//  }

	  // Command line parsing
	  bool format = false;
	  std::string mode = "FORCE_COLOR";
	  print_info ("%s mode selected.\n", mode.c_str ());

	  // Load the input file
	  vtkSmartPointer<vtkImageData> image_data;
	  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
	  reader->SetFileName (fileName);
	  image_data = reader->GetOutput ();
	  image_data->Update ();

	  // Retrieve the entries from the image data and copy them into the output RGB cloud
	  int components = image_data->GetNumberOfScalarComponents();

	  double* pixel = new double [4];
	  memset (pixel, 0, sizeof(double) * 4);

	  if (mode.compare ("FORCE_COLOR") == 0)
	  {
		//
		// Force the output cloud to be a pcl::PointCloud<pcl::RGB> even if the input image is a
		// monochrome image.
		//

		PointCloud<RGB> cloud;
		int dimensions[3];
		image_data->GetDimensions (dimensions);
		cloud.width = dimensions[0];
		cloud.height = dimensions[1]; // This indicates that the point cloud is organized
		cloud.is_dense = true;
		cloud.points.resize (cloud.width * cloud.height);

		for (int z = 0; z < dimensions[2]; z++)
		{
		  for (int y = 0; y < dimensions[1]; y++)
		  {
			for (int x = 0; x < dimensions[0]; x++)
			{
			  for (int c = 0; c < components; c++)
				pixel[c] = image_data->GetScalarComponentAsDouble(x, y, 0, c);

			  RGB color;
			  color.r = 0;
			  color.g = 0;
			  color.b = 0;
			  color.a = 0;
			  color.rgb = 0.0f;
			  color.rgba = 0;

			  int rgb;
			  int rgba;
			  switch (components)
			  {
				case 1:  color.r = static_cast<uint8_t> (pixel[0]);
				color.g = static_cast<uint8_t> (pixel[0]);
				color.b = static_cast<uint8_t> (pixel[0]);

				rgb = (static_cast<int> (color.r)) << 16 |
					(static_cast<int> (color.g)) << 8 |
					(static_cast<int> (color.b));

				rgba = rgb;
				color.rgb = static_cast<float> (rgb);
				color.rgba = static_cast<uint32_t> (rgba);
				break;

				case 3:  color.r = static_cast<uint8_t> (pixel[0]);
				color.g = static_cast<uint8_t> (pixel[1]);
				color.b = static_cast<uint8_t> (pixel[2]);

				rgb = (static_cast<int> (color.r)) << 16 |
					(static_cast<int> (color.g)) << 8 |
					(static_cast<int> (color.b));

				rgba = rgb;
				color.rgb = static_cast<float> (rgb);
				color.rgba = static_cast<uint32_t> (rgba);
				break;

				case 4:  color.r = static_cast<uint8_t> (pixel[0]);
				color.g = static_cast<uint8_t> (pixel[1]);
				color.b = static_cast<uint8_t> (pixel[2]);
				color.a = static_cast<uint8_t> (pixel[3]);

				rgb = (static_cast<int> (color.r)) << 16 |
					(static_cast<int> (color.g)) << 8 |
					(static_cast<int> (color.b));
				rgba = (static_cast<int> (color.a)) << 24 |
					(static_cast<int> (color.r)) << 16 |
					(static_cast<int> (color.g)) << 8 |
					(static_cast<int> (color.b));

				color.rgb = static_cast<float> (rgb);
				color.rgba = static_cast<uint32_t> (rgba);
				break;
			  }

			  cloud (x, dimensions[1] - y - 1) = color;

			}
		  }
		}

		// Save the point cloud into a PCD file
		saveCloud<RGB> (argv[pcd_file_indices[0]], cloud, format);
	  }

	  delete[] pixel;

	  img_index++;

  }

  return 0;
}
