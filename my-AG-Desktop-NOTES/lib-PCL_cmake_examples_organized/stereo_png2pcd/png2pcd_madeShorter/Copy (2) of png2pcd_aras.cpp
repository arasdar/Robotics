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

static void print_help()
{
    printf("\n png2pcd converting png images into pcd images \n");
    printf("\nUsage: ./png2pcd <input_png_image_dir> <output_pcd_image_dir> <output_pcd_image_filename> \n");
}


int
main (int argc, char** argv)
{

	// reading input dir and help
	if(argc < 3 || argc < 2 || argc < 4)
	{
		print_help();
		return 0;
	}
	int img_number_left = 0, img_number_right = 0 ;
	int img_index = 0;
	// Get list of input png images left
	vector<string> left_images;
	boost::filesystem::directory_iterator end_itr;
	for (boost::filesystem::directory_iterator itr (argv[1]); itr != end_itr; ++itr)
	{
		left_images.push_back (itr->path ().string ());
		img_number_left++;
	}
	sort (left_images.begin (), left_images.end ());


  for (;;)
  {

		  // showing the input images number and exit criteria
		cout << "img_index:" << img_index << "  img_number: " << img_number_left << endl;
		if ( img_index == img_number_left){
			cout << "end............................................." << endl;
			break;
		}
		// showing the input images
		cout << "left_images[img_index]: " << left_images[img_index] << std::endl;
		//variable initialization
		const char* img1_filename = (char*) left_images[img_index].c_str();


	  // Command line parsing
	  bool format = false;
	  std::string mode = "FORCE_COLOR";
	  print_info ("%s mode selected.\n", mode.c_str ());

	  // Load the input file
	  vtkSmartPointer<vtkImageData> image_data;
	  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
	  reader->SetFileName (img1_filename);
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

		//save output pcd file
	    std::string dir_name_ = argv [2];
	    std::string file_name_ = argv[3];
		std::stringstream ss;
		ss << dir_name_ << "/" << file_name_ << "_writeBinary_" << img_index << ".pcd";
	    pcl::PCDWriter writer_;
	    writer_.writeBinary<RGB> (ss.str (), cloud);
	   cout << "output pcd file " << ss.str () << std::endl;

	  }

	  delete[] pixel;

	  img_index++;

  }

  return 0;
}
