#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_lib_io.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

/* ---[ */
int
main (int argc, char** argv)
{

//  // Parse the command line arguments for .vtk and .ply files
//  std::vector<int> png_file_indices = parse_file_extension_argument (argc, argv, ".png");
//  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
//  std::vector<int> pcd_file_indices_cloud_xyzrgba = parse_file_extension_argument (argc, argv, ".pcd");


	/////reading the input
////	char* filename_input_cloud_xyzrgba = argv[pcd_file_indices_cloud_xyzrgba[0]];
//	PointCloud<PointXYZRGBA>::Ptr cloud_xyzrgba (new PointCloud<PointXYZRGBA>);
//	loadPCDFile (argv[3], *cloud_xyzrgba); //the third argument in command line
//	   size_t i = 0;

	   PointCloud<PointXYZRGBA> cloud_xyzrgba;
		loadPCDFile (argv[3], cloud_xyzrgba); //the third argument in command line




	// Command line parsing
	bool format = false;
	std::string mode = "FORCE_COLOR";
	print_info ("%s mode selected.\n", mode.c_str ());


  // Load the input file
  vtkSmartPointer<vtkImageData> image_data;
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New ();
  reader->SetFileName (argv[1]); //the first argument in command line
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

//    PointCloud<XYZRGB> cloud_XYZRGB;


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
//          cloud_xyzrgba->points[i].rgba = color.rgba;
//          ++i;
          cloud_xyzrgba(x, dimensions[1] - y - 1).rgba = color.rgba;


        }
      }
    }

    // Save the point cloud into a PCD file
    std::string filename = argv[2];
    print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
    savePCDFile ( filename, cloud, format);
    savePCDFile ( argv[4], cloud_xyzrgba, format);
//    savePCDFile ( argv[4], *cloud_xyzrgba, format);
    TicToc tt;
    tt.tic ();
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");

  }

  delete[] pixel;

  return 0;
}
