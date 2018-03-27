
#include <iostream> // cout , cin
#include <boost/filesystem.hpp> // pretty obvious
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/highgui/highgui.hpp> // imread and imshow ...
#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace boost::filesystem;
using namespace std;
using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::read_directory_recursively(/*INPUT*/int argc, /*INPUT*/const char** argv, /*OUTPUT*/std::vector<cv::Mat>& all_image_data_in_directory)
{

  //How do I iterate over every file, directory, and sub-directory and store the file names in a vector in C++?
  //boost filesystem example for iterate directory and sub-directories
  std::string dir_path(argv[1]);

  for (auto it : recursive_directory_range(dir_path))
  {
    //    std::cout << it << std::endl;

    path p(it);    // p reads clearer than argv[1] in the following code

    if (exists(p))    // does p actually exist?
    {
      if (is_regular_file(p))        // is p a regular file?
      {
        //cout << p << " size is " << file_size(p) << '\n';

        if (p.has_extension() && p.has_stem() && p.has_filename())
        {
          //cout  <<  "  has_filename()-------: " <<  (p.has_filename()) << '\n';
          //cout  <<  "  has_stem()-----------: " <<  (p.has_stem()) << '\n';
          //cout  <<  "  has_extension()------: " <<  (p.has_extension()) << '\n';

          // This is for reading the downloaded databases of different data data types such as IMAGES(img) or point clouds(pcd)
          if (p.extension() == ".pgm" ||
              p.extension() == ".png" ||
              p.extension() == ".jpg" ||
              p.extension() == ".BMP" ||
              p.extension() == ".ppm" ||
              p.extension() == ".PPM") // TODO: lower cases and upper cases
          {
            cout  <<  "  relative_path()------: " << p.relative_path() << '\n';
            cout  <<  "  parent_path()--------: " << p.parent_path() << '\n';
            cout  <<  "  filename()-----------: " << p.filename() << '\n';
            cout  <<  "  stem()---------------: " << p.stem() << '\n';
            cout  <<  "  extension()----------: " << p.extension() << '\n';

            cout  <<  "  has_relative_path()--: " << (p.has_relative_path()) << '\n';
            cout  <<  "  has_parent_path()----: " << (p.has_parent_path()) << '\n';
            cout  <<  "  has_filename()-------: " << (p.has_filename()) << '\n';
            cout  <<  "  has_stem()-----------: " << (p.has_stem()) << '\n';
            cout  <<  "  has_extension()------: " << (p.has_extension()) << '\n';


            if (!p.empty())  // if the XML file is not empty
            {

              // now let s open the IMG file and look into it
              //read
              const string IMG_filepath(p.string());  // it has to be CONSTANT in one iteration
              cout << "IMG_filepath: " << IMG_filepath << endl;

              // let s show the image
              const string camera_image_filepath = IMG_filepath; //p.parent_path().string() + "/" + camera_image_filename;
              const path camera_image_path(camera_image_filepath);
              cout << "camera_image_filepath: " << camera_image_filepath << endl;
              if (exists(camera_image_path)    // does p actually exist?
                  && (is_regular_file(camera_image_path))        // is p a regular file?
                  && (!camera_image_path.empty())  // if the XML file is not empty
                 )

              {
                const cv::Mat camera_image = cv::imread(camera_image_filepath/*, DEFAULT IST color*/);
                if (camera_image.size().width > 0 && camera_image.size().height > 0)
                {
                  /*const */string window_name = "camera_image"; //camera_image_filename + "___" + time_stamp_string_label + "___" + cv::format("_%d", data_index);
                  cv::namedWindow(window_name, CV_WINDOW_NORMAL);
                  imshow(window_name, camera_image);

                  const cv::Mat camera_image_grayscale = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_GRAYSCALE);
                  window_name = "camera_image_GRAYSCALE";
                  cv::namedWindow(window_name, WINDOW_NORMAL);
                  cv::imshow(window_name, camera_image_grayscale);

                  cv::waitKey(1);

                  // loading the image data vector for learning
                  all_image_data_in_directory.push_back(camera_image_grayscale);
                  //cout << "all_image_data_in_directory.size(): " << all_image_data_in_directory.size() << endl;

                } // if img not empty
              } // if img exist
            } // if imgpath not empty
          } // if img or jpg, png, pgm extension


        }// if has extension
      }// if regular file
    } // if exists

  }// for

  // the 1st one images
  cout << "all_image_data_in_directory.size(): " << all_image_data_in_directory.size() << endl;

}// end of function
