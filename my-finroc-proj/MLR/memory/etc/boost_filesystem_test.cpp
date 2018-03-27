/*
 * path_info.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: aras
 */



//  path_info.cpp  ---------------------------------------------------------------------//

//  Copyright Beman Dawes 2009

//  Distributed under the Boost Software License, Version 1.0.
//  See http://www.boost.org/LICENSE_1_0.txt

//  Library home page: http://www.boost.org/libs/filesystem

#include <iostream>
#include <boost/filesystem.hpp>
using namespace std;
using namespace boost::filesystem;

const char * say_what(bool b)
{
  return b ? "true" : "false";
}

int main_1(int argc, const char* argv[])
{
  if (argc < 2)
  {
    cout << "Usage: path_info path-portion...\n"
         "Example: path_info foo/bar baz\n"
#            ifdef BOOST_POSIX_API
         "         would report info about the composed path foo/bar/baz\n";
#            else  // BOOST_WINDOWS_API
         "         would report info about the composed path foo/bar\\baz\n";
#            endif
    return 1;
  }

  path p;  //  compose a path from the command line arguments

  for (; argc > 1; --argc, ++argv)
    p /= argv[1];

  cout  <<  "\ncomposed path:\n";
  cout  <<  "  cout << -------------: " << p << "\n";
  cout  <<  "  make_preferred()----------: " << path(p).make_preferred() << "\n";

  cout << "\nelements:\n";

  for (path::iterator it(p.begin()), it_end(p.end()); it != it_end; ++it)
    cout << "  " << *it << '\n';

  cout  <<  "\nobservers, native format:" << endl;
# ifdef BOOST_POSIX_API
  cout  <<  "  native()-------------: " << p.native() << endl;
  cout  <<  "  c_str()--------------: " << p.c_str() << endl;
# else  // BOOST_WINDOWS_API
  wcout << L"  native()-------------: " << p.native() << endl;
  wcout << L"  c_str()--------------: " << p.c_str() << endl;
# endif
  cout  <<  "  string()-------------: " << p.string() << endl;
  wcout << L"  wstring()------------: " << p.wstring() << endl;

  cout  <<  "\nobservers, generic format:\n";
  cout  <<  "  generic_string()-----: " << p.generic_string() << endl;
  wcout << L"  generic_wstring()----: " << p.generic_wstring() << endl;

  cout  <<  "\ndecomposition:\n";
  cout  <<  "  root_name()----------: " << p.root_name() << '\n';
  cout  <<  "  root_directory()-----: " << p.root_directory() << '\n';
  cout  <<  "  root_path()----------: " << p.root_path() << '\n';
  cout  <<  "  relative_path()------: " << p.relative_path() << '\n';
  cout  <<  "  parent_path()--------: " << p.parent_path() << '\n';
  cout  <<  "  filename()-----------: " << p.filename() << '\n';
  cout  <<  "  stem()---------------: " << p.stem() << '\n';
  cout  <<  "  extension()----------: " << p.extension() << '\n';

  cout  <<  "\nquery:\n";
  cout  <<  "  empty()--------------: " << say_what(p.empty()) << '\n';
  cout  <<  "  is_absolute()--------: " << say_what(p.is_absolute()) << '\n';
  cout  <<  "  has_root_name()------: " << say_what(p.has_root_name()) << '\n';
  cout  <<  "  has_root_directory()-: " << say_what(p.has_root_directory()) << '\n';
  cout  <<  "  has_root_path()------: " << say_what(p.has_root_path()) << '\n';
  cout  <<  "  has_relative_path()--: " << say_what(p.has_relative_path()) << '\n';
  cout  <<  "  has_parent_path()----: " << say_what(p.has_parent_path()) << '\n';
  cout  <<  "  has_filename()-------: " << say_what(p.has_filename()) << '\n';
  cout  <<  "  has_stem()-----------: " << say_what(p.has_stem()) << '\n';
  cout  <<  "  has_extension()------: " << say_what(p.has_extension()) << '\n';

  return 0;
}


//  filesystem tut4.cpp  ---------------------------------------------------------------//

//  Copyright Beman Dawes 2009

//  Distributed under the Boost Software License, Version 1.0.
//  See http://www.boost.org/LICENSE_1_0.txt

//  Library home page: http://www.boost.org/libs/filesystem

#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>
using namespace std;
using namespace boost::filesystem;

int main_2(int argc, const char* argv[])
{
  if (argc < 2)
  {
    cout << "Usage: tut4 path\n";
    return 1;
  }

  path p(argv[1]);    // p reads clearer than argv[1] in the following code

  try
  {
    if (exists(p))    // does p actually exist?
    {
      if (is_regular_file(p))        // is p a regular file?
        cout << p << " size is " << file_size(p) << '\n';

      else if (is_directory(p))      // is p a directory?
      {
        cout << p << " is a directory containing:\n";

        typedef vector<path> vec;             // store paths,
        vec v;                                // so we can sort them later

        copy(directory_iterator(p), directory_iterator(), back_inserter(v));

        sort(v.begin(), v.end());             // sort, since directory iteration
        // is not ordered on some file systems

        for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
        {
          cout << "   " << *it << '\n';
        }
      }
      else
        cout << p << " exists, but is neither a regular file nor a directory\n";
    }
    else
      cout << p << " does not exist\n";
  }

  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }

  return 0;
}

//  filesystem tut3.cpp  ---------------------------------------------------------------//

//  Copyright Beman Dawes 2009

//  Distributed under the Boost Software License, Version 1.0.
//  See http://www.boost.org/LICENSE_1_0.txt

//  Library home page: http://www.boost.org/libs/filesystem

#include <iostream>
#include <iterator>
#include <algorithm>
#include <boost/filesystem.hpp>
using namespace std;
using namespace boost::filesystem;

int main_3(int argc, const char* argv[])
{
  if (argc < 2)
  {
    cout << "Usage: tut3 path\n";
    return 1;
  }

  path p(argv[1]);    // p reads clearer than argv[1] in the following code

  try
  {
    if (exists(p))    // does p actually exist?
    {
      if (is_regular_file(p))        // is p a regular file?
        cout << p << " size is " << file_size(p) << '\n';

      else if (is_directory(p))      // is p a directory?
      {
        cout << p << " is a directory containing:\n";

        copy(directory_iterator(p), directory_iterator(),  // directory_iterator::value_type
             ostream_iterator<directory_entry>(cout, "\n"));  // is directory_entry, which is
        // converted to a path by the
        // path stream inserter
      }
      else
        cout << p << " exists, but is neither a regular file nor a directory\n";
    }
    else
      cout << p << " does not exist\n";
  }

  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }

  return 0;
}


//  filesystem tut2.cpp  ---------------------------------------------------------------//

//  Copyright Beman Dawes 2009

//  Distributed under the Boost Software License, Version 1.0.
//  See http://www.boost.org/LICENSE_1_0.txt

//  Library home page: http://www.boost.org/libs/filesystem

#include <iostream>
#include <boost/filesystem.hpp>
using namespace std;
using namespace boost::filesystem;

int main_4(int argc, const char* argv[])
{
  if (argc < 2)
  {
    cout << "Usage: tut2 path\n";
    return 1;
  }

  path p(argv[1]);    // p reads clearer than argv[1] in the following code

  if (exists(p))    // does p actually exist?
  {
    if (is_regular_file(p))        // is p a regular file?
      cout << p << " size is " << file_size(p) << '\n';

    else if (is_directory(p))      // is p a directory?
      cout << p << " is a directory\n";

    else
      cout << p << " exists, but is neither a regular file nor a directory\n";
  }
  else
    cout << p << " does not exist\n";

  return 0;
}



#include <iostream>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;

int main_5(int argc, const char* argv[])
{
  if (argc < 2)
  {
    std::cout << "Usage: tut1 path\n";
    return 1;
  }
  std::cout << argv[1] << " " << file_size(argv[1]) << '\n';
  return 0;
}

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace boost::filesystem;

struct recursive_directory_range
{
  typedef recursive_directory_iterator iterator;
  recursive_directory_range(path p) : p_(p) {}

  iterator begin()
  {
    return recursive_directory_iterator(p_);
  }
  iterator end()
  {
    return recursive_directory_iterator();
  }

  path p_;
};

int main_6(int argc, const char** argv)
{

  //How do I iterate over every file, directory, and subdirectory and store the file names in a vector in C++?
  //boost filesystem example for iterate directory and subdirectories

  // Get list of input_data in a directory -- mass reading and iterating through a directory
  //std::vector<std::string> all_XML_data_path;

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
          //cout  <<  "  has_filename()-------: " << say_what(p.has_filename()) << '\n';
          //cout  <<  "  has_stem()-----------: " << say_what(p.has_stem()) << '\n';
          //cout  <<  "  has_extension()------: " << say_what(p.has_extension()) << '\n';

          if (p.extension() == ".xml")
          {
            cout  <<  "  relative_path()------: " << p.relative_path() << '\n';
            cout  <<  "  parent_path()--------: " << p.parent_path() << '\n';
            cout  <<  "  filename()-----------: " << p.filename() << '\n';
            cout  <<  "  stem()---------------: " << p.stem() << '\n';
            cout  <<  "  extension()----------: " << p.extension() << '\n';

            cout  <<  "  has_relative_path()--: " << say_what(p.has_relative_path()) << '\n';
            cout  <<  "  has_parent_path()----: " << say_what(p.has_parent_path()) << '\n';
            cout  <<  "  has_filename()-------: " << say_what(p.has_filename()) << '\n';
            cout  <<  "  has_stem()-----------: " << say_what(p.has_stem()) << '\n';
            cout  <<  "  has_extension()------: " << say_what(p.has_extension()) << '\n';


            if (!p.empty())  // if the XML file is not empty
            {

              //cout  <<  "  empty()--------------: " << say_what(p.empty()) << '\n';
              //cout << "path.string(): " << p.string() << endl;

              // push the xml file path to one vector
              //all_XML_data_path.push_back(p.string());
              //cout << "vector size: " << all_XML_data_path.size() << endl;

              // now let s open the XML file and look into it
              //read
              const string XML_filepath(p.string());  // it has to be CONSTANT in one iteration
              cout << "XML_filepath: " << XML_filepath << endl;

              cv::FileStorage fs(XML_filepath, cv::FileStorage::READ);
              cout << endl << "Opened for Reading: " << endl;

              // First of all let s check if we can open the file
              if (!fs.isOpened())
              {
                cerr << "Fail to open: " << XML_filepath << endl;
                return 1;
              }

              //              // first method: use (type) operator on FileNode.
              //              int frameCount = (int)fs2["frameCount"];
              //
              //              std::string date;
              //              // second method: use FileNode::operator >>
              //              fs2["calibrationDate"] >> date;
              //
              //              Mat cameraMatrix2, distCoeffs2;
              //              fs2["cameraMatrix"] >> cameraMatrix2;
              //              fs2["distCoeffs"] >> distCoeffs2;
              //
              //              cout << "frameCount: " << frameCount << endl
              //                   << "calibration date: " << date << endl
              //                   << "camera matrix: " << cameraMatrix2 << endl
              //                   << "distortion coeffs: " << distCoeffs2 << endl;
              //
              //              FileNode features = fs2["features"];
              //              FileNodeIterator it = features.begin(), it_end = features.end();
              //              int idx = 0;
              //              std::vector<uchar> lbpval;


              //fs << "time_stamp_string_label" << this->time_stamp_string_label; // this is dir label - date
              const string time_stamp_string_label = fs["time_stamp_string_label"];
              cout << "time_stamp_string_label: " << time_stamp_string_label << endl;
              //              fs << "data_index" << this->data_index; // this is file label inside dir - index
              const /*unsigned*/ int data_index = fs["data_index"];
              cout << "data_index: " << data_index << endl;
              //              fs << "camera_image_filename" << camera_image_filename;
              const string camera_image_filename = fs["camera_image_filename"];
              cout << "camera_image_filename: " << camera_image_filename << endl;

              // let s show the image
              const string camera_image_filepath = p.parent_path().string() + "/" + camera_image_filename;
              const path camera_image_path(camera_image_filepath);
              cout << "camera_image_filepath: " << camera_image_filepath << endl;
              cv::Mat camera_image = cv::imread(camera_image_filepath);
              if (exists(camera_image_path)    // does p actually exist?
                  && (is_regular_file(camera_image_path))        // is p a regular file?
                  && (!camera_image_path.empty())  // if the XML file is not empty
                 )

              {
                if (camera_image.size().width > 0 && camera_image.size().height > 0)
                {
                  const string window_name = "camera_image"; //camera_image_filename + "___" + time_stamp_string_label + "___" + cv::format("_%d", data_index);
                  cv::namedWindow(window_name, CV_WINDOW_NORMAL);
                  imshow(window_name, camera_image);
                  cv::waitKey(1);
                }
              }



              //ir_distance.push_back(fs["ir_distance_front_value"]);
              //              fs << "ir_distance_front_value" << this->ir_distance_front.Get().Value();
              //              fs << "ir_distance_l_value" << this->ir_distance_l.Get().Value();
              //              fs << "ir_distance_ll_value" << this->ir_distance_ll.Get().Value();
              //              fs << "ir_distance_lll_value" << this->ir_distance_lll.Get().Value();
              //              fs << "ir_distance_rear_value" << this->ir_distance_rear.Get().Value();
              //              fs << "ir_distance_rrr_value" << this->ir_distance_rrr.Get().Value();
              //              fs << "ir_distance_rr_value" << this->ir_distance_rr.Get().Value();
              //              fs << "ir_distance_r_value" << this->ir_distance_r.Get().Value();
              const std::vector<double> ir_distance = {fs["ir_distance_front_value	"],
                                                       fs["ir_distance_lll_value"], fs["ir_distance_ll_value"], fs["ir_distance_l_value"],
                                                       fs["ir_distance_rear_value"],
                                                       fs["ir_distance_r_value"], fs["ir_distance_rr_value"], fs["ir_distance_rrr_value"]
                                                      }; // from front -- anti clockwise to rrr
              cout << "ir_distance.size(): " << ir_distance.size() << endl;

              for (std::vector<double>::const_iterator iter = ir_distance.begin(); iter < ir_distance.end(); ++iter)
              {
                cout << "*iter(ir_distance in order from front to ....): " << *iter << endl;
              }

              //              //fs << "localization data" ;
              //              fs << "pose_x" << this->pose_x.Get();
              //              fs << "pose_y" << this->pose_y.Get();
              //              fs << "pose_yaw" <<  this->pose_yaw.Get();
              const std::vector<double> pose = {fs["pose_x"], fs["pose_y"], fs["pose_yaw"]};
              cout << "pose.size(): " << pose.size() << endl;
              for (std::vector<double>::const_iterator iter = pose.begin(); iter < pose.end(); ++iter)
              {
                cout << "*iter (pose values from X, Y, Yaw): " << *iter << endl;
              }

              //              // output == controlling data
              //              fs << "desired_velocity" << this->desired_velocity.Get().Value();
              //              fs << "desired_angular_velocity" << this->desired_angular_velocity.Get().Value().Value();
              //              fs << "desired_fork_position" << this->desired_fork_position.Get();
              const double desired_velocity = fs["desired_velocity"];
              cout << "desired_velocity: " << desired_velocity << endl;
              const double desired_angular_velocity = fs["desired_angular_velocity"];
              cout << "desired_angular_velocity: " << desired_angular_velocity << endl;
              const double desired_fork_position = fs["desired_fork_position"];
              cout << "desired_fork_position: " << desired_fork_position << endl;


              cout << endl << "Closed and released after reading: " << endl;
              fs.release();

            }// if xml file not empty
          } // if xml
        }// if has extension
      }// if regular file
    } // if exists

  }// for


  // is this really neccessary SINCE WE WANT TO LITERALLY MASS READ ALLT THE INPUT DATA FOR READING AND all of them have labels and stuff
  //sort(all_XML_data_path.begin(), all_XML_data_path.end());


  return 0;
}

#include "projects/stereo_traversability_experiments/def/tMLR.h"
void finroc::stereo_traversability_experiments::def::tMLR::boost_filesystem_test(int argc, const char** argv)
{

//  main_1(argc, argv);
//  main_2(argc, argv);
//  main_3(argc, argv);
//  main_4(argc, argv);
//  main_5(argc, argv);
  main_6(argc, argv);
}
