#include <iostream> // cout , cin
#include <boost/filesystem.hpp> // pretty obvious // iterating through system files --> boost::filesystem
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/highgui/highgui.hpp> // imread and imshow ...

using namespace boost::filesystem;
using namespace std;

#include "projects/stereo_traversability_experiments/mlr/tMLR.h"
using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::fuse_data(/*INPUT*/const char** argv, /*OUTPUT*/cv::Mat& I_all)
{

  mlr::learn_PCA::tPCA pca_img;
  read_tPCA(argv, pca_img, "learned_PCA_img");
  //cout << "pca loaded successfully....................." << endl;
  //this->display_PCs(pca, I_img_sample);

  mlr::learn_PCA::tPCA pca_dist;
  read_tPCA(argv, pca_dist, "learned_PCA_dist");
  //  cv::Mat mean_dist;
  //  read_mean(argv, "mean_dist", mean_dist);

  //  mlr::learn_PCA::tPCA pca_loc;
  //  read_tPCA(argv, pca_loc, "learned_PCA_loc");
  cv::Mat mean_pose/*loc*/;
  read_mean(argv, "mean_pose", mean_pose/*loc*/);

  // reading all the sensors data separately first and concatenate them all at the end
  std::vector<cv::Mat> vec_I_img_all, vec_I_dist_all, vec_I_pose_all;

  //How do I iterate over every file, directory, and sub-directory and store the file names in a vector in C++?
  // to iterate directory and sub-directories
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
          //cout  <<  "  has_filename()-------: " <<   (p.has_filename()) << '\n';
          //cout  <<  "  has_stem()-----------: " <<   (p.has_stem()) << '\n';
          //cout  <<  "  has_extension()------: " <<   (p.has_extension()) << '\n';

          if (p.extension() == ".xml")
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

              //cout  <<  "  empty()--------------: " <<   (p.empty()) << '\n';
              //cout << "path.string(): " << p.string() << endl;

              // now let s open the XML file and look into it
              const string XML_filepath(p.string());  // it has to be CONSTANT in one iteration
              cout << "XML_filepath: " << XML_filepath << endl;

              // The recorded data structure to pass allt he recorded data at once
              //finroc::stereo_traversability_experiments::control::tRecordedData recorded_data;

              cv::FileStorage fs(XML_filepath, cv::FileStorage::READ);
              cout << endl << "Opened for Reading: " << endl;

              // First of all let s check if we can open the file
              if (!fs.isOpened())
              {
                cerr << "Fail to open: " << XML_filepath << endl;
                //return 1;
                exit(1);
              }


              //fs << "time_stamp_string_label" << this->time_stamp_string_label; // this is dir label - date
              const string time_stamp_string_label = fs["time_stamp_string_label"];
              cout << "time_stamp_string_label: " << time_stamp_string_label << endl;
              //recorded_data.time_stamp_string_label = time_stamp_string_label;

              //              fs << "data_index" << this->data_index; // this is file label inside dir - index
              const /*unsigned*/ int data_index = fs["data_index"];
              cout << "data_index: " << data_index << endl;
              //recorded_data.data_index = data_index;

              //              fs << "camera_image_filename" << camera_image_filename;
              const string camera_image_filename = fs["camera_image_filename"];
              cout << "camera_image_filename: " << camera_image_filename << endl;
              // camera image filename is not needed since we read the actual image

              // let s show the image
              const string camera_image_filepath = p.parent_path().string() + "/" + camera_image_filename;
              const path camera_image_path(camera_image_filepath);
              cout << "camera_image_filepath: " << camera_image_filepath << endl;



              const std::vector<double> ir_distance = {fs["ir_distance_front_value	"],
                                                       fs["ir_distance_lll_value"], fs["ir_distance_ll_value"], fs["ir_distance_l_value"],
                                                       fs["ir_distance_rear_value"],
                                                       fs["ir_distance_r_value"], fs["ir_distance_rr_value"], fs["ir_distance_rrr_value"]
                                                      }; // from front -- anti clockwise to rrr
              cout << "ir_distance.size(): " << ir_distance.size() << endl;
              //
              //              for (std::vector<double>::const_iterator iter = ir_distance.begin(); iter < ir_distance.end(); ++iter)
              //              {
              //                cout << "*iter(ir_distance in order from front to ....): " << *iter << endl;
              //              }

              const std::vector<double> pose = {fs["pose_x"], fs["pose_y"], fs["pose_yaw"]};
              cout << "pose.size(): " << pose.size() << endl;
              //              for (std::vector<double>::const_iterator iter = pose.begin(); iter < pose.end(); ++iter)
              //              {
              //                cout << "*iter (pose values from X, Y, Yaw): " << *iter << endl;
              //              }

              cout << endl << "Closed and released after reading: " << endl;
              fs.release();


              //////////////Fusing data
              if (
                (exists(camera_image_path)    // does p actually exist?
                 && is_regular_file(camera_image_path)        // is p a regular file?
                 && !camera_image_path.empty())  // if the XML file is not empty
                && (pose.size() > 0 && !pose.empty())
                && (ir_distance.size() > 0 && !ir_distance.empty())
              )
              {
                const cv::Mat camera_image = cv::imread(camera_image_filepath/*, DEFAULT IST color*/); // cv::Mat and std::vector they are almost having the same data structure
                if (camera_image.rows/*size().width*/ > 0 && camera_image.cols/*size().height*/ > 0  && !camera_image.empty() /*camera image not empty*/)
                {
                  /*const */string window_name = "camera_image"; //camera_image_filename + "___" + time_stamp_string_label + "___" + cv::format("_%d", data_index);
                  cv::namedWindow(window_name, CV_WINDOW_NORMAL);
                  imshow(window_name, camera_image);

                  const cv::Mat I_img_RAW /*one channel*/ = cv::imread(camera_image_filepath, CV_LOAD_IMAGE_GRAYSCALE); // one channel
                  cv::Mat I_img_norm;
                  toNormalize(I_img_RAW, I_img_norm, eDATA_TYPE::eIMAGE_DATA);
                  cv::Mat I_img = vectorize_input_data(I_img_norm);
                  cv::Mat I_img_proj = pca_img.V.t() * (I_img - pca_img.mean); // projection or transform or dim reduced to PCs
                  vec_I_img_all.push_back(I_img_proj);

                  const cv::Mat I_dist_RAW/*one channel*/(ir_distance); // one channel
                  cv::Mat I_dist_norm;
                  toNormalize(I_dist_RAW, I_dist_norm, eDATA_TYPE::eDISTANCE_DATA);
                  cv::Mat I_dist = vectorize_input_data(I_dist_norm);
                  cv::Mat I_dist_proj = pca_dist.V.t() * (I_dist - pca_dist.mean); // projection or transform or dim reduced to PCs
                  vec_I_dist_all.push_back(I_dist_proj);

                  cv::Mat I_pose_RAW(pose); // same as I_localization
                  //                  cv::Mat I_pose_norm;
                  //                  toNormalize(I_pose_RAW, I_pose_norm, eDATA_TYPE::eLOCALIZATION_DATA);
                  cv::Mat I_pose = vectorize_input_data(I_pose_RAW/*norm*/);
                  cv::Mat I_pose_proj = /*pca_loc.V.t()*/ /*1000 **/ (I_pose - mean_pose/*loc*//*pca_loc.mean*//*_pose*/); // projection or transform or dim reduced to PCs
                  vec_I_pose_all.push_back(I_pose_proj);

                }// if camera image exit and not empy and have size
              }// main if all have size and not empty
            }// if xml file not empty
          } // if xml
        }// if has extension
      }// if regular file
    } // if exists
  }// for

  // vectorzing them
  cv::Mat I_img_all = vectorize_input_data(vec_I_img_all);
  cv::Mat I_dist_all = vectorize_input_data(vec_I_dist_all);
  cv::Mat I_pose_all = vectorize_input_data(vec_I_pose_all);

  // concatenating the vectors
  std::vector<cv::Mat> vec_I_all = {I_img_all, I_dist_all, I_pose_all};
  cv::vconcat(vec_I_all, I_all);

}// end of the function
