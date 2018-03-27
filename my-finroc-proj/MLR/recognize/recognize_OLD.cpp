#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::recognize_OLD(/*INPUT*/const Mat& I_scaled_vec, /*INPUT*/const Mat& I_new_scaled_vec, /*INPUT*/const Mat& I_img_sample, /*INPUT*/const char** argv)
{

  mlr::learn_PCA::tPCA pca;
  read_tPCA(argv, pca);
  cout << "pca loaded successfully....................." << endl;
  LOG_PRINT(pca.mean, "pca.mean: ", eLOG_PRINT_STATE::eIS_ACTIVE);
  LOG_PRINT(pca.S, "pca.S: ", eLOG_PRINT_STATE::eIS_ACTIVE);
  std::cout << "pca.S: " << pca.S << std::endl;
  LOG_PRINT(pca.V, "pca.V: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  // MSD (Min Square Diff) --> Also known as Mean Square Error or Least Square Error
  vector<double> vec_diff_scaled_norm, vec_diff_norm; // used for result

  // MCC (Max Cross Correlation) --> Also know projection on axis
  vector<double> vec_sim_scaled_norm, vec_sim_norm;

  // Feature Transform (projection) & compare using distance functions
  for (/*unsigned*/ int col_num = 0; col_num < I_new_scaled_vec.cols; ++col_num)
  {
    // This is only to visualize the input data sample after scaling, vectorizing and projection -- the reconstructed input dat sample
    Mat test_data = I_new_scaled_vec.col(col_num).clone(); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
    test_data = test_data.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
    test_data = toGrayscale(test_data);
    string window_name = "new data";
    namedWindow(window_name, WINDOW_NORMAL);
    imshow(window_name, test_data);
    //waitKey(0);
    string winName = "search in database"; // destroy the window later

    // the query or test data
    Mat I_new_scaled_vec_col_i_projected = pca.project(I_new_scaled_vec.col(col_num).clone()); // new input data as test

    for (/*unsigned */int i = 0 ; i < I_scaled_vec.cols; i++) //pool of patches
    {
      // showing the the current database
      Mat sample = I_scaled_vec.col(i).clone();
      sample = sample.reshape(0, I_img_sample.rows);
      sample = toGrayscale(sample);
      namedWindow(winName, WINDOW_NORMAL);
      imshow(winName, sample);
      //      unsigned int delay = 0; // stop for comparing the results
      //      /*char key =*/ waitKey(delay);

      Mat I_scaled_vec_col_i_projected = pca.project(I_scaled_vec.col(i).clone()); // project into the EigenSpace
      LOG_PRINT(I_scaled_vec_col_i_projected, "I_scaled_vec_col_i_projected", false);


      // compare them based on difference in distance with scaling -- should be a value at the end
      Mat diff = I_scaled_vec_col_i_projected - I_new_scaled_vec_col_i_projected; // input vectors are already CV64C1 after pre-processing(scaling and vectorization)
      Mat diff_scaled(diff.size(), diff.type());

      // similarity measurement (Cross Correlation)
      Mat sim(diff.size(), diff.type());
      Mat sim_scaled(diff.size(), diff.type());
      double sim_cosine(0.0);
      double sim_scaled_cosine(0.0);
      double I_scaled_vec_col_i_projected_norm = norm(I_scaled_vec_col_i_projected, NORM_L2); // NORM_L2 == sqrt(sum(X^2))
      double I_new_scaled_vec_col_i_projected_norm = norm(I_new_scaled_vec_col_i_projected, NORM_L2);

      // Distance function and Similarity function using metrics
      for (/*unsigned*/ int i = 0; i < diff.rows; i++)
      {

        //diff_scaled.row(i) = (diff.row(i)) / (std::pow(pca.S.col(i), 2.0));
        //diff_scaled.at<double>(i) = diff.at<double>(i) / double(std::pow(pca.S.at<double>(i), 2.0));
        diff_scaled.at<double>(i) = diff.at<double>(i) / pca.S.at<double>(i);

        /*! This is equal to Fi_normalized dot Fi_new_normalized */
        sim.at<double>(i) = (I_scaled_vec_col_i_projected.at<double>(i) / I_scaled_vec_col_i_projected_norm) *
                            (I_new_scaled_vec_col_i_projected.at<double>(i) / I_new_scaled_vec_col_i_projected_norm);
        sim_cosine += sim.at<double>(i);

        // scaling the MCC
        //sim_scaled.at<double>(i) = (I_scaled_vec_col_i_projected.at<double>(i) * I_new_scaled_vec_col_i_projected.at<double>(i)) / double(std::pow(pca.S.at<double>(i), 2.0));
        sim_scaled.at<double>(i) = (I_scaled_vec_col_i_projected.at<double>(i) / pca.S.at<double>(i)) *
                                   (I_new_scaled_vec_col_i_projected.at<double>(i) / pca.S.at<double>(i));
        sim_scaled_cosine += sim_scaled.at<double>(i);
      }

      double diff_scaled_norm = norm(diff_scaled, NORM_L2); // this should be MIN
      vec_diff_scaled_norm.push_back(diff_scaled_norm);
      double diff_norm = norm(diff, NORM_L2); // this should be MIN
      vec_diff_norm.push_back(diff_norm);

      vec_sim_scaled_norm.push_back(sim_scaled_cosine);
      vec_sim_norm.push_back(sim_cosine);
    }// for searching the database

    // once done searching for the best match - min diff & max sim
    destroyWindow(winName);



    unsigned int delay = 0;
    char key;
    unsigned int min_index_diff_scaled(0), max_index_diff_scaled(0); // array or vector index > 0 ALWAYS --> unsgined
    unsigned int max_index_sim_scaled(0), min_index_sim_scaled(0);
    unsigned int min_index_diff(0), max_index_diff(0);
    unsigned int max_index_sim(0), min_index_sim(0);


    // To show the results in order
    for (unsigned int best_match_index = 0; /*best match should be smaller than vector size*/; best_match_index++)
    {
      for (unsigned i /*index in an array or vector or row*/ = 0; i < vec_diff_scaled_norm.size(); ++i)
      {
        //Scaled MSD
        if (vec_diff_scaled_norm[i] < vec_diff_scaled_norm[min_index_diff_scaled]) // Found a smaller min for difference
        {
          min_index_diff_scaled = i; // this is important  -- min difference
        }
        if (vec_diff_scaled_norm[i] > vec_diff_scaled_norm[max_index_diff_scaled]) // Found a bigger max for similarity
        {
          max_index_diff_scaled = i; // this is important  -- min difference
        }
        // MCC
        if (vec_sim_scaled_norm[i] > vec_sim_scaled_norm[max_index_sim_scaled]) // Found a bigger max for similarity
        {
          max_index_sim_scaled = i;
        }
        if (vec_sim_scaled_norm[i] < vec_sim_scaled_norm[min_index_sim_scaled]) // Found a bigger max for similarity
        {
          min_index_sim_scaled = i;
        }
        // MSD (Min Square Diff)
        if (vec_diff_norm[i] < vec_diff_norm[min_index_diff]) // Found a smaller min for difference
        {
          min_index_diff = i;
        }
        if (vec_diff_norm[i] > vec_diff_norm[max_index_diff]) // Found a smaller min for difference
        {
          max_index_diff = i;
        }
        // MCC (Max Cross Correlation)
        if (vec_sim_norm[i] > vec_sim_norm[max_index_sim]) // Found a bigger max for similarity
        {
          max_index_sim = i;
        }
        if (vec_sim_norm[i] < vec_sim_norm[min_index_sim]) // Found a bigger max for similarity
        {
          min_index_sim = i;
        }
      }// for


      cout << "==================================================================================================" << endl;
      std::cout << "Min difference is " << vec_diff_norm[min_index_diff] << " at position " << min_index_diff << endl /*end the current line*/;  // this is wanted
      std::cout << "Max difference is " << vec_diff_norm[max_index_diff] << " at position " << max_index_diff << endl /*end the current line*/;  // this is wanted
      std::cout << "Min difference scaled is " << vec_diff_scaled_norm[min_index_diff_scaled] << " at position " << min_index_diff_scaled << endl /*end the current line*/;  // this is wanted
      std::cout << "Max difference scaled is " << vec_diff_scaled_norm[max_index_diff_scaled] << " at position " << max_index_diff_scaled << endl /*end the current line*/;  // this is wanted
      std::cout << "Max similarity is " << vec_sim_norm[max_index_sim] << " at position " << max_index_sim << endl;
      std::cout << "Min similarity is " << vec_sim_norm[min_index_sim] << " at position " << min_index_sim << endl;
      std::cout << "Max similarity scaled is " << vec_sim_scaled_norm[max_index_sim_scaled] << " at position " << max_index_sim_scaled << endl;
      std::cout << "Min similarity scaled is " << vec_sim_scaled_norm[min_index_sim_scaled] << " at position " << min_index_sim_scaled << endl;
      cout << "==================================================================================================" << endl;

      // This is only to visualize the input data sample after scaling, vectorizing and projection -- the reconstructed input dat sample
      Mat min_diff_scaled_result = I_scaled_vec.col(min_index_diff_scaled).clone(); ///*pca.backProject*/(I_scaled_vec_col_i); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      min_diff_scaled_result = min_diff_scaled_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      min_diff_scaled_result = toGrayscale(min_diff_scaled_result);
      string window_name = format("min_diff_scaled_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name, WINDOW_NORMAL);
      imshow(window_name, min_diff_scaled_result);

      //      // This is only to visualize the input data sample after scaling, vectorizing and projection -- the reconstructed input dat sample
      //      Mat max_diff_scaled_result = I_scaled_vec.col(max_index_diff_scaled).clone(); ///*pca.backProject*/(I_scaled_vec_col_i); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      //      max_diff_scaled_result = max_diff_scaled_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      //      max_diff_scaled_result = toGrayscale(max_diff_scaled_result);
      //      /*string */window_name = "max_diff_scaled_result"; //format("max_diff_scaled_result_%d", best_match_index); //"best match based on min difference";
      //      namedWindow(window_name, WINDOW_NORMAL);
      //      imshow(window_name, max_diff_scaled_result);

      Mat max_sim_scaled_result = I_scaled_vec.col(max_index_sim_scaled).clone(); //pca.backProject(I_scaled_vec_col_i_projected); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      max_sim_scaled_result = max_sim_scaled_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      max_sim_scaled_result = toGrayscale(max_sim_scaled_result);
      /*string*/ window_name = format("max_sim_scaled_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name, WINDOW_NORMAL);
      imshow(window_name, max_sim_scaled_result);

      //      Mat min_sim_scaled_result = I_scaled_vec.col(min_index_sim_scaled).clone(); //pca.backProject(I_scaled_vec_col_i_projected); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      //      min_sim_scaled_result = min_sim_scaled_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      //      min_sim_scaled_result = toGrayscale(min_sim_scaled_result);
      //      /*string*/ window_name = "min_sim_scaled_result"; //format("min_sim_scaled_result_%d", best_match_index); //"best match based on min difference";
      //      namedWindow(window_name, WINDOW_NORMAL);
      //      imshow(window_name, min_sim_scaled_result);

      Mat min_diff_result = I_scaled_vec.col(min_index_diff).clone(); ///*pca.backProject*/(I_scaled_vec_col_i); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      min_diff_result = min_diff_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      min_diff_result = toGrayscale(min_diff_result);
      /*string*/ window_name = format("min_diff_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name, WINDOW_NORMAL);
      imshow(window_name, min_diff_result);

      //      Mat max_diff_result = I_scaled_vec.col(max_index_diff).clone(); ///*pca.backProject*/(I_scaled_vec_col_i); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      //      max_diff_result = max_diff_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      //      max_diff_result = toGrayscale(max_diff_result);
      //      /*string*/ window_name = "max_diff_result"; //format("min_diff_result_%d", best_match_index); //"best match based on min difference";
      //      namedWindow(window_name, WINDOW_NORMAL);
      //      imshow(window_name, max_diff_result);


      Mat max_sim_result = I_scaled_vec.col(max_index_sim).clone(); //pca.backProject(I_scaled_vec_col_i_projected); // re-create the input data sample from the "I_scaled_vec_col_i_projected"
      max_sim_result = max_sim_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      max_sim_result = toGrayscale(max_sim_result);
      /*string*/ window_name = format("max_sim_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name, WINDOW_NORMAL);
      imshow(window_name, max_sim_result);

      key = waitKey(delay);
      if (key == 'q' /*quit*/ || key == 27 /*ESC*/)
      {
        break;
      }// if

      vec_diff_scaled_norm[min_index_diff_scaled] = vec_diff_scaled_norm[max_index_diff_scaled];
      vec_sim_scaled_norm[max_index_sim_scaled] = vec_sim_scaled_norm[min_index_sim_scaled];
      vec_diff_norm[min_index_diff] = vec_diff_norm[max_index_diff];
      vec_sim_norm[max_index_sim] = vec_sim_norm[min_index_sim];
    }// for finding the max sim and min diff


    destroyAllWindows();
    vec_diff_scaled_norm.clear();
    vec_sim_scaled_norm.clear();
    vec_diff_norm.clear();
    vec_sim_norm.clear();
  }// for loop for I_new

}// end of recognition
