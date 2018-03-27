#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

void tMLR::recognize_test(/*INPUT*/const Mat& I_all, /*INPUT*/const Mat& /*images*/I_all_img, /*INPUT*/const Mat& I_img_sample, /*INPUT*/const char** argv, /*INPUT*/const eDATA_TYPE& data_type)
{

  mlr::learn_PCA::tPCA pca;
  //  if (data_type == eDATA_TYPE::eLOCALIZATION_DATA)
  //  {
  //    read_tPCA(argv, pca, "learned_PCA_loc");
  //  }
  if (data_type == eDATA_TYPE::eALL_DATA)
  {
    read_tPCA(argv, pca, "learned_PCA_all");
  }
  if (data_type == eDATA_TYPE::eIMAGE_DATA)
  {
    read_tPCA(argv, pca, "learned_PCA_img");
  }
  if (data_type == eDATA_TYPE::eDISTANCE_DATA)
  {
    read_tPCA(argv, pca, "learned_PCA_dist");
  }
  //cout << "pca loaded successfully....................." << endl;
  //this->display_PCs(pca, I_img_sample);



  // instantiate, create and initialize with zero TODO
  cv::Mat S_diag_inv /*Diagonal singular values or Diagonal version of S */ =
    cv::Mat::zeros(pca.S.cols /*rows*/, pca.S.cols /*columns*/, pca.S.type()); // V_1,....., V_k --> S_1,....,S_k --> D_1x1,...., D_kxk
  for (int j = 0; j < S_diag_inv.cols; j++)
  {
    S_diag_inv.at<double>(j, j) = double(1.0) / pca.S.at<double>(j); // in order to preserve the double precision while inversing
  }
  //LOG_PRINT(S_diag_inv, "S_diag_inv", finroc::stereo_traversability_experiments::mlr::eLOG_PRINT_STATE::eIS_NOT_ACTIVE);


  // Feature Transform (projection) & compare using distance functions
  for (/*unsigned*/ int j_I_new = 0; j_I_new < I_all.cols; ++j_I_new) // j, col, x and width
  {
    // the new input data sample after pre-processing (one channel (2D array or matrix), scaling (normalizing) and vectorizing (COL vector))
    // AKA (Also Known AS) the query or test data
    Mat I_new = I_all.col(j_I_new).clone();
    cv::Mat I_new_img = I_all_img.col(j_I_new).clone().reshape(0 /* ZERO channels or Matrix*/, I_img_sample.rows);
    I_new_img = toGrayscale(I_new_img);
    string window_name_I_new_img = "I_new_img";
    namedWindow(window_name_I_new_img, WINDOW_NORMAL);
    imshow(window_name_I_new_img, I_new_img);


    // the window for searching in database for finding the best match
    string win_name_I_img = "search in database"; // destroy the window later
    std::vector<double> vec_diff_norm;/*.push_back(diff_norm);*/
    std::vector<double> vec_sim_value; /*.push_back(sim_value);*/
    std::vector<double> vec_diff_mu_norm;/*.push_back(diff_mu_norm);*/
    std::vector<double> vec_sim_mu_value; /*.push_back(sim_mu_value);*/
    std::vector<double> vec_diff_mu_V_norm;/*.push_back(diff_mu_V_norm);*/
    std::vector<double> vec_sim_mu_V_value; /*.push_back(sim_mu_V_value);*/
    std::vector<double> vec_diff_mu_V_S_norm;/*.push_back(diff_mu_V_S_norm);*/
    std::vector<double> vec_sim_mu_V_S_value; /*.push_back(sim_mu_V_S_value);*/


    for (/*unsigned */int j_I = 0 ; j_I < I_all.cols; j_I++) //pool of patches
    {
      // showing the the current database
      Mat I = I_all.col(j_I).clone();
      cv::Mat I_img = I_all_img.col(j_I).clone().reshape(0, I_img_sample.rows);
      I_img = toGrayscale(I_img);
      namedWindow(win_name_I_img, WINDOW_NORMAL);
      imshow(win_name_I_img, I_img);

      std::vector<double> vec_diff_norms, vec_sim_values; // comparison results
      compare(I, I_new, pca, S_diag_inv, vec_diff_norms, vec_sim_values);

      vec_diff_norm.push_back(vec_diff_norms[0]);
      vec_diff_mu_norm.push_back(vec_diff_norms[1]);
      vec_diff_mu_V_norm.push_back(vec_diff_norms[2]);
      vec_diff_mu_V_S_norm.push_back(vec_diff_norms[3]);

      vec_sim_value.push_back(vec_sim_values[0]);
      vec_sim_mu_value.push_back(vec_sim_values[1]);
      vec_sim_mu_V_value.push_back(vec_sim_values[2]);
      vec_sim_mu_V_S_value.push_back(vec_sim_values[3]);

    }// for loop I (searching through database)

    // close the searching window
    destroyWindow(win_name_I_img);
    unsigned int min_index_diff(0), max_index_diff(0);
    unsigned int max_index_sim(0), min_index_sim(0);
    unsigned int min_index_diff_mu(0), max_index_diff_mu(0);
    unsigned int max_index_sim_mu(0), min_index_sim_mu(0);
    unsigned int min_index_diff_mu_V(0), max_index_diff_mu_V(0);
    unsigned int max_index_sim_mu_V(0), min_index_sim_mu_V(0);
    unsigned int min_index_diff_mu_V_S(0), max_index_diff_mu_V_S(0);
    unsigned int max_index_sim_mu_V_S(0), min_index_sim_mu_V_S(0);

    // To show the results in order
    for (unsigned int best_match_index = 0; /*EMPTY*/ ; best_match_index++)
    {
      for (unsigned i = 0; i < vec_diff_norm.size(); ++i) // the size of all vectors == I.cols number of input data samples
      {
        //MSD (Minimum Square Difference (or Distance)) --> Distance function
        if (vec_diff_norm[i] < vec_diff_norm[min_index_diff])
        {
          min_index_diff = i; // this is important  -- min difference
        }
        if (vec_diff_norm[i] > vec_diff_norm[max_index_diff])
        {
          max_index_diff = i;
        }
        // MCC (maximum Cross Correlation (or CoRelation)) --> Distance function
        if (vec_sim_value[i] > vec_sim_value[max_index_sim]) // Found a bigger max for similarity
        {
          max_index_sim = i;
        }
        if (vec_sim_value[i] < vec_sim_value[min_index_sim]) // Found a bigger max for similarity
        {
          min_index_sim = i;
        }
        ////////////////////////////////////////// Mu
        //MSD (Minimum Square Difference (or Distance)) --> Distance function
        if (vec_diff_mu_norm[i] < vec_diff_mu_norm[min_index_diff_mu])
        {
          min_index_diff_mu = i; // this is important  -- min difference
        }
        if (vec_diff_mu_norm[i] > vec_diff_mu_norm[max_index_diff_mu])
        {
          max_index_diff_mu = i;
        }
        // MCC (maximum Cross Correlation (or CoRelation)) --> Distance function
        if (vec_sim_mu_value[i] > vec_sim_mu_value[max_index_sim_mu]) // Found a bigger max for similarity
        {
          max_index_sim_mu = i;
        }
        if (vec_sim_mu_value[i] < vec_sim_mu_value[min_index_sim_mu]) // Found a bigger max for similarity
        {
          min_index_sim_mu = i;
        }
        //////////////////////////////////////////// Mu_V
        //MSD (Minimum Square Difference (or Distance)) --> Distance function
        if (vec_diff_mu_V_norm[i] < vec_diff_mu_V_norm[min_index_diff_mu_V])
        {
          min_index_diff_mu_V = i; // this is important  -- min difference
        }
        if (vec_diff_mu_V_norm[i] > vec_diff_mu_V_norm[max_index_diff_mu_V])
        {
          max_index_diff_mu_V = i;
        }
        // MCC (maximum Cross Correlation (or CoRelation)) --> Distance function
        if (vec_sim_mu_V_value[i] > vec_sim_mu_V_value[max_index_sim_mu_V]) // Found a bigger max for similarity
        {
          max_index_sim_mu_V = i;
        }
        if (vec_sim_mu_V_value[i] < vec_sim_mu_V_value[min_index_sim_mu_V]) // Found a bigger max for similarity
        {
          min_index_sim_mu_V = i;
        }
        /////////////////////////////////////////////// Mu_V_S
        //MSD (Minimum Square Difference (or Distance)) --> Distance function
        if (vec_diff_mu_V_S_norm[i] < vec_diff_mu_V_S_norm[min_index_diff_mu_V_S])
        {
          min_index_diff_mu_V_S = i; // this is important  -- min difference
        }
        if (vec_diff_mu_V_S_norm[i] > vec_diff_mu_V_S_norm[max_index_diff_mu_V_S])
        {
          max_index_diff_mu_V_S = i;
        }
        // MCC (maximum Cross Correlation (or CoRelation)) --> Distance function
        if (vec_sim_mu_V_S_value[i] > vec_sim_mu_V_S_value[max_index_sim_mu_V_S]) // Found a bigger max for similarity
        {
          max_index_sim_mu_V_S = i;
        }
        if (vec_sim_mu_V_S_value[i] < vec_sim_mu_V_S_value[min_index_sim_mu_V_S]) // Found a bigger max for similarity
        {
          min_index_sim_mu_V_S = i;
        }
      }// for vec.size()

      cout << "==================================================================================================" << endl;
      std::cout << "Min difference is " << vec_diff_norm[min_index_diff] << " at position " << min_index_diff << endl /*end the current line*/;  // this is wanted
      std::cout << "Max difference is " << vec_diff_norm[max_index_diff] << " at position " << max_index_diff << endl /*end the current line*/;  // this is wanted
      std::cout << "Min difference Mu is " << vec_diff_mu_norm[min_index_diff_mu] << " at position " << min_index_diff_mu << endl /*end the current line*/;  // this is wanted
      std::cout << "Max difference Mu is " << vec_diff_mu_norm[max_index_diff_mu] << " at position " << max_index_diff_mu << endl /*end the current line*/;  // this is wanted
      std::cout << "Min difference Mu V is " << vec_diff_mu_V_norm[min_index_diff_mu_V] << " at position " << min_index_diff_mu_V << endl /*end the current line*/;  // this is wanted
      std::cout << "Max difference Mu V is " << vec_diff_mu_V_norm[max_index_diff_mu_V] << " at position " << max_index_diff_mu_V << endl /*end the current line*/;  // this is wanted
      std::cout << "Min difference Mu V S is " << vec_diff_mu_V_S_norm[min_index_diff_mu_V_S] << " at position " << min_index_diff_mu_V_S << endl /*end the current line*/;  // this is wanted
      std::cout << "Max difference Mu V S is " << vec_diff_mu_V_S_norm[max_index_diff_mu_V_S] << " at position " << max_index_diff_mu_V_S << endl /*end the current line*/;  // this is wanted

      std::cout << "Max similarity is " << vec_sim_value[max_index_sim] << " at position " << max_index_sim << endl;
      std::cout << "Min similarity is " << vec_sim_value[min_index_sim] << " at position " << min_index_sim << endl;
      std::cout << "Max similarity Mu is " << vec_sim_mu_value[max_index_sim_mu] << " at position " << max_index_sim_mu << endl;
      std::cout << "Min similarity Mu is " << vec_sim_mu_value[min_index_sim_mu] << " at position " << min_index_sim_mu << endl;
      std::cout << "Max similarity Mu V is " << vec_sim_mu_V_value[max_index_sim_mu_V] << " at position " << max_index_sim_mu_V << endl;
      std::cout << "Min similarity Mu V is " << vec_sim_mu_V_value[min_index_sim_mu_V] << " at position " << min_index_sim_mu_V << endl;
      std::cout << "Max similarity Mu V S is " << vec_sim_mu_V_S_value[max_index_sim_mu_V_S] << " at position " << max_index_sim_mu_V_S << endl;
      std::cout << "Min similarity Mu V S is " << vec_sim_mu_V_S_value[min_index_sim_mu_V_S] << " at position " << min_index_sim_mu_V_S << endl;
      cout << "==================================================================================================" << endl;

      // display the resulting input data sample
      Mat min_diff_result = I_all_img.col(min_index_diff).clone();
      min_diff_result = min_diff_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      min_diff_result = toGrayscale(min_diff_result);
      string window_name_diff = format("min_diff_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_diff, WINDOW_NORMAL);
      imshow(window_name_diff, min_diff_result);

      Mat max_sim_result = I_all_img.col(max_index_sim).clone();
      max_sim_result = max_sim_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      max_sim_result = toGrayscale(max_sim_result);
      string window_name_sim = format("max_sim_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_sim, WINDOW_NORMAL);
      imshow(window_name_sim, max_sim_result);

      // display the resulting input data sample - after subtracting Mu (mean)
      Mat min_diff_mu_result = I_all_img.col(min_index_diff_mu).clone();
      min_diff_mu_result = min_diff_mu_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      min_diff_mu_result = toGrayscale(min_diff_mu_result);
      string window_name_diff_mu = format("min_diff_mu_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_diff_mu, WINDOW_NORMAL);
      imshow(window_name_diff_mu, min_diff_mu_result);

      Mat max_sim_mu_result = I_all_img.col(max_index_sim_mu).clone();
      max_sim_mu_result = max_sim_mu_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      max_sim_mu_result = toGrayscale(max_sim_mu_result);
      string window_name_sim_mu = format("max_sim_mu_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_sim_mu, WINDOW_NORMAL);
      imshow(window_name_sim_mu, max_sim_mu_result);

      // display the resulting input data sample - after subtracting Mu (mean) and applying V
      Mat min_diff_mu_V_result = I_all_img.col(min_index_diff_mu_V).clone();
      min_diff_mu_V_result = min_diff_mu_V_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      min_diff_mu_V_result = toGrayscale(min_diff_mu_V_result);
      string window_name_diff_mu_V = format("min_diff_mu_V_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_diff_mu_V, WINDOW_NORMAL);
      imshow(window_name_diff_mu_V, min_diff_mu_V_result);

      Mat max_sim_mu_V_result = I_all_img.col(max_index_sim_mu_V).clone();
      max_sim_mu_V_result = max_sim_mu_V_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      max_sim_mu_V_result = toGrayscale(max_sim_mu_V_result);
      string window_name_sim_mu_V = format("max_sim_mu_V_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_sim_mu_V, WINDOW_NORMAL);
      imshow(window_name_sim_mu_V, max_sim_mu_V_result);

      // display the resulting input data sample - after subtracting Mu (mean) and applying V and Scaling using S
      Mat min_diff_mu_V_S_result = I_all_img.col(min_index_diff_mu_V_S).clone();
      min_diff_mu_V_S_result = min_diff_mu_V_S_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      min_diff_mu_V_S_result = toGrayscale(min_diff_mu_V_S_result);
      string window_name_diff_mu_V_S = format("min_diff_mu_V_S_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_diff_mu_V_S, WINDOW_NORMAL);
      imshow(window_name_diff_mu_V_S, min_diff_mu_V_S_result);

      Mat max_sim_mu_V_S_result = I_all_img.col(max_index_sim_mu_V_S).clone();
      max_sim_mu_V_S_result = max_sim_mu_V_S_result.reshape(0 /*no change in number of channels*/, I_img_sample.rows);
      max_sim_mu_V_S_result = toGrayscale(max_sim_mu_V_S_result);
      string window_name_sim_mu_V_S = format("max_sim_mu_V_S_result_%d", best_match_index); //"best match based on min difference";
      namedWindow(window_name_sim_mu_V_S, WINDOW_NORMAL);
      imshow(window_name_sim_mu_V_S, max_sim_mu_V_S_result);

      char key = waitKey(/*delay*/0);
      if (key == 'q' /*quit*/ || key == 27 /*ESC*/)
      {
        break;
      }// if

      // closing the previous windows
      destroyWindow(window_name_diff);
      destroyWindow(window_name_sim);
      destroyWindow(window_name_diff_mu);
      destroyWindow(window_name_sim_mu);
      destroyWindow(window_name_diff_mu_V);
      destroyWindow(window_name_sim_mu_V);
      destroyWindow(window_name_diff_mu_V_S);
      destroyWindow(window_name_sim_mu_V_S);

      vec_diff_norm[min_index_diff] = vec_diff_norm[max_index_diff];
      vec_sim_value[max_index_sim] = vec_sim_value[min_index_sim];
      vec_diff_mu_norm[min_index_diff_mu] = vec_diff_mu_norm[max_index_diff_mu];
      vec_sim_mu_value[max_index_sim_mu] = vec_sim_mu_value[min_index_sim_mu];
      vec_diff_mu_V_norm[min_index_diff_mu_V] = vec_diff_mu_V_norm[max_index_diff_mu_V];
      vec_sim_mu_V_value[max_index_sim_mu_V] = vec_sim_mu_V_value[min_index_sim_mu_V];
      vec_diff_mu_V_S_norm[min_index_diff_mu_V_S] = vec_diff_mu_V_S_norm[max_index_diff_mu_V_S];
      vec_sim_mu_V_S_value[max_index_sim_mu_V_S] = vec_sim_mu_V_S_value[min_index_sim_mu_V_S];
    }// for best match index

    destroyAllWindows();
    vec_diff_norm.clear();
    vec_sim_value.clear();
    vec_diff_mu_norm.clear();
    vec_sim_mu_value.clear();
    vec_diff_mu_V_norm.clear();
    vec_sim_mu_V_value.clear();
    vec_diff_mu_V_S_norm.clear();
    vec_sim_mu_V_S_value.clear();
  }// for loop I_new
}// end of recognition_test
