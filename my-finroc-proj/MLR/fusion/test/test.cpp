#include "projects/stereo_traversability_experiments/mlr/tMLR.h"
using namespace finroc::stereo_traversability_experiments::mlr;

/*int main*/void data_fusion(int argc, const char** argv)
{
  //  //  To horizontally concatenate two cv::Mat matrices (with the same number of rows) there is an undocumented function cv::hconcat().
  //   cv::Mat m1(10, 5, CV_32FC3);
  //   cv::Mat m2(10, 3, CV_32FC3);
  //   cv::Mat m3;
  //   cv::hconcat(m1, m2, m3);
  //   /*std::*/assert(10 == m3.rows && 8 == m3.cols);
  //   std::cout << m1 << std::endl;
  //   std::cout << m2 << std::endl;
  //   std::cout << m3 << std::endl;
  //
  //   cv::hconcat(std::vector<cv::Mat>{m1, m2}, m3);
  //   assert(10 == m3.rows && 8 == m3.cols);
  //
  //   cv::Mat matArray[] = { m1, m2 };
  //   cv::hconcat(matArray, 2, m3); // Accept array + size
  //   assert(10 == m3.rows && 8 == m3.cols);
  //
  //  //  The function cv::hconcat() has 2 additional overloads: One for accepting an array of cv::Mats and array size,
  //   // and another for accepting a cv::InputArrayOfArrays which is essentially an std::vector<cv::Mat>.
  //  //  Note that this function will copy all the elements of input matrices and (possibly) reallocate the output matrix.


  //////////////////////////////
  cv::Mat m1 = cv::Mat(10 /*rows*/, 1/*cols*/, CV_64FC1/*type*/, cv::Scalar(1)/*intial value or content*/);
  std::cout << m1 << std::endl;
  std::vector<cv::Mat> vec_mat;
  vec_mat.push_back(m1);
  //  cv::Mat m_vec(vec_mat);
  //  std::cout <<  m_vec << std::endl;
  // instantiate or making an instance of cv::Mat
  // create or construntion
  //initialization or assigning the value or clear ...
  cv::Mat m2 = cv::Mat(8/*rows*/, 1/*cols*/, CV_64FC1/*Opencv type, double, one channel mat*//*type*/, cv::Scalar(2)/*initial content or value or initialization*/);
  std::cout << m2 << std::endl;
  vec_mat.push_back(m2);
  cv::Mat m3 = cv::Mat(3, 1, CV_64FC1, cv::Scalar(3));
  std::cout << m3 << std::endl;
  mlr.LOG_PRINT(m3, "m3", eLOG_PRINT_STATE::eIS_ACTIVE);
  vec_mat.push_back(m3);
  cv::Mat m_all;
  cv::vconcat(vec_mat, m_all);
  std::cout << m_all << std::endl;
  mlr.LOG_PRINT(m_all, "m_all", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat /*instantiate */I_all = cv::Mat::zeros /*init*/(m_all.rows/*dimensionality of sensor data fused*//*rows*/,
                                  10/*number of samples*//*cols*/, m_all.type()) /*create*/;
  for (unsigned int j /*columns*/ = 0; j < 10; j++)
  {

    std::cout << I_all << std::endl;
    mlr.LOG_PRINT(I_all, "I_all", eLOG_PRINT_STATE::eIS_ACTIVE);

    //m_all.copyTo(I_all.col(j));
    // also we can use convert to as well
    m_all.convertTo(I_all.col(j), I_all.type()/*type*/, 1.0/*ratio*/);

    char key;
    cin >> key;
    if (key == 'n')
    {
      continue;
    }
  }



  //  //////////////////////////////
  //  cv::Mat m1 = cv::Mat(10 /*rows*/, 1/*cols*/, CV_64FC1/*type*/, cv::Scalar(1)/*intial value or content*/);
  //  std::cout << m1 << std::endl;
  //  std::vector<cv::Mat> vec_mat;
  //  vec_mat.push_back(m1);
  //  //  cv::Mat m_vec(vec_mat);
  //  //  std::cout <<  m_vec << std::endl;
  //  // instantiate or making an instance of cv::Mat
  //  // create or construntion
  //  //initialization or assigning the value or clear ...
  //  cv::Mat m2 = cv::Mat(8/*rows*/, 1/*cols*/, CV_64FC1/*Opencv type, double, one channel mat*//*type*/, cv::Scalar(2)/*initial content or value or initialization*/);
  //  std::cout << m2 << std::endl;
  //  vec_mat.push_back(m2);
  //  cv::Mat m3 = cv::Mat(3, 1, CV_64FC1, cv::Scalar(3));
  //  std::cout << m3 << std::endl;
  //  mlr.LOG_PRINT(m3, "m3", eLOG_PRINT_STATE::eIS_ACTIVE);
  //  vec_mat.push_back(m3);
  //  cv::Mat m_all;
  //  cv::vconcat(vec_mat, m_all);
  //  std::cout << m_all << std::endl;
  //  mlr.LOG_PRINT(m_all, "m_all", eLOG_PRINT_STATE::eIS_ACTIVE);
  //
  //  cv::Mat /*instantiate */I_all_fused = cv::Mat::zeros /*init*/(m_all.rows/*dimensionality of sensor data fused*//*rows*/,
  //                                  10/*number of samples*//*cols*/, m_all.type()) /*create*/;
  //  for (unsigned int j /*columns*/ = 0; j < 10; j++)
  //  {
  //
  //    std::cout << I_all_fused << std::endl;
  //    mlr.LOG_PRINT(I_all_fused, "I_all_fused", eLOG_PRINT_STATE::eIS_ACTIVE);
  //
  //    //m_all.copyTo(I_all_fused.col(j));
  //    // also we can use convert to as well
  //    m_all.convertTo(I_all_fused.col(j), I_all_fused.type()/*type*/, 1.0/*ratio*/);
  //
  //    char key;
  //    cin >> key;
  //    if (key == 'n')
  //    {
  //      continue;
  //    }
  //  }


  //////////////////////////////

  cv::Mat I_img_proj = cv::Mat(10 /*rows*/, 1/*cols*/, CV_64FC1/*type*/, cv::Scalar(1)/*intial value or content*/);
  std::cout << I_img_proj << std::endl;
  mlr.LOG_PRINT(I_img_proj, "I_img_proj", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat I_dist = cv::Mat(8/*rows*/, 1/*cols*/, CV_64FC1/*Opencv type, double, one channel mat*//*type*/, cv::Scalar(2)/*initial content or value or initialization*/);
  std::cout << I_dist << std::endl;
  mlr.LOG_PRINT(I_dist, "I_dist", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat I_pose = cv::Mat(3, 1, CV_64FC1, cv::Scalar(3));
  std::cout << I_pose << std::endl;
  mlr.LOG_PRINT(I_pose, "I_pose", eLOG_PRINT_STATE::eIS_ACTIVE);

  std::vector<cv::Mat> vec_I_all_fused = {I_img_proj, I_dist, I_pose};
  cv::Mat I_all_fused_col;
  cv::vconcat(vec_I_all_fused, I_all_fused_col);
  std::cout << I_all_fused_col << std::endl;
  mlr.LOG_PRINT(I_all_fused_col, "I_all_fused_col", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat /*instantiate */I_all_fused;

  for (unsigned int j /*columns*/ = 0; j < 10; j++)
  {

    //m_all.copyTo(I_all_fused.col(j));
    // also we can use convert to as well
    //m_all.convertTo(I_all_fused.col(j), I_all_fused.type()/*type*/, 1.0/*ratio*/);
    I_all_fused.push_back(I_all_fused_col);
    std::cout << I_all_fused << std::endl;
    mlr.LOG_PRINT(I_all_fused, "I_all_fused", eLOG_PRINT_STATE::eIS_ACTIVE);


    //    char key;
    //    cin >> key;
    //    if (key == 'n')
    //    {
    //      continue;
    //    }
  }

  I_all_fused = I_all_fused.clone().reshape(0, I_all_fused_col.rows);
  std::cout << I_all_fused << std::endl;
  std::cout << I_all_fused.col(0) << std::endl;
  std::cout << I_all_fused.row(0) << std::endl;
  mlr.LOG_PRINT(I_all_fused, "I_all_fused", eLOG_PRINT_STATE::eIS_ACTIVE);



  cv::Mat I_img_proj = cv::Mat(10 /*rows*/, 10/*cols*/, CV_64FC1/*type*/, cv::Scalar(1)/*intial value or content*/);
  std::cout << I_img_proj << std::endl;
  mlr.LOG_PRINT(I_img_proj, "I_img_proj", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat I_dist = cv::Mat(8/*rows*/, 10/*cols*/, CV_64FC1/*Opencv type, double, one channel mat*//*type*/, cv::Scalar(2)/*initial content or value or initialization*/);
  std::cout << I_dist << std::endl;
  mlr.LOG_PRINT(I_dist, "I_dist", eLOG_PRINT_STATE::eIS_ACTIVE);

  cv::Mat I_pose = cv::Mat(3, 10, CV_64FC1, cv::Scalar(3));
  std::cout << I_pose << std::endl;
  mlr.LOG_PRINT(I_pose, "I_pose", eLOG_PRINT_STATE::eIS_ACTIVE);

  std::vector<cv::Mat> vec_I_all_fused = {I_img_proj, I_dist, I_pose};
  cv::Mat I_all_fused_col;
  cv::vconcat(vec_I_all_fused, I_all_fused_col);
  std::cout << I_all_fused_col << std::endl;
  mlr.LOG_PRINT(I_all_fused_col, "I_all_fused_col", eLOG_PRINT_STATE::eIS_ACTIVE);

  //return 0; //exit
}// end of main function
