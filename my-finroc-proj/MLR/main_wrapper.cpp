#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace std;
using namespace cv;
using namespace finroc::stereo_traversability_experiments::mlr;

int main(int argc, const char** argv)
{
  // memory, learn and recognition
  tMLR mlr;

  /*! Data Collection reading input data from memory & collecting the data*/
  /*! Data Collection reading input data from memory & collecting the data*/
  vector<Mat> I_img;
  mlr.read_directory_recursively(argc, argv, I_img);

  /*! Data TYPE Scaling -- data type (CV8UC1 UNISGNED CHAR 256 for IMAGES */
  /*! Data TYPE Scaling -- data type (CV8UC1 UNISGNED CHAR 256 for IMAGES */
  /*! Data TYPE Scaling -- data type (CV8UC1 UNISGNED CHAR 256 for IMAGES */
  /*! Data TYPE Scaling -- data type (CV8UC1 UNISGNED CHAR 256 for IMAGES */
  vector<Mat> I_img_scaled; // this is the output so it can NOT be constant -- ONLY THE INPUT DATA IS CONSTANT
  mlr.toNormalize(I_img, I_img_scaled, eDATA_TYPE::eIMAGE_DATA);
  cv::Mat I_img_0(I_img[0]); // This is only one image sample for its size for showing and visualizing the result
  I_img.clear(); //release extra RAM and cache

  /*! Scaled Data Vectorization -- data type (CV64C1 DOUBLE precision -- MAX precision POSSIBLE and tested for the implemented EVD) */
  /*! Scaled Data Vectorization -- data type (CV64C1 DOUBLE precision -- MAX precision POSSIBLE and tested for the implemented EVD) */
  /*! Scaled Data Vectorization -- data type (CV64C1 DOUBLE precision -- MAX precision POSSIBLE and tested for the implemented EVD) */
  /*! Data STRUCTURE for space analysis - Vectorizing data samples and store them in a Matrix (COL_AS_DATA)
   * for creating the space and enabling space analysis (PCA, SVD, EVD && Eigen Space Analysis) */
  Mat I_img_vec = mlr.vectorize_input_data(I_img_scaled);
  I_img_scaled.clear(); //release RAM and allocated memory
  mlr.LOG_PRINT(I_img_vec, "I_img_vec: ", eLOG_PRINT_STATE::eIS_ACTIVE);

  /*! Learning & Space Analysis for Eigen Space Analysis */
  /*! Learning & Space Analysis for Eigen Space Analysis */
  std::string user_input;
  //cout << "Please enter if you want to LEARN the folder (yes) or recognize it (no): " << endl;
  std::cout << "options: learn or recognition" << std::endl;
  cin >> user_input;
  cout << "Your choice was: " << user_input <<  " -----------------------> please REPEAT your answer" << endl;
  user_input.clear(); // empty it again
  cin >> user_input;
  if (user_input == "learn")
  {
    cout << "Learning..............................................................................." << endl;
    mlr.learn(I_img_vec, I_img_0, eDATA_TYPE::eIMAGE_DATA, argv);
  }// if
  if (user_input == "recognition")
  {
    //cout << " should be already LEARNEDDDDDDDDDDDDDDDDDDDDDDDDDDD" << endl;
    /*! Recognition & comparing the new input data with our LEARNED database */
    /*! Recognition & comparing the new input data with our LEARNED database */
    mlr.recognize_test(I_img_vec, I_img_vec, I_img_0, argv, eDATA_TYPE::eIMAGE_DATA); // read the learned model from memory
    I_img_vec.release(); // release RAM and CACHE
    I_img_0.release(); // release RAM & CACHE
  }// else


  return 0; //exit
}// end of main function
