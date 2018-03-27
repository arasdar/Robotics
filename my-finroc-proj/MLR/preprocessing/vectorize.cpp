#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace cv;

/*static */Mat finroc::stereo_traversability_experiments::mlr::tMLR::vectorize_input_data(/*INPUT*/const vector<Mat>& input_data_scaled)// changing image mat to column vector (NOT row vector as before)
{
  unsigned int rows = input_data_scaled[0].total(),
               cols = input_data_scaled.size();
  unsigned int type = CV_64FC1; /* DOUBLE and MAXIMUM precision*/
  Mat I_vec(rows, cols, type);
  for (/*unsigned*/ int j = 0; j < I_vec.cols; j++) // i is usually used either for ITERATION or ROWS
  {
    Mat scaled_sample = input_data_scaled[j].clone();
    Mat scaled_sample_vectorized = scaled_sample.reshape(0, /*total_number_of_pixels*/ scaled_sample.total());
    Mat /*row*/col_j = I_vec./*row*/col(j);  // this is where we load the images as the column of the output or I_vec matrix
    scaled_sample_vectorized.convertTo(col_j, type); // converting every image to a vector (Column vector)
  }
  return I_vec;
}// end of function

/*static */Mat finroc::stereo_traversability_experiments::mlr::tMLR::vectorize_input_data(/*INPUT*/const cv::Mat& input_data_scaled)// changing image mat to column vector (NOT row vector as before)
{
  unsigned int rows = input_data_scaled.total(),
               cols = 1/*input_data_scaled.size()*/;
  unsigned int type = CV_64FC1; /* DOUBLE and MAXIMUM precision*/
  Mat I_vec(rows, cols, type);
  Mat data_col = input_data_scaled.clone().reshape(0, /*total_number_of_pixels*/ rows);
  data_col.convertTo(I_vec/*col_j*/, type); // converting every image to a vector (Column vector)
  return I_vec;
}// end of function

///*static*/  Mat finroc::stereo_traversability_experiments::mlr::tMLR::formatImagesForPCA(const vector<Mat> &data)
//{
//  Mat dst(static_cast<int>(data.size()), data[0].rows * data[0].cols, CV_32F);
//  for (unsigned int i = 0; i < data.size(); i++)
//  {
//    Mat image_row = data[i].clone().reshape(1, 1);
//    Mat row_i = dst.row(i);
//    image_row.convertTo(row_i, CV_32F);
//  }
//  return dst;
//
//  /*! This is how to use it
//    // Reshape and stack images into a rowMatrix
//    Mat data = formatImagesForPCA(images);
//
//    // perform PCA
//    PCA pca(data, cv::Mat(), PCA::DATA_AS_ROW, 0.95); // trackbar is initially set here, also this is a common value for retainedVariance
//  */
//
//}
//
//
//// Converts the images given in src into a row matrix.
//Mat finroc::stereo_traversability_experiments::mlr::tMLR::asRowMatrix(const vector<Mat>& src, int rtype, double alpha = 1, double beta = 0)
//{
//  // Number of samples:
//  size_t n = src.size(); // this is UNSIGNED (0-255) -- 8U IN OPENCV
//  // Return empty matrix if no matrices given:
//  if (n == 0)
//    return Mat();
//  // dimensionality of (reshaped) samples
//  size_t d = src[0].total();
//  // Create resulting data matrix:
//  Mat data(n, d, rtype);
//  // Now copy data:
//  for (unsigned int i = 0; i < n; i++)
//  {
//    //
//    if (src[i].empty())
//    {
//      string error_message = format("Image number %d was empty, please check your input data.", i);
//      CV_Error(CV_StsBadArg, error_message);
//    }
//    // Make sure data can be reshaped, throw a meaningful exception if not!
//    if (src[i].total() != d)
//    {
//      string error_message = format("Wrong number of elements in matrix #%d! Expected %d was %d.", i, d, src[i].total());
//      CV_Error(CV_StsBadArg, error_message);
//    }
//    // Get a hold of the current row:
//    Mat xi = data.row(i);
//    // Make reshape happy by cloning for non-continuous matrices:
//    if (src[i].isContinuous())
//    {
//      src[i].reshape(1, 1).convertTo(xi, rtype, alpha, beta);
//    }
//    else
//    {
//      src[i].clone().reshape(1, 1).convertTo(xi, rtype, alpha, beta);
//    }
//  }
//
//  /*! This is how to use it
//    // Build a matrix with the observations in row:
//    Mat data = asRowMatrix(db, CV_32FC1);
//
//    // Number of components to keep for the PCA:
//    int num_components = 10;
//
//    // Perform a PCA:
//    PCA pca(data, Mat(), CV_PCA_DATA_AS_ROW, num_components);
//  */
//
//  return data;
//}
