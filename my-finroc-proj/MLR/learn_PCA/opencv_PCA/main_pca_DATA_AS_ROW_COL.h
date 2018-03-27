/*
* pca.cpp
*
*  Author:
*  Kevin Hughes <kevinhughes27[at]gmail[dot]com>
*
*  Special Thanks to:
*  Philipp Wagner <bytefish[at]gmx[dot]de>
*
* This program demonstrates how to use OpenCV PCA with a
* specified amount of variance to retain. The effect
* is illustrated further by using a trackbar to
* change the value for retained varaince.
*
* The program takes as input a text file with each line
* begin the full path to an image. PCA will be performed
* on this list of images. The author recommends using
* the first 15 faces of the AT&T face data set:
* http://www.cl.cam.ac.uk/research/dtg/attarchive/facedatabase.html
*
* so for example your input text file would look like this:
*
*        <path_to_at&t_faces>/orl_faces/s1/1.pgm
*        <path_to_at&t_faces>/orl_faces/s2/1.pgm
*        <path_to_at&t_faces>/orl_faces/s3/1.pgm
*        <path_to_at&t_faces>/orl_faces/s4/1.pgm
*        <path_to_at&t_faces>/orl_faces/s5/1.pgm
*        <path_to_at&t_faces>/orl_faces/s6/1.pgm
*        <path_to_at&t_faces>/orl_faces/s7/1.pgm
*        <path_to_at&t_faces>/orl_faces/s8/1.pgm
*        <path_to_at&t_faces>/orl_faces/s9/1.pgm
*        <path_to_at&t_faces>/orl_faces/s10/1.pgm
*        <path_to_at&t_faces>/orl_faces/s11/1.pgm
*        <path_to_at&t_faces>/orl_faces/s12/1.pgm
*        <path_to_at&t_faces>/orl_faces/s13/1.pgm
*        <path_to_at&t_faces>/orl_faces/s14/1.pgm
*        <path_to_at&t_faces>/orl_faces/s15/1.pgm
*
*/

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

///////////////////////
// Functions


//static  Mat formatImagesForPCA(const vector<Mat> &data)
//{
//  Mat dst(static_cast<int>(data.size()), data[0].rows * data[0].cols, CV_32F);
//  for (unsigned int i = 0; i < data.size(); i++)
//  {
//    Mat image_mat_to_oneRow = data[i].clone().reshape(1, 1); // converting wxh to 1xwh -> this is reshaping process
//    Mat row_i = dst.row(i);
//    image_mat_to_oneRow.convertTo(row_i, CV_32F);
//  }
//  return dst;
//}

static  Mat formatImagesForPCA_row(const vector<Mat> &data)
{
//    Mat dst(static_cast<int>(data.size()), data[0].rows * data[0].cols, CV_32F);
  unsigned int num_rows = static_cast<int>(data.size());
  unsigned int num_cols = data[0].rows * data[0].cols;
  unsigned int X = num_cols /*width*/, Y = num_rows /*height*/;
  Size size(X, Y);
  const int type = CV_32F;
  Mat dst(size, type);
  for (unsigned int i = 0; i < data.size(); i++)
  {
    Mat image = data[i].clone();
    Mat image_mat_to_oneRow = image.reshape(1 /*one channel*/, 1 /*one row*/); // converting wxh to 1xwh -> this is reshaping process
    cout << "img_oneRow.size(): " << image_mat_to_oneRow.size() << endl;
    Mat row_i = dst.row(i);
    image_mat_to_oneRow.convertTo(row_i, CV_32F);
  }
  return dst;
}

static  Mat formatImagesForPCA_col(const vector<Mat> &data)
{
//    Mat dst(static_cast<int>(data.size()), data[0].rows * data[0].cols, CV_32F);
  unsigned int num_cols = static_cast<int>(data.size());
  unsigned int num_rows = data[0].rows * data[0].cols;
  unsigned int X = num_cols /*width*/, Y = num_rows /*height*/;
  Size size(X, Y);
  const int type = CV_32F;
  Mat dst(size, type);
  for (unsigned int i = 0; i < data.size(); i++)
  {
    Mat image = data[i].clone();
    Mat image_mat_to_oneCol = image.reshape(1 /*one channel*/, Y /*one row*/); // converting wxh to 1xwh -> this is reshaping process
    cout << "img_oneCol.size(): " << image_mat_to_oneCol.size() << endl;

    Mat col_i = dst.col(i);
    image_mat_to_oneCol.convertTo(col_i, CV_32F);
  }
  return dst;
}

static Mat convertImageMatToVector(const vector<Mat>& data)// changing image mat to column vector (NOT row vector as before)
{
  Mat img = data[0]; // data itslef is a vector or array
//    Mat dst(/*static_cast<int>(data.size())*/data.size(), data[0].total()/*data[0].rows * data[0].cols*/, data[0].type()/*CV_32F*/);
  unsigned int num_rows = img.total(), num_cols = data.size();
  Mat dst(num_rows, num_cols, img.type());
  for (/*unsigned*/ int i = 0; i < dst.cols/*data.size()*/; i++)
  {
//      const unsigned int new_num_channels = data[i].channels(), new_num_rows = num_rows;

    // number of channels for the loaded data has to be one before PCA
    Mat image = data[i].clone();
    Mat image_mat_to_oneRow = image.reshape(img.channels(), img.total());
    Mat /*row*/col_i = dst./*row*/col(i);  // this is where we load the images as the column of the output or dst matrix
    image_mat_to_oneRow.convertTo(col_i, data[0].type()/*CV_32F*/); // converting every image to a vector (Column vector)
  }
  return dst;
}

//static Mat toGrayscale(InputArray _src)
//{
//  Mat src = _src.getMat();
//  // only allow one channel
//  if (src.channels() != 1)
//  {
//    CV_Error(Error::StsBadArg, "Only Matrices with one channel are supported");
//  }
//  // create and return normalized image
//  Mat dst;
//  cv::normalize(_src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
//  return dst;
//}

//struct params
//{
//  Mat data;
//  int ch;
//  int rows;
//  PCA pca;
//  string winName;
//};

//static void onTrackbar(int pos, void* ptr)
//{
//  cout << "Retained Variance = " << pos << "%   ";
//  cout << "re-calculating PCA..." << std::flush;
//
//  double var = pos / 100.0;
//
//  struct params *p = (struct params *)ptr;
//
//  p->pca = PCA(p->data, cv::Mat(), PCA::DATA_AS_ROW, var);
//
//  Mat point = p->pca.project(p->data.row(0));
//  Mat reconstruction = p->pca.backProject(point);
//  reconstruction = reconstruction.reshape(p->ch, p->rows);
//  reconstruction = toGrayscale(reconstruction);
//
//  imshow(p->winName, reconstruction);
//  cout << "done!   # of principal components: " << p->pca.eigenvectors.rows << endl;
//}


///////////////////////
// Main
//int main(int argc, char** argv)
int main_pca(/*InputArray*/vector<Mat>& images)
{

  // Quit if there are not enough images for this demo.
  if (images.size() <= 1)
  {
    string error_message = "This demo needs at least 2 images to work. Please add more images to your data set!";
    CV_Error(Error::StsError, error_message);
  }


  // Testing the colMatrix as the stack of the images
  //Mat m_I_nxt = convertImageMatToVector(images); // n: Num Rows , NumDimensions(features and variables) and t: Num Cols , NumSamples //nxt == Rows_X_Cols
  Mat m_I_nxt_col_new = convertImageMatToVector(images); // n: Num Rows , NumDimensions(features and variables) and t: Num Cols , NumSamples //nxt == Rows_X_Cols
  Mat m_I_nxt_col = formatImagesForPCA_col(images); // n: Num Rows , NumDimensions(features and variables) and t: Num Cols , NumSamples //nxt == Rows_X_Cols
  Mat m_I_nxt_row = formatImagesForPCA_row(images); // n: Num Rows , NumDimensions(features and variables) and t: Num Cols , NumSamples //nxt == Rows_X_Cols

  /*  PCA pca(pcaset, // pass the data
            Mat(), // there is no pre-computed mean vector,
                   // so let the PCA engine to compute it
            CV_PCA_DATA_AS_ROW, // indicate that the vectors
                                // are stored as matrix rows
                                // (use CV_PCA_DATA_AS_COL if the vectors are
                                // the matrix columns)
            maxComponents // specify how many principal components to retain
            );*/

  /*const *//*unsigned */int /*maxComponents*/ numComponents = 1.00;
  PCA pca_nxt_col_new(m_I_nxt_col_new, Mat(), PCA::DATA_AS_COL, numComponents);
  PCA pca_nxt_col(m_I_nxt_col, Mat(), PCA::DATA_AS_COL, numComponents);
  PCA pca_nxt_row(m_I_nxt_row, Mat(), PCA::DATA_AS_ROW, numComponents);

  Mat m_I_nxt_mean = pca_nxt_col.mean; // this is only a vector
//  Mat m_I_nxt_mean = pca_nxt_row.mean; // this is only a vector
//  Mat m_I_nxt_mean = pca_nxt_col_new.mean; // this is only a vector
  cout << "new mean.size(): " << m_I_nxt_mean.size() << endl;
  m_I_nxt_mean = m_I_nxt_mean.reshape(images[0].channels(), images[0].rows);
  m_I_nxt_mean = toGrayscale(m_I_nxt_mean);
  imshow("m_I_nxt_mean", m_I_nxt_mean);


//  // Reshape and stack images into a rowMatrix
//  Mat data = formatImagesForPCA(images);
//
//  // perform PCA
//  PCA pca(data, cv::Mat(), PCA::DATA_AS_ROW, 0.95); // trackbar is initially set here, also this is a common value for retainedVariance
//
//  // Demonstration of the effect of retainedVariance on the first image
//  Mat point = pca.project(data.row(0)); // project into the eigenspace, thus the image becomes a "point"
//  Mat reconstruction = pca.backProject(point); // re-create the image from the "point"
//  reconstruction = reconstruction.reshape(images[0].channels(), images[0].rows); // reshape from a row vector into image shape
//  reconstruction = toGrayscale(reconstruction); // re-scale for displaying purposes
//
//  // copy the PCA results
//  Mat mean = pca.mean; //.reshape(1, 1); // store the mean vector
//  mean = mean.reshape(images[0].channels(), images[0].rows);
//  mean = toGrayscale(mean);
//  imshow("mean", mean);
//
//  Mat eigenvalues = pca.eigenvalues.clone(); // eigenvalues by row
//  Mat eigenvectors;
//  transpose(pca.eigenvectors, eigenvectors); // eigenvectors by column
//
//  // Display EigenFeatures or Principle Components
//  for (/*unsigned*/ int i = 0; i < eigenvectors.cols; i++)
//  {
//    // show the eigenvalues
//    string msg = format("Eigenvalues #%d = %0.5f", i, eigenvalues.at<double>(i));
//    cout << msg << endl;
//
//    // Reading the eigenvectors
//    Mat eigenFeature = eigenvectors.col(i).clone();
//
//    // Reshape from a vector to an image (original size) -> rows and channels should be added to it
//    eigenFeature = eigenFeature.reshape(images[0].channels(), images[0].rows); // now it s only a colVector -> needs rows & channels for original shape
//
//    // normalize to 0-255 for display and grayscale conversion
//    eigenFeature = toGrayscale(eigenFeature);
//
//    // Displaying it with its order with respect to WRT its eigenvalues
//    string name = format("EigenFeature_%d", i);
//    imshow(name, eigenFeature);
//  }
//
//
//  // init highgui window
//  string winName = "Reconstruction | press 'q' to quit";
//  namedWindow(winName, WINDOW_NORMAL);
//
//  // params struct to pass to the trackbar handler
//  params p;
//  p.data = data;
//  p.ch = images[0].channels();
//  p.rows = images[0].rows;
//  p.pca = pca;
//  p.winName = winName;
//
//  // create the tracbar
//  int pos = 95;
//  createTrackbar("Retained Variance (%)", winName, &pos, 100, onTrackbar, (void*)&p);
//
//  // display until user presses q
//  imshow(winName, reconstruction);

  int key = 0;
  while (key != 'q')
    key = waitKey();

  return 0;
}
