//Udacity HW2 Driver

#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "../timer.h"
#include "../utils.h"

using namespace cv;
using namespace std;

#define PI 3.141592653589793238463

cv::Mat G_imageInputGray_1;
cv::Mat G_imageInputGray_2;
cv::Mat G_imageOutputGray;

uchar *d_inputImageGray_1__;
uchar *d_inputImageGray_2__;
uchar *d_outputImageGray__;//_1__;

float *GPU_gw_average_color_1_;
float *GPU_gw_auto_correlation_1_;
float *GPU_gw_average_color_2_;
float *GPU_gw_auto_correlation_2_;
float *GPU_gw_cross_correlation_3_;
float *GPU_gw_normalized_score_4_;

int *match_matrix_;
int *disparity_map_;
int *depth_map_;

int numRows;
int numCols;

float *h_filter__;

/*******  DEFINED IN student_func.cu *********/
void your_gaussian_blur(const uchar* const h_inputImageGray_1,
                        uchar* const d_inputImageGray_1,
                        uchar* const h_inputImageGray_2,
                        uchar* const d_inputImageGray_2,
                        uchar* const d_outputImageGray,
                        float* const GPU_gw_average_color_1,
                        float* const GPU_gw_auto_correlation_1,
                        float* const GPU_gw_average_color_2,
                        float* const GPU_gw_auto_correlation_2,
                        float* const GPU_gw_cross_correlation_3,
                        float* const GPU_gw_normalized_score_4,
                        int* match_matrix,
                        int* disparity_map,
                        int* depth_map,
                        const int numRows, const int numCols,
                        const int filterWidth);

void allocateMemoryAndCopyToGPU(const int numRows, const int numCols,
                                const float* const h_filter, const int filterWidth);

/************postprocess*******/
void GPU_postProcess(const std::string& output_file, uchar* data_ptr)
{
  cv::Mat output(numRows, numCols, CV_8UC1, (void*)data_ptr);

  Mat img_hist_equalized;
  equalizeHist(output, img_hist_equalized);
  printf("%s\n", "hi karen5");
  //cv::Mat imageOutputBGR;
  //cv::cvtColor(output, imageOutputBGR, CV_GRAY2BGR);//CV_RGBA2BGR);
  //output the image
  //cv::imwrite(output_file.c_str(), imageOutputBGR);
  cv::imwrite(output_file.c_str(), img_hist_equalized);
}


//************************************
void GPU_preProcess(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2, uchar **h_outputImageGray,
                    uchar **d_inputImageGray_1, uchar **d_inputImageGray_2, uchar **d_outputImageGray,
                    float **GPU_gw_average_color_1, float **GPU_gw_auto_correlation_1,
                    float **GPU_gw_average_color_2, float **GPU_gw_auto_correlation_2,
                    float **GPU_gw_cross_correlation_3, float **GPU_gw_normalized_score_4,
                    int **match_matrix, int **disparity_map, int **depth_map,
                    //unsigned char **d_redBlurred,
                    //unsigned char **d_redBlurred_1,
                    //unsigned char **d_greenBlurred,
                    //unsigned char **d_greenBlurred_1,
                    //unsigned char **d_blueBlurred,
                    //unsigned char **d_blueBlurred_1,
                    float **h_filter, int *filterWidth,
                    const std::string &filename_1, const std::string &filename_2)
{


  //make sure the context initializes ok
  //checkCudaErrors(cudaFree(0));

  cv::Mat image_1 = cv::imread(filename_1.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::Mat image_2 = cv::imread(filename_2.c_str(), CV_LOAD_IMAGE_COLOR);

  if (image_1.empty() || image_2.empty())
  {
    std::cerr << "Couldn't open file: " << filename_1 << "or" << filename_2 << std::endl;
    exit(1);
  }

  //copy the transformed image from first parameter to the second one
  cv::cvtColor(image_1, G_imageInputGray_1, CV_BGR2GRAY);
  cv::cvtColor(image_2, G_imageInputGray_2, CV_BGR2GRAY);

  //allocate memory for the output
  G_imageOutputGray.create(image_1.rows, image_1.cols, CV_8UC1);

  //This shouldn't ever happen given the way the images are created
  //at least based upon my limited understanding of OpenCV, but better to check
  if (!G_imageInputGray_1.isContinuous() || !G_imageOutputGray.isContinuous() ||
      !G_imageInputGray_2.isContinuous())
  {
    std::cerr << "Images aren't continuous!! Exiting." << std::endl;
    exit(1);
  }

  *h_inputImageGray_1  = (unsigned char *)G_imageInputGray_1.ptr<unsigned char>(0);
  *h_inputImageGray_2  = (unsigned char *)G_imageInputGray_2.ptr<unsigned char>(0);
  *h_outputImageGray = (unsigned char *)G_imageOutputGray.ptr<unsigned char>(0);

  numRows = G_imageInputGray_1.rows;
  numCols = G_imageInputGray_1.cols;
  const int numPixels = numRows * numCols;

  //allocate memory on the device for both input and output
  //checkCudaErrors(   cudaMalloc((void**)d_inputImageGray_1, sizeof(unsigned char) * numPixels)   );

  //cudaError_t t1 = cudaGetLastError();

  //cout << cudaGetErrorString(t1) << endl;

  //char c;
  //cin >> c;

  checkCudaErrors(cudaMalloc((void**)d_inputImageGray_1, sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMalloc((void**)d_inputImageGray_2, sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMalloc((void**)d_outputImageGray, sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMemset((void*)*d_outputImageGray, 0,  sizeof(unsigned char) * numPixels)); //make sure no memory is left laying around

  checkCudaErrors(cudaMalloc((void**)GPU_gw_average_color_1,    sizeof(float) * numPixels));
  checkCudaErrors(cudaMalloc((void**)GPU_gw_auto_correlation_1,  sizeof(float) * numPixels));
  checkCudaErrors(cudaMalloc((void**)GPU_gw_average_color_2,   sizeof(float) * numPixels));
  checkCudaErrors(cudaMalloc((void**)GPU_gw_auto_correlation_2,  sizeof(float) * numPixels));
  checkCudaErrors(cudaMemset((void*)*GPU_gw_average_color_1,   0, sizeof(float) * numPixels));
  checkCudaErrors(cudaMemset((void*)*GPU_gw_auto_correlation_1, 0, sizeof(float) * numPixels));
  checkCudaErrors(cudaMemset((void*)*GPU_gw_average_color_2,  0, sizeof(float) * numPixels));
  checkCudaErrors(cudaMemset((void*)*GPU_gw_auto_correlation_2, 0, sizeof(float) * numPixels));

  checkCudaErrors(cudaMalloc((void**)GPU_gw_cross_correlation_3,   sizeof(float) * numPixels));
  checkCudaErrors(cudaMalloc((void**)GPU_gw_normalized_score_4,  sizeof(float) * numPixels));
  checkCudaErrors(cudaMemset((void*)*GPU_gw_cross_correlation_3, 0, sizeof(float) * numPixels));
  checkCudaErrors(cudaMemset((void*)*GPU_gw_normalized_score_4, 0, sizeof(float) * numPixels));

  checkCudaErrors(cudaMalloc((void**)match_matrix,  sizeof(int) * numPixels));
  checkCudaErrors(cudaMalloc((void**)disparity_map,  sizeof(int) * numPixels));
  checkCudaErrors(cudaMalloc((void**)depth_map,  sizeof(int) * numPixels));
  checkCudaErrors(cudaMemset((void*) *match_matrix, 0,  sizeof(int) * numPixels));
  checkCudaErrors(cudaMemset((void*)*disparity_map, 0,  sizeof(int) * numPixels));
  checkCudaErrors(cudaMemset((void*)*depth_map, 0,  sizeof(int) * numPixels));

  //copy input array to the GPU
  checkCudaErrors(cudaMemcpy(*d_inputImageGray_1, *h_inputImageGray_1, sizeof(uchar) * numPixels, cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(*d_inputImageGray_2, *h_inputImageGray_2, sizeof(uchar) * numPixels, cudaMemcpyHostToDevice));

  d_inputImageGray_1__ = *d_inputImageGray_1;
  d_inputImageGray_2__ = *d_inputImageGray_2;
  d_outputImageGray__ = *d_outputImageGray;

  GPU_gw_average_color_1_ = *GPU_gw_average_color_1;
  GPU_gw_auto_correlation_1_ = *GPU_gw_auto_correlation_1;
  GPU_gw_average_color_2_ = *GPU_gw_average_color_2;
  GPU_gw_auto_correlation_2_ = *GPU_gw_auto_correlation_2;
  GPU_gw_cross_correlation_3_ = *GPU_gw_cross_correlation_3;
  GPU_gw_normalized_score_4_ = *GPU_gw_normalized_score_4;
  match_matrix_ = *match_matrix;
  disparity_map_ = *disparity_map;
  depth_map_ = *depth_map;



  //now create the filter that they will use
  const int blurKernelWidth = 7;
  const float blurKernelSigma = 2.;

  *filterWidth = blurKernelWidth;

  //create and fill the filter we will convolve with
  *h_filter = new float[blurKernelWidth * blurKernelWidth];
  h_filter__ = *h_filter;

  float filterSum = 0.f; //for normalization

  for (int r = -blurKernelWidth / 2; r <= blurKernelWidth / 2; ++r)
  {
    for (int c = -blurKernelWidth / 2; c <= blurKernelWidth / 2; ++c)
    {
      float filterValue = pow(cos(r * PI / blurKernelWidth), 2.0) * pow(cos(c * PI / blurKernelWidth), 2.0);
      //float filterValue = expf( -(float)(c * c + r * r) / (2.f * blurKernelSigma * blurKernelSigma));
      (*h_filter)[(r + blurKernelWidth / 2) * blurKernelWidth + c + blurKernelWidth / 2] = filterValue;
      //filterSum += filterValue;
    }
  }
  /*
  float normalizationFactor = 1.f / filterSum;

  for (int r = -blurKernelWidth/2; r <= blurKernelWidth/2; ++r) {
    for (int c = -blurKernelWidth/2; c <= blurKernelWidth/2; ++c) {
      (*h_filter)[(r + blurKernelWidth/2) * blurKernelWidth + c + blurKernelWidth/2] *= normalizationFactor;
    }
  }
  //blurred
  checkCudaErrors(cudaMalloc(d_redBlurred,    sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMalloc(d_greenBlurred,  sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMalloc(d_blueBlurred,   sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMemset(*d_redBlurred,   0, sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMemset(*d_greenBlurred, 0, sizeof(unsigned char) * numPixels));
  checkCudaErrors(cudaMemset(*d_blueBlurred,  0, sizeof(unsigned char) * numPixels));

  */
}

/**********cleanUp**********/
void GPU_cleanUp(void)
{

  checkCudaErrors(cudaFree(d_inputImageGray_1__));
  checkCudaErrors(cudaFree(d_inputImageGray_2__));
  checkCudaErrors(cudaFree(d_outputImageGray__));
  checkCudaErrors(cudaFree(GPU_gw_average_color_1_));
  checkCudaErrors(cudaFree(GPU_gw_auto_correlation_1_));
  checkCudaErrors(cudaFree(GPU_gw_average_color_2_));
  checkCudaErrors(cudaFree(GPU_gw_auto_correlation_2_));
  checkCudaErrors(cudaFree(GPU_gw_cross_correlation_3_));
  checkCudaErrors(cudaFree(GPU_gw_normalized_score_4_));
  checkCudaErrors(cudaFree(match_matrix_));
  checkCudaErrors(cudaFree(disparity_map_));
  checkCudaErrors(cudaFree(depth_map_));


  delete[] h_filter__;

}

//***********************
void GPU_Part(int argc, char **argv)
{

  uchar *h_inputImageGray_1, *h_inputImageGray_2, *d_inputImageGray_1, *d_inputImageGray_2;
  uchar *h_outputImageGray, *d_outputImageGray;
  //unsigned char *d_GrayBlurred,*d_GrayBlurred_1;//,*d_redBlurred,*d_redBlurred_1, *d_greenBlurred,*d_greenBlurred_1, *d_blueBlurred,*d_blueBlurred_1;

  float *GPU_gw_average_color_1; //  = new float[numRows * numCols];
  float *GPU_gw_auto_correlation_1; //  = new float[numRows * numCols];
  float *GPU_gw_average_color_2; //  = new float[numRows * numCols];
  float *GPU_gw_auto_correlation_2; //  = new float[numRows * numCols];
  float *GPU_gw_cross_correlation_3; //  = new float*[numRows * numCols];
  float *GPU_gw_normalized_score_4; //  = new float*[numRows * numCols];

  int *match_matrix;//=new int[numRows*numCols];
  int *disparity_map;//=new int [numRows*numCols];
  int *depth_map;//=new int [numRows*numCols];

  /*
  for (int r = 0; r < (int)numRows; ++r) {
    for (int c = 0; c < (int)numCols; ++c) {
      float image_value = static_cast<float>(channel[r * numCols + c]);
    }
  }*/

  float *h_filter;
  int    filterWidth;
  std::string input_file_1;
  std::string input_file_2;
  std::string output_file;
  std::string output_file_2;
  std::string reference_file;
  double perPixelError = 0.0;
  double globalError   = 0.0;
  bool useEpsCheck = false;
  switch (argc)
  {
  case 2:
    input_file_1 = std::string(argv[1]);
    output_file = "nwcc_output_1.png";
    output_file_2 = "nwcc_output_2.png";
    reference_file = "nwcc_reference.png";
    break;
  case 3:
    input_file_1  = std::string(argv[1]);
    input_file_2 = std::string(argv[2]);
    output_file = "nwcc_output_1.png";
    output_file_2 = "nwcc_output_2.png";
    reference_file = "nwcc_reference.png";
    break;
  case 4:
    input_file_1  = std::string(argv[1]);
    output_file = std::string(argv[2]);
    reference_file = std::string(argv[3]);
    break;
  case 6:
    useEpsCheck = true;
    input_file_1  = std::string(argv[1]);
    output_file = std::string(argv[2]);
    reference_file = std::string(argv[3]);
    perPixelError = atof(argv[4]);
    globalError   = atof(argv[5]);
    break;
  default:
    std::cerr << "Usage: ./nwcc input_file_1  input_file_2  [output_filename] [reference_filename] [perPixelError] [globalError]" << std::endl;
    exit(1);
  }
  GPU_preProcess(&h_inputImageGray_1, &h_inputImageGray_2, &h_outputImageGray,
                 &d_inputImageGray_1, &d_inputImageGray_2, &d_outputImageGray,
                 &GPU_gw_average_color_1, &GPU_gw_auto_correlation_1,
                 &GPU_gw_average_color_2, &GPU_gw_auto_correlation_2,
                 &GPU_gw_cross_correlation_3, &GPU_gw_normalized_score_4,
                 &match_matrix, &disparity_map, &depth_map,
                 &h_filter, &filterWidth, input_file_1, input_file_2);

  //load the image and give us our input and output pointers
  //preProcess(&h_inputImageRGBA, &h_outputImageRGBA, &h_outputImageRGBA_1, &d_inputImageRGBA, &d_outputImageRGBA, &d_outputImageRGBA_1,
  //           &d_redBlurred,&d_redBlurred_1, &d_greenBlurred,&d_greenBlurred_1, &d_blueBlurred,&d_blueBlurred_1,
  //           &h_filter, &filterWidth, input_file);

  allocateMemoryAndCopyToGPU(numRows, numCols, h_filter, filterWidth);
  GpuTimer timer;
  timer.Start();

  //your_gaussian_blur(h_inputImageRGBA, d_inputImageRGBA, d_outputImageRGBA,d_outputImageRGBA_1, numRows, numCols,
  //                   d_redBlurred,d_redBlurred_1, d_greenBlurred,d_greenBlurred_1, d_blueBlurred,d_blueBlurred_1, filterWidth);

  your_gaussian_blur(h_inputImageGray_1, d_inputImageGray_1,
                     h_inputImageGray_2, d_inputImageGray_2, d_outputImageGray,
                     GPU_gw_average_color_1, GPU_gw_auto_correlation_1,
                     GPU_gw_average_color_2, GPU_gw_auto_correlation_2,
                     GPU_gw_cross_correlation_3, GPU_gw_normalized_score_4,
                     match_matrix, disparity_map, depth_map,
                     numRows, numCols, filterWidth);

  timer.Stop();
  cudaDeviceSynchronize();
  checkCudaErrors(cudaGetLastError());
  int err = printf("Your code ran in: %f msecs.\n", timer.Elapsed());

  if (err < 0)
  {
    //Couldn't print! Probably the student closed stdout - bad news
    std::cerr << "Couldn't print timing information! STDOUT Closed!" << std::endl;
    exit(1);
  }

  //check results and output the blurred image
  int numPixels = numRows * numCols;
  //
  //copy the output back to the host
  checkCudaErrors(cudaMemcpy(h_outputImageGray, d_outputImageGray__, sizeof(uchar) * numPixels, cudaMemcpyDeviceToHost));
  //checkCudaErrors(cudaMemcpy(h_outputImageRGBA_1, d_outputImageRGBA_1__, sizeof(uchar4) * numPixels, cudaMemcpyDeviceToHost));

  GPU_postProcess(output_file, h_outputImageGray);
  //postProcess(output_file_1, h_outputImageRGBA_1);

  //GPU_referenceCalculation(h_inputImageGray_1,h_inputImageGray_2, h_outputImageGray_1,h_outputImageGray_2,numRows, numCols,h_filter, filterWidth);

  //GPU_postProcess(output_file_1, h_outputImageGray_1);
  //CPU_postProcess(output_file_2, h_outputImageGray_2);

  //CPU_postProcess(reference_file, h_outputImageRGBA);
  //  Cheater easy way with OpenCV
  //generateReferenceImage(input_file, reference_file, filterWidth);
  //compareImages(reference_file, output_file, useEpsCheck, perPixelError, globalError);

  //checkCudaErrors(cudaFree(d_inputImageGray_1));
  //checkCudaErrors(cudaFree(d_inputImageGray_2));
  //checkCudaErrors(cudaFree(d_outputImageGray));
  //checkCudaErrors(cudaFree(d_outputImageGray__));

  GPU_cleanUp();
}

int main(int argc, char **argv)
{

  GPU_Part(argc, argv);

  return 0;
}
