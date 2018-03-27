
//#include "projects/stereo_traversability_experiments/aras/libstereo_test/gpu_finroc_test/gpu_part.h"
#include "gpu_part.h"

namespace finroc
{
namespace stereo_traversability_experiments
{
namespace aras
{
namespace libstereo_test
{
namespace gpu
{

gpu_part::gpu_part() {}

gpu_part::~gpu_part() {}

//************************************
void gpu_part::GPU_preProcess(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2, uchar **h_outputImageGray,
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
void gpu_part::GPU_cleanUp(void)
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
void gpu_part::GPU_run(int argc, char **argv)
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

/************postprocess*******/
void gpu_part::GPU_postProcess(const std::string& output_file, uchar* data_ptr)
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


__global__ void Kernel_1(const unsigned char* const inputChannel_1, //inputChannel
                         const unsigned char* const inputChannel_2,
                         unsigned char* const outputChannel,
                         float* const GPU_gw_average_color_1,
                         float* const GPU_gw_average_color_2,
                         float* const GPU_gw_auto_correlation_1,
                         float* const GPU_gw_auto_correlation_2,
                         int numRows, int numCols,
                         const float* const filter, const int filterWidth, int mid_shared_address)
{
  // TODO

  // NOTE: Be sure to compute any intermediate results in floating point
  // before storing the final result as unsigned char.

  // NOTE: Be careful not to try to access memory that is outside the bounds of
  // the image. You'll want code that performs the following check before accessing
  // GPU memory:
  //
  // if ( absolute_image_position_x >= numCols ||
  //      absolute_image_position_y >= numRows )
  // {
  //     return;
  // }
  int x = threadIdx.x + blockIdx.x * blockDim.x;
  int y = threadIdx.y + blockIdx.y * blockDim.y;
  int thread_1D = x + y * numCols;
  //printf("the x= %d y= %d  and  threadID %d inputvalue %d \n",x,y,thread_1D,inputChannel[thread_1D] );

  extern __shared__ unsigned char temp_1[];

  unsigned char* pointer = (unsigned char*)temp_1;
  unsigned char* temp_2 = (unsigned char*)&pointer[mid_shared_address];

  //first batch transfer
  //temp_1[(threadIdx.x+filterWidth/2)+(threadIdx.y+filterWidth/2)*(blockDim.x+2*(filterWidth/2))]=inputChannel[thread_1D];
  int dest = threadIdx.y * blockDim.x + threadIdx.x, // index(in a one dimentional array) of a thread(pixel) in a block
      destY = dest / (blockDim.x + 2 * (filterWidth / 2)), // y index of destination in shared memory
      destX = dest % (blockDim.x + 2 * (filterWidth / 2)), // x index of destination in shared memory
      srcY = blockIdx.y * blockDim.y + destY - (filterWidth / 2), // y index in source image (-filterwidth/2 because of overlaping)
      srcX = blockIdx.x * blockDim.x + destX - (filterWidth / 2), // x index in source image (-filterwidth/2 because of overlaping)
      src = srcY * numCols + srcX;// index(in a one dimentional array) of a thread(pixel) in the source image

  //printf("threadIdx.x= %d threadIdx.y= %d blockIdx.x= %d blockIdx.y= %d \n",threadIdx.x,threadIdx.y,blockIdx.x,blockIdx.y);
  //printf("dest= %d destY= %d destX= %d srcY= %d srcX= %d src %d \n",dest,destY,destX,srcY,srcX,src );
  //int help_1 = destY*(blockDim.x+2*(filterWidth/2))+destX;
  if (srcY >= 0 && srcY < numRows && srcX >= 0 && srcX < numCols)
  {
    temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[src];
    temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[src];
    //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
  }
  else
  {
    //temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX] =0; //;
    //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
    if (srcY < 0)
    {
      srcY = 0;
    }
    if (srcY >= numRows)
    {
      srcY = numRows - 1 ;
    }
    if (srcX < 0)
    {
      srcX = 0;
    }
    if (srcX >= numCols)
    {
      srcX = numCols - 1 ;
    }
    int newindex = srcY * numCols + srcX;

    temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[newindex];
    //printf("destY*(blockDim.x+2*(filterWidth/2))+destX= %d \n",destY*(blockDim.x+2*(filterWidth/2))+destX);
    temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[newindex];
  }

//Second batch loading
  dest = threadIdx.y * blockDim.x + threadIdx.x + blockDim.x * blockDim.y;
  destY = dest / (blockDim.x + 2 * (filterWidth / 2));
  destX = dest % (blockDim.x + 2 * (filterWidth / 2));
  srcY = blockIdx.y * blockDim.y + destY - (filterWidth / 2);
  srcX = blockIdx.x * blockDim.x + destX - (filterWidth / 2);
  src =  srcY * numCols + srcX;

  //printf("threadIdx.x= %d threadIdx.y= %d blockIdx.x= %d blockIdx.y= %d \n",threadIdx.x,threadIdx.y,blockIdx.x,blockIdx.y);
  //printf("dest= %d destY= %d destX= %d srcY= %d srcX= %d src %d \n",dest,destY,destX,srcY,srcX,src );
  if (destY < (blockDim.y + 2 * (filterWidth / 2)))
  {
    //int help_2 = destY*(blockDim.x+2*(filterWidth/2))+destX;
    if (srcY >= 0 && srcY < numRows && srcX >= 0 && srcX < numCols)
    {
      temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[src];
      temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[src];
      //printf("temp_1[help_2= %d]= %d \n",help_2,temp_1[help_2]);
    }
    else
    {
      if (srcY < 0)
      {
        srcY = 0;
      }
      if (srcY >= numRows)
      {
        srcY = numRows - 1 ;
      }
      if (srcX < 0)
      {
        srcX = 0;
      }
      if (srcX >= numCols)
      {
        srcX = numCols - 1 ;
      }
      int newindex2 = srcY * numCols + srcX;
      temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[newindex2];
      temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[newindex2];
      //printf("temp_2[destY*(blockDim.x+2*(filterWidth/2))+destX= %d]= %d , %d \n",destY*(blockDim.x+2*(filterWidth/2))+destX,(temp_2[destY*(blockDim.x+2*(filterWidth/2))+destX]), inputChannel_2[newindex2] );

      //printf("temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX= %d]= %d , %d \n",destY*(blockDim.x+2*(filterWidth/2))+destX,(temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX]), inputChannel_1[newindex2] );
    }
  }
  __syncthreads();

// int temp1_size=(blockDim.x+(2*(filterWidth/2)))*(blockDim.y+(2*(filterWidth/2)))*sizeof(char);

  float result_1 = 0.0f;
  float result_2 = 0.0f;
  //#pragma unroll 16
  for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; filter_r++)
  {
    //printf("hello babak %d \n",filter_r+filterWidth/2);
    for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; filter_c++)
    {

      float image_value_1 = static_cast<float>(temp_1[(threadIdx.y + filter_r + filterWidth / 2) * (blockDim.x + 2 * (filterWidth / 2)) + threadIdx.x + filter_c + filterWidth / 2]);
      float image_value_2 = static_cast<float>(temp_2[(threadIdx.y + filter_r + filterWidth / 2) * (blockDim.x + 2 * (filterWidth / 2)) + threadIdx.x + filter_c + filterWidth / 2]);
      float filter_value = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];
      result_1 += image_value_1 * filter_value;
      result_2 += image_value_2 * filter_value;
    }
  }
  result_1 = result_1 / (filterWidth * filterWidth);
  result_2 = result_2 / (filterWidth * filterWidth);
  //__syncthreads();
  if (y < numRows &&  x < numCols)
  {
    GPU_gw_average_color_1[thread_1D] = result_1;
    GPU_gw_average_color_2[thread_1D] = result_2;
  }
  __syncthreads();
  //printf("GPU_gw_average_color_1_2[thread_1D= %d]= %f , %f \n",thread_1D,GPU_gw_average_color_1[thread_1D],GPU_gw_average_color_2[thread_1D]);

// Computing the weighted auto correlation (alpha)
  result_1 = 0.0f;
  result_2 = 0.0f;
  //#pragma unroll 16

  if (y < numRows && x < numCols)
  {

    for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; filter_r++)
    {
      //printf("hello babak %d \n",filter_r+filterWidth/2);
      for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; filter_c++)
      {

        float image_value_1 = static_cast<float>(temp_1[(threadIdx.y + filter_r + filterWidth / 2) * (blockDim.x + 2 * (filterWidth / 2)) + threadIdx.x + filter_c + filterWidth / 2]);
        float image_value_2 = static_cast<float>(temp_2[(threadIdx.y + filter_r + filterWidth / 2) * (blockDim.x + 2 * (filterWidth / 2)) + threadIdx.x + filter_c + filterWidth / 2]);
        //float diff_Pow_2_1=(image_value_1-result_1)*(image_value_1-result_1);
        float diff_Pow_2_1 = (image_value_1 - GPU_gw_average_color_1[thread_1D]) * (image_value_1 - GPU_gw_average_color_1[thread_1D]);
        //float diff_Pow_2_2=(image_value_2-result_2)*(image_value_2-result_2);
        float diff_Pow_2_2 = (image_value_2 - GPU_gw_average_color_2[thread_1D]) * (image_value_2 - GPU_gw_average_color_2[thread_1D]);
        float filter_value_1 = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];
        result_1 += filter_value_1 * diff_Pow_2_1;
        result_2 += filter_value_1 * diff_Pow_2_2;
      }
    }
    result_1 = result_1 / (filterWidth * filterWidth);
    result_2 = result_2 / (filterWidth * filterWidth);
    __syncthreads();


    GPU_gw_auto_correlation_1[thread_1D] = result_1;
    GPU_gw_auto_correlation_2[thread_1D] = result_2;
    //if (x==200){
    //printf("GPU_gw_auto_correlation_2[%d][%d]=%f\n",y,x,GPU_gw_auto_correlation_2[thread_1D]);
    //}
    //outputChannel[thread_1D]=GPU_gw_auto_correlation_1[thread_1D];
  }
  //outputChannel[thread_1D]=GPU_gw_auto_correlation_1[thread_1D];
  __syncthreads();
  //printf("GPU_gw_auto_correlation_1_2[thread_1D= %d]= %f , %f \n",thread_1D,GPU_gw_auto_correlation_1[thread_1D],GPU_gw_auto_correlation_2[thread_1D]);


  /*
  if ( x >= numCols || y >= numRows ){
      return;
  }
  else
  {
      outputChannel[thread_1D]=result_1;

  }*/
  //__syncthreads();
  // NOTE: If a thread's absolute position 2D position is within the image, but some of
  // its neighbors are outside the image, then you will need to be extra careful. Instead
  // of trying to read such a neighbor value from GPU memory (which won't work because
  // the value is out of bounds), you should explicitly clamp the neighbor values you read
  // to be within the bounds of the image. If this is not clear to you, then please refer
  // to sequential reference solution for the exact clamping semantics you should follow.
}
//-----------------------------------
__global__ void Kernel_2(const unsigned char* const inputChannel_1, //inputChannel
                         unsigned char* const inputChannel_2,
                         unsigned char* const outputChannel,
                         float* const GPU_gw_average_color_1,
                         float* const GPU_gw_average_color_2,
                         float* const GPU_gw_auto_correlation_1,
                         float* const GPU_gw_auto_correlation_2,
                         float* const GPU_gw_cross_correlation_3,
                         float* const GPU_gw_normalized_score_4,
                         int* match_matrix,
                         int* disparity_map,
                         int* depth_map,
                         int numRows, int numCols,
                         const float* const filter, const int filterWidth, const int mid_shared_address)
{
  // TODO

  // NOTE: Be sure to compute any intermediate results in floating point
  // before storing the final result as unsigned char.

  // NOTE: Be careful not to try to access memory that is outside the bounds of
  // the image. You'll want code that performs the following check before accessing
  // GPU memory:
  //
  // if ( absolute_image_position_x >= numCols || absolute_image_position_y >= numRows )
  // {
  //     return;
  // }
  int x = threadIdx.x + blockIdx.x * blockDim.x; // x index of a tread in an image (not block)
  int y = threadIdx.y + blockIdx.y * blockDim.y; // y index of a tread in an image (not block)
  int thread_1D = x + y * numCols;
  //printf("the x= %d y= %d  and  threadID %d inputvalue %d \n",x,y,thread_1D,inputChannel[thread_1D] );

  extern __shared__ unsigned char temp_1[];

  unsigned char* pointer = (unsigned char*)temp_1;
  unsigned char* temp_2 = (unsigned char*)&pointer[mid_shared_address];

  //float result_1=0.0f;
  //float result_2=0.0f;
  const int max_disparity = 200;
  //int disparity; //this value used for save the number of pixels which should be compared for each pixel
  //disparity=(x>=max_disparity)?max_disparity:x;
  //disparity=((blockIdx.x*blockDim.x)>=max_disparity)?max_disparity:x;//(blockIdx.x*blockDim.x);
  if (x < numCols  && y < numRows)
  {
    GPU_gw_cross_correlation_3[thread_1D] = -100000.0; //__FLT_MIN__;
    GPU_gw_normalized_score_4[thread_1D] = -100000.0; //__FLT_MIN__;
  }
  //first batch transfer
  //temp_1[(threadIdx.x+filterWidth/2)+(threadIdx.y+filterWidth/2)*(blockDim.x+2*(filterWidth/2))]=inputChannel[thread_1D];
  int dest = threadIdx.y * blockDim.x + threadIdx.x, // index(in a one dimentional array) of a thread(pixel) in a block
      destY = dest / (blockDim.x + 2 * (filterWidth / 2)), // y index of destination in shared memory
      destX = dest % (blockDim.x + 2 * (filterWidth / 2)), // x index of destination in shared memory
      srcY = blockIdx.y * blockDim.y + destY - (filterWidth / 2), // y index in source image (-filterwidth/2 because of overlaping)
      srcX = blockIdx.x * blockDim.x + destX - (filterWidth / 2), //// x index in source image (-filterwidth/2 because of overlaping)
      src = srcY * numCols + srcX;// index(in a one dimentional array) of a thread(pixel) in the source image
  int second_srcX = srcX - max_disparity;
  int second_src = src - max_disparity; // for transfering the second window of cross correlation to shared memory

  if (srcY >= 0 && srcY < numRows && srcX >= 0 && srcX < numCols)
  {
    temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[src];
  }
  else
  {
    //temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX] =0; //;
    //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
    if (srcY < 0)
    {
      srcY = 0;
    }
    if (srcY >= numRows)
    {
      srcY = numRows - 1 ;
    }
    if (srcX < 0)
    {
      srcX = 0;
    }
    if (srcX >= numCols)
    {
      srcX = numCols - 1 ;
    }
    int newindex = srcY * numCols + srcX;
    temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[newindex];
  }
  // first batch loading for second window in shared memory
  //if (blockIdx.x==14 && threadIdx.y==3 ){
  //printf("second_src=%d  ",second_src);
  //}

  if (srcY >= 0 && srcY < numRows && second_srcX >= 0 && second_srcX < numCols && second_src >= 0 && second_src < numCols * numRows)
  {
    temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[second_src];
    //printf("temp2[%d]=%d\t",destY*(blockDim.x+2*(filterWidth/2))+destX,temp_2[destY*(blockDim.x+2*(filterWidth/2))+destX]);
  }

  else
  {
    //temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX] =0; //;
    //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
    if (srcY < 0)
    {
      srcY = 0;
    }
    if (srcY >= numRows)
    {
      srcY = numRows - 1 ;
    }
    if (second_srcX < 0)
    {
      second_srcX = 0;
    }
    if (second_srcX >= numCols)
    {
      second_srcX = numCols - 1 ;
    }
    int newindex = srcY * numCols + second_srcX;
    temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[newindex];
    //printf("temp2[%d]=%d\t",destY*(blockDim.x+2*(filterWidth/2))+destX,temp_2[destY*(blockDim.x+2*(filterWidth/2))+destX]);
  }
//Second batch loading
  dest = threadIdx.y * blockDim.x + threadIdx.x + blockDim.x * blockDim.y;
  destY = dest / (blockDim.x + 2 * (filterWidth / 2));
  destX = dest % (blockDim.x + 2 * (filterWidth / 2));
  srcY = blockIdx.y * blockDim.y + destY - (filterWidth / 2);
  srcX = blockIdx.x * blockDim.x + destX - (filterWidth / 2);
  src =  srcY * numCols + srcX;
  second_srcX = srcX - max_disparity;
  second_src = src - max_disparity; // for transfering second batch of the second window of cross correlation to shared memory
  //printf("threadIdx.x= %d threadIdx.y= %d blockIdx.x= %d blockIdx.y= %d \n",threadIdx.x,threadIdx.y,blockIdx.x,blockIdx.y);
  //printf("dest= %d destY= %d destX= %d srcY= %d srcX= %d src %d \n",dest,destY,destX,srcY,scX,src );
  if (destY < (blockDim.y + 2 * (filterWidth / 2)))
  {
    //int help_2 = destY*(blockDim.x+2*(filterWidth/2))+destX;
    if (srcY >= 0 && srcY < numRows && srcX >= 0 && srcX < numCols)
    {
      temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[src];
    }
    else
    {
      if (srcY < 0)
      {
        srcY = 0;
      }
      if (srcY >= numRows)
      {
        srcY = numRows - 1 ;
      }
      if (srcX < 0)
      {
        srcX = 0;
      }
      if (srcX >= numCols)
      {
        srcX = numCols - 1 ;
      }
      int newindex2 = srcY * numCols + srcX;
      temp_1[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_1[newindex2];
    }
    // second batch loading for second window in shared memory

    if (srcY >= 0 && srcY < numRows && second_srcX >= 0 && second_srcX < numCols  && second_src >= 0 && second_src < numCols * numRows)
    {
      temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[second_src];
      //printf("temp2[%d]=%d\t",destY*(blockDim.x+2*(filterWidth/2))+destX,temp_2[destY*(blockDim.x+2*(filterWidth/2))+destX]);
    }
    else
    {
      //temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX] =0; //;
      //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
      if (srcY < 0)
      {
        srcY = 0;
      }
      if (srcY >= numRows)
      {
        srcY = numRows - 1 ;
      }
      if (second_srcX < 0)
      {
        second_srcX = 0;
      }
      if (second_srcX >= numCols)
      {
        second_srcX = numCols - 1 ;
      }
      int newindex2 = srcY * numCols + second_srcX;
      temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[newindex2];
      //printf("temp2[%d]=%d\t",destY*(blockDim.x+2*(filterWidth/2))+destX,temp_2[destY*(blockDim.x+2*(filterWidth/2))+destX]);
    }
  }
  __syncthreads();


  //until here there was just transfering the first block from main memory to shared memoryhelecoptrei zadi,
  //after here it is computing the convultion filter and updating the shared memory

  for (int j = 0; j < max_disparity; j++)
  {

    if (x - max_disparity + j >= 0)
    {
      float result_1 = 0.0f;
      if (y >= 0 && y < numRows && x >= 0 && x < numCols && (thread_1D - max_disparity + j) >= 0)
      {
        for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; filter_r++)
        {
          //printf("hello babak %d \n",filter_r+filterWidth/2);
          for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; filter_c++)
          {
            float image_value_1 = static_cast<float>(temp_1[(threadIdx.y + filter_r + filterWidth / 2) * (blockDim.x + 2 * (filterWidth / 2)) + threadIdx.x + filter_c + filterWidth / 2]);
            float image_value_2 = static_cast<float>(temp_2[(threadIdx.y + filter_r + filterWidth / 2) * (blockDim.x + 2 * (filterWidth / 2)) + threadIdx.x + filter_c + filterWidth / 2]);
            //if (x==100 && y==100 && j==50){
            //printf(" hello babak2 x=%d y=%d j=%d  image_value_1=%f image_value_2=%f  \n", x,y,j,image_value_1,image_value_2 );
            //}
            float filter_value = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];
            float image_minus_average_1 = static_cast<float>(image_value_1) - static_cast<float>(GPU_gw_average_color_1[thread_1D]);
            float image_minus_average_2 = static_cast<float>(image_value_2) - static_cast<float>(GPU_gw_average_color_2[thread_1D - max_disparity + j]);
            result_1 += image_minus_average_1 * image_minus_average_2 * filter_value;
          }
        }
        //__syncthreads();
        result_1 = result_1 / (filterWidth * filterWidth);
        //printf("result_1,GPU_gw_cross_correlation_3[thread_1D=%d]=%f   %f\n",thread_1D,result_1,GPU_gw_cross_correlation_3[thread_1D]);
        float help_3 = (static_cast<float>(result_1) / (static_cast<float>(sqrt(GPU_gw_auto_correlation_1[thread_1D] * GPU_gw_auto_correlation_2[thread_1D - max_disparity + j]))));
        //__syncthreads();
        if (help_3 > GPU_gw_normalized_score_4[thread_1D] && y < numRows && x < numCols) // && (x-max_disparity+j)>=0 ){
        {
          GPU_gw_cross_correlation_3[thread_1D] = result_1;
          GPU_gw_normalized_score_4[thread_1D] = help_3;
          match_matrix[thread_1D] = x - max_disparity + j;
          //disparity_map[thread_1D]=abs(match_matrix[thread_1D]-x);
          //match_matrix[thread_1D]=x-(x-max_disparity+j);
          //if (x==100 && y==100){
          //printf("x=%d y=%d j=%d thread_1D=%d help_3=%f  match_matrix[thread_1D]=%d  \n", x,y,j,thread_1D,help_3,match_matrix[thread_1D]);
          //}
        }
      }
    }
    // moving second matching window
    //first batch transfer
    //temp_1[(threadIdx.x+filterWidth/2)+(threadIdx.y+filterWidth/2)*(blockDim.x+2*(filterWidth/2))]=inputChannel[thread_1D];
    unsigned char help_1, help_2;
    int dest = threadIdx.y * blockDim.x + threadIdx.x, // index(in a one dimentional array) of a thread(pixel) in a block
        destY = dest / (blockDim.x + 2 * (filterWidth / 2)), // y index of destination in shared memory
        destX = dest % (blockDim.x + 2 * (filterWidth / 2)), // x index of destination in shared memory
        srcY = blockIdx.y * blockDim.y + destY - (filterWidth / 2), // y index in source image (-filterwidth/2 because of overlaping)
        srcX = blockIdx.x * blockDim.x + destX - (filterWidth / 2), //// x index in source image (-filterwidth/2 because of overlaping)
        src = srcY * numCols + srcX;// index(in a one dimentional array) of a thread(pixel) in the source image
    int second_srcX = srcX - max_disparity + (j + 1);
    int second_src = src - max_disparity + (j + 1); // for transfering the second window of cross correlation to shared memory
    //printf("threadIdx.x= %d threadIdx.y= %d blockIdx.x= %d blockIdx.y= %d \n",threadIdx.x,threadIdx.y,blockIdx.x,blockIdx.y);
    //printf("dest= %d destY= %d destX= %d srcY= %d srcX= %d src %d \n",dest,destY,destX,srcY,srcX,src );

    // first batch moving for second window in shared memory

    if (destX >= 1 && destX < (blockDim.x + 2 * (filterWidth / 2)) && destY < (blockDim.y + 2 * (filterWidth / 2)))
    {
      //if(blockIdx.x==4 && blockIdx.y==0 && j==190 ){
      //printf(" hello booob2 dest=%d x=%d y=%d threadIdx.x=%d threadIdx.y=%d j=%d destX=%d destY=%d \n", dest,x,y,threadIdx.x,threadIdx.y,j,destX,destY);
      //}
      help_1 = temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX];
    }
    syncthreads();
    if (destX >= 1 && destX < (blockDim.x + 2 * (filterWidth / 2)) && destY < (blockDim.y + 2 * (filterWidth / 2)))
    {
      temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX - 1] = help_1;
    }
    //__syncthreads();
    if ((destX == ((blockDim.x + 2 * (filterWidth / 2)) - 1)) && destY < (blockDim.y + 2 * (filterWidth / 2))  && second_srcX >= 0)
    {

      //if (blockIdx.x==5 && blockIdx.y==5){
      //    printf(" hello booob x=%d y=%d j=%d destX=%d destY=%d \n", x,y,j,destX,destY);
      //}
      if (srcY >= 0 && srcY < numRows && second_srcX >= 0 && (second_srcX + 1) < numCols  && second_src >= 0 && second_src < numCols * numRows)
      {
        temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[second_src];
      }
      else
      {
        //temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX] =0; //;
        //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
        if (srcY < 0)
        {
          srcY = 0;
        }
        if (srcY >= numRows)
        {
          srcY = numRows - 1 ;
        }
        if (second_srcX < 0)
        {
          second_srcX = 0;
        }
        if ((second_srcX) >= numCols)
        {
          second_srcX = numCols - 1 ;
        }
        int newindex = srcY * numCols + second_srcX;
        temp_2[destY * (blockDim.x + 2 * (filterWidth / 2)) + destX] = inputChannel_2[newindex];
      }
    }
    __syncthreads();

    //Second batch moving
    int dest_2 = threadIdx.y * blockDim.x + threadIdx.x + blockDim.x * blockDim.y;
    int destY_2 = dest_2 / (blockDim.x + 2 * (filterWidth / 2));
    int destX_2 = dest_2 % (blockDim.x + 2 * (filterWidth / 2));
    int srcY_2 = blockIdx.y * blockDim.y + destY_2 - (filterWidth / 2);
    int srcX_2 = blockIdx.x * blockDim.x + destX_2 - (filterWidth / 2);
    int src_2 =  srcY_2 * numCols + srcX_2;
    int second_srcX_2 = srcX_2 - max_disparity + (j + 1);;
    int second_src_2 = src_2 - max_disparity + (j + 1); // for transfering second batch of the second window of cross correlation to shared memory
    //if(blockIdx.x==0 && blockIdx.y==0   && destY_2 < (blockDim.y+2*(filterWidth/2))  ){
    //    printf(" hello booob3 dest_2=%d x=%d y=%d threadIdx.x=%d threadIdx.y=%d j=%d destX_2=%d destY_2=%d \n", dest_2,x,y,threadIdx.x,threadIdx.y,j,destX_2,destY_2);
    //}
    //if(destY_2 < (blockDim.y+2*(filterWidth/2))){

    ///*

    if (destX_2 >= 1 && destX_2 < (blockDim.x + 2 * (filterWidth / 2)) && destY_2 < (blockDim.y + 2 * (filterWidth / 2)))
    {
      help_2 = temp_2[destY_2 * (blockDim.x + 2 * (filterWidth / 2)) + destX_2];
    }

    syncthreads();
    if (destX_2 >= 1 && destX_2 < (blockDim.x + 2 * (filterWidth / 2)) && destY_2 < (blockDim.y + 2 * (filterWidth / 2)))
    {
      temp_2[destY_2 * (blockDim.x + 2 * (filterWidth / 2)) + destX_2 - 1] = help_2;
    }
    //__syncthreads();
    //printf("destX=%d ((blockDim.x+2*(filterWidth/2))-1)=%d destY=%d (blockDim.y+2*(filterWidth/2))=%d \n",destX ,((blockDim.x+2*(filterWidth/2))-1),destY,(blockDim.y+2*(filterWidth/2)));

    //if ((destX== ((blockDim.x+2*(filterWidth/2))-1))){
    //  printf("destX=%d destY=%d \n", destX,destY);
    //}
    if ((destX_2 == ((blockDim.x + 2 * (filterWidth / 2)) - 1)) && destY_2 < (blockDim.y + 2 * (filterWidth / 2)) && second_srcX_2 >= 0)
    {
      //if (x==100 && y==100){
      //  printf(" hello booob x=%d y=%d j=%d \n", x,y,j);
      //}
      //printf("destX_2=%d destY_2=%d \n", destX_2,destY_2);
      if (srcY_2 >= 0 && srcY_2 < numRows && second_srcX_2 >= 0 && (second_srcX_2 + 1) < numCols  && second_src_2 >= 0 && second_src_2 < numCols * numRows)
      {
        temp_2[destY_2 * (blockDim.x + 2 * (filterWidth / 2)) + destX_2] = inputChannel_2[second_src_2];
      }
      else
      {
        //temp_1[destY*(blockDim.x+2*(filterWidth/2))+destX] =0; //;
        //printf("temp_1[help_1=%d]=%d \n",help_1,temp_1[help_1]);
        if (srcY_2 < 0)
        {
          srcY_2 = 0;
        }
        if (srcY_2 >= numRows)
        {
          srcY_2 = numRows - 1 ;
        }
        if (second_srcX_2 < 0)
        {
          second_srcX_2 = 0;
        }
        if ((second_srcX_2) >= numCols)
        {
          second_srcX_2 = numCols - 1 ;
        }
        int newindex2_2 = srcY_2 * numCols + second_srcX_2;
        temp_2[destY_2 * (blockDim.x + 2 * (filterWidth / 2)) + destX_2] = inputChannel_2[newindex2_2];
      }
    }//*/
    //__syncthreads();
/////////////////////////////////////////////////////////////////
  }

  __syncthreads();
  if (y < numRows && y > 0 && x > 0 && x < numCols)
  {
    disparity_map[thread_1D] = abs(match_matrix[thread_1D] - x);
    //printf("disparity_map[%d]= %d\n",thread_1D,disparity_map[thread_1D]);
    //__syncthreads();
    //outputChannel[thread_1D] =static_cast<unsigned char>(GPU_gw_average_color_2[thread_1D]);
    outputChannel[thread_1D] = static_cast<unsigned char>(disparity_map[thread_1D]);
  }
  //outputChannel[thread_1D]=GPU_gw_auto_correlation_1[thread_1D];
  __syncthreads();

  //---------------------------------------------------------------------------------------------------------

  /*
  if ( x >= numCols || y >= numRows ){
      return;
  }
  else
  {
      outputChannel[thread_1D]=result_1;

  }*/

  //__syncthreads();
  // NOTE: If a thread's absolute position 2D position is within the image, but some of
  // its neighbors are outside the image, then you will need to be extra careful. Instead
  // of trying to read such a neighbor value from GPU memory (which won't work because
  // the value is out of bounds), you should explicitly clamp the neighbor values you read
  // to be within the bounds of the image. If this is not clear to you, then please refer
  // to sequential reference solution for the exact clamping semantics you should follow.
}



float         *d_filter;

void gpu_part::allocateMemoryAndCopyToGPU(const int numRows, const int numCols,
    const float* const h_filter, const int filterWidth)
{

  //allocate memory for the three different channels
  //original


  //checkCudaErrors(cudaMalloc(&d_Gray_1,   sizeof(unsigned char) * numRows * numCols));
  //checkCudaErrors(cudaMalloc(&d_Gray_2,   sizeof(unsigned char) * numRows * numCols));

  //TODO:
  //Allocate memory for the filter on the GPU
  //Use the pointer d_filter that we have already declared for you
  //You need to allocate memory for the filter with cudaMalloc
  //be sure to use checkCudaErrors like the above examples to
  //be able to tell if anything goes wrong
  //IMPORTANT: Notice that we pass a pointer to a pointer to cudaMalloc
  checkCudaErrors(cudaMalloc(&d_filter, sizeof(float)*filterWidth * filterWidth));


  //TODO:
  //Copy the filter on the host (h_filter) to the memory you just allocated
  //on the GPU.  cudaMemcpy(dst, src, numBytes, cudaMemcpyHostToDevice);
  //Remember to use checkCudaErrors!
  cudaMemcpy(d_filter, h_filter, sizeof(float)*filterWidth * filterWidth, cudaMemcpyHostToDevice);

}



void gpu_part::your_gaussian_blur(const unsigned char* const h_inputImageGray_1,
                                  unsigned char* const d_inputImageGray_1,
                                  unsigned char* const h_inputImageGray_2,
                                  unsigned char* const d_inputImageGray_2,
                                  unsigned char* const d_outputImageGray,
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
                                  const int filterWidth)
{
  //TODO: Set reasonable block size (i.e., number of threads per block)
  const dim3 blockSize(32, 32);
  //const dim3 blockSize(8,8);
  //TODO:
  //Compute correct grid size (i.e., number of blocks per kernel launch)
  //from the image size and and block size.
  //const dim3 gridSize(15,12);
  //const dim3 gridSize(5,1);
  //const dim3 gridSize(1,1);
  const dim3 gridSize(ceilf(static_cast<float>(numCols) / blockSize.x), ceilf(static_cast<float>(numRows) / blockSize.y));

  //TODO: Launch a kernel for separating the RGBA image into different color channels
  //separateChannels<<<gridSize,blockSize>>>(d_inputImageRGBA,(int)numRows,(int)numCols,d_red, d_green,d_blue);

  // Call cudaDeviceSynchronize(), then call checkCudaErrors() immediately after
  // launching your kernel to make sure that you didn't make any mistakes.
  //cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());

  //TODO: Call your convolution kernel here 3 times, once for each color channel.
  int mid_shared_address = (blockSize.x + (2 * (filterWidth / 2))) * (blockSize.y + (2 * (filterWidth / 2))) * sizeof(unsigned char);
  Kernel_1 <<< gridSize, blockSize, (blockSize.x + (2 * (filterWidth / 2)))*(blockSize.y + (2 * (filterWidth / 2)))*sizeof(unsigned char) * 2 >>> (d_inputImageGray_1,
      d_inputImageGray_2,
      d_outputImageGray,
      GPU_gw_average_color_1,
      GPU_gw_average_color_2,
      GPU_gw_auto_correlation_1,
      GPU_gw_auto_correlation_2,
      (int)numRows,
      (int)numCols,
      d_filter,
      filterWidth, mid_shared_address);
  // Again, call cudaDeviceSynchronize(), then call checkCudaErrors() immediately after
  // launching your kernel to make sure that you didn't make any mistakes.
  cudaDeviceSynchronize();
  checkCudaErrors(cudaGetLastError());
  //for (int r = 0; r < (int)numRows; ++r) {
  //    for (int c = 0; c < (int)numCols; ++c) {
  //     printf("%f ",GPU_gw_auto_correlation_1[r * numCols + c]);
  //    }
  //    printf("\n");
  //}


  Kernel_2 <<< gridSize, blockSize, (blockSize.x + (2 * (filterWidth / 2)))*(blockSize.y + (2 * (filterWidth / 2)))*sizeof(char) * 2 >>> (d_inputImageGray_1,
      d_inputImageGray_2,
      d_outputImageGray,
      GPU_gw_average_color_1,
      GPU_gw_average_color_2,
      GPU_gw_auto_correlation_1,
      GPU_gw_auto_correlation_2,
      GPU_gw_cross_correlation_3,
      GPU_gw_normalized_score_4,
      match_matrix,
      disparity_map,
      depth_map,
      (int)numRows,
      (int)numCols,
      d_filter,
      filterWidth, mid_shared_address);
  cudaDeviceSynchronize();
  checkCudaErrors(cudaGetLastError());

  /*


   cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());

   // Now we recombine your results. We take care of launching this kernel for you.
   //
   // NOTE: This kernel launch depends on the gridSize and blockSize variables,
   // which you must set yourself.
   recombineChannels<<<gridSize, blockSize>>>(d_redBlurred,
                                              d_greenBlurred,
                                              d_blueBlurred,
                                              d_outputImageRGBA,
                                              numRows,
                                              numCols);
   cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());


   recombineChannels<<<gridSize, blockSize>>>(d_redBlurred_1,
                                              d_greenBlurred_1,
                                              d_blueBlurred_1,
                                              d_outputImageRGBA_1,
                                              numRows,
                                              numCols);
   cudaDeviceSynchronize(); checkCudaErrors(cudaGetLastError());
  */

}


}
}
}
}
}
