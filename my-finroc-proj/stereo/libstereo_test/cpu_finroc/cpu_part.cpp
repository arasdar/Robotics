/*
 * gpu_part.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/libstereo_test/cpu_finroc/cpu_part.h"

/************reference calculation2***************/
void CPU_referenceCalculation(const uchar* const grayImage_1, const uchar* const grayImage_2, uchar *const outputImage_1,
                              uchar *const outputImage_2, int numRows, int numCols,
                              const float* const filter, const int filterWidth)
{
  //unsigned char *gray_1  = new unsigned char[numRows * numCols];
  //gw_average_color ====>  gray_weighted_average_color
  float *gw_average_color_1   = new float[numRows * numCols];
  //gw_auto_correlation ====>  gray_weighted_auto_correlation
  float *gw_auto_correlation_1   = new float[numRows * numCols];
  //unsigned char *temp_1   = new unsigned char[numRows * numCols];
  //unsigned char *gray_2  = new unsigned char[numRows * numCols];
  float *gw_average_color_2   = new float[numRows * numCols];
  float *gw_auto_correlation_2   = new float[numRows * numCols];
  //unsigned char *temp_2   = new unsigned char[numRows * numCols];
  //gw_cross_correlation_3 ===>gray_weighted_cross_correlation_3
  float **gw_cross_correlation_3   = new float*[numRows * numCols];
  //gw_normalized_score_4 ===>gray_weighted_normalized_score_4
  float **gw_normalized_score_4   = new float*[numRows * numCols];
  //unsigned char **temp_3   = new unsigned char*[numRows * numCols];

  //unsigned char *temp_4   = new unsigned char[numRows * numCols];
  //unsigned char *temp_5   = new unsigned char[numRows * numCols];

  int *match_matrix = new int[numRows * numCols];
  int *disparity_map = new int [numRows * numCols];
  //unsigned char *depth_map=new unsigned char [numRows*numCols];
  int *depth_map = new int [numRows * numCols];

  /*
  for (int r = 0; r < (int)numRows; ++r) {
    for (int c = 0; c < (int)numCols; ++c) {
      float image_value = static_cast<float>(channel[r * numCols + c]);
    }
  }*/
  const int max_disparity = 80; //(int)numCols;
  int index;
  //for (int i = 0; i < numRows * numCols; ++i){
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int) numCols; ++c)
    {
      index = r * numCols + c;
      if (c >= max_disparity)
      {
        gw_cross_correlation_3[index] = new float[max_disparity];
        gw_normalized_score_4[index] = new float[max_disparity];
        //temp_3[i]=new unsigned char[max_disparity];
      }
      else
      {
        gw_cross_correlation_3[index] = new float[c + 1];
        gw_normalized_score_4[index] = new float[c + 1];
      }
    }
  }

  //Now we can do the convolution for each of the color channels
  //channelConvolution(grayImage_1, gw_average_color_1, numRows, numCols, filter, filterWidth);
  //channelConvolution(grayImage_2, gw_average_color_2, numRows, numCols, filter, filterWidth);

  //Dealing with an even width filter is trickier
  assert(filterWidth % 2 == 1);

  //For every pixel in the image
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      float result_1 = 0.f;
      float result_2 = 0.f;
      //For every value in the filter around the pixel (c, r)
      for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; ++filter_r)
      {
        for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; ++filter_c)
        {
          //Find the global image position for this filter position
          //clamp to boundary of the image
          int image_r = std::min(std::max(r + filter_r, 0), static_cast<int>(numRows - 1));
          int image_c = std::min(std::max(c + filter_c, 0), static_cast<int>(numCols - 1));

          float image_value_1 = static_cast<float>(grayImage_1[image_r * numCols + image_c]);
          float image_value_2 = static_cast<float>(grayImage_2[image_r * numCols + image_c]);
          float filter_value = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];

          result_1 += image_value_1 * filter_value;
          result_2 += image_value_2 * filter_value;
        }
      }

      gw_average_color_1[r * numCols + c] = result_1 / (filterWidth * filterWidth);
      gw_average_color_2[r * numCols + c] = result_2 / (filterWidth * filterWidth);
    }
  }

  //now recombine into the output image - Alpha is 255 for no transparency

  //for (int i = 0; i < numRows * numCols; ++i) {
  //temp_1[i] = pow (static_cast<float> (grayImage_1[i])-static_cast<float> (gw_average_color_1[i]),2.0);
  //temp_2[i] = pow (static_cast<float> (grayImage_2[i])-static_cast<float> (gw_average_color_2[i]),2.0);
  //}
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      float result_1 = 0.f;
      float result_2 = 0.f;
      //For every value in the filter around the pixel (c, r)
      for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; ++filter_r)
      {
        for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; ++filter_c)
        {
          //Find the global image position for this filter position
          //clamp to boundary of the image
          int image_r = std::min(std::max(r + filter_r, 0), static_cast<int>(numRows - 1));
          int image_c = std::min(std::max(c + filter_c, 0), static_cast<int>(numCols - 1));

          float image_value_1 = static_cast<float>(grayImage_1[image_r * numCols + image_c]) - static_cast<float>(gw_average_color_1[r * numCols + c]);
          float image_value_2 = static_cast<float>(grayImage_2[image_r * numCols + image_c]) - static_cast<float>(gw_average_color_2[r * numCols + c]);
          float filter_value = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];

          result_1 += image_value_1 * image_value_1 * filter_value;
          result_2 += image_value_2 * image_value_2 * filter_value;
        }
      }

      gw_auto_correlation_1[r * numCols + c] = result_1 / (filterWidth * filterWidth);
      gw_auto_correlation_2[r * numCols + c] = result_2 / (filterWidth * filterWidth);
    }
  }

  /*for (int r = 0; r < (int)numRows; ++r) {
      for (int c = 0; c < (int)numCols; ++c) {
  printf("%d ",gw_auto_correlation_1[r * numCols + c]);
      }
      printf("\n");
  }*/

  //channelConvolution(temp_1,gw_auto_correlation_1, numRows, numCols, filter, filterWidth);
  //channelConvolution(temp_2, gw_auto_correlation_2, numRows, numCols, filter, filterWidth);


  int disparity; //this value used for save the number of pixels which should be compared for each pixel
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      if ((r * numCols + c) % 10000 == 0)
        printf("%d\n", r);

      // if fixed pixel'S column index is more than max_disparity then just compare the fixed pixel convolution
      // with max_disparity numebr of pixel from left side of convolution window in right side image
      //otherwise comare it with c+1 pixel from right side image


      disparity = (c >= max_disparity) ? max_disparity : c;

      if (disparity == max_disparity)
      {
        for (int j = 0; j < max_disparity; j++)
        {
          //temp_3[r * numCols + c][j]=(static_cast<float> (grayImage_1[r * numCols + c])-static_cast<float> (gw_average_color_1[r * numCols + c]))*(static_cast<float> (grayImage_2[r * numCols + j])-static_cast<float> (gw_average_color_2[r * numCols + j]));
          float result = 0.f;
          //For every value in the filter around the pixel (c, r)
          for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; ++filter_r)
          {
            for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; ++filter_c)
            {
              //Find the global image position for this filter position
              //clamp to boundary of the image
              int image_r = std::min(std::max(r + filter_r, 0), static_cast<int>(numRows - 1));
              int image_c = std::min(std::max(c + filter_c, 0), static_cast<int>(numCols - 1));
              int image_j = std::min(std::max((c - max_disparity + j) + filter_c, 0), static_cast<int>(numCols - 1));

              float image_value_1 = static_cast<float>(grayImage_1[image_r * numCols + image_c]) - static_cast<float>(gw_average_color_1[r * numCols + c]);
              float image_value_2 = static_cast<float>(grayImage_2[image_r * numCols + image_j]) - static_cast<float>(gw_average_color_2[r * numCols + (c - max_disparity + j)]);
              float filter_value = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];

              result += image_value_1 * image_value_2 * filter_value;

            }
          }
          //if (r==200 && c== 200 )
          //printf("%f ",result);
          gw_cross_correlation_3[r * numCols + c][j] = result / (filterWidth * filterWidth);

        }

      }
      else
      {
        for (int j = 0; j <= c; j++)
        {
          //temp_3[r * numCols + c][j]=(static_cast<float> (grayImage_1[r * numCols + c])-static_cast<float> (gw_average_color_1[r * numCols + c]))*(static_cast<float> (grayImage_2[r * numCols + j])-static_cast<float> (gw_average_color_2[r * numCols + j]));
          float result = 0.f;
          //For every value in the filter around the pixel (c, r)
          for (int filter_r = -filterWidth / 2; filter_r <= filterWidth / 2; ++filter_r)
          {
            for (int filter_c = -filterWidth / 2; filter_c <= filterWidth / 2; ++filter_c)
            {
              //Find the global image position for this filter position
              //clamp to boundary of the image
              int image_r = std::min(std::max(r + filter_r, 0), static_cast<int>(numRows - 1));
              int image_c = std::min(std::max(c + filter_c, 0), static_cast<int>(numCols - 1));
              int image_j = std::min(std::max(j + filter_c, 0), static_cast<int>(numCols - 1));

              float image_value_1 = static_cast<float>(grayImage_1[image_r * numCols + image_c]) - static_cast<float>(gw_average_color_1[r * numCols + c]);
              float image_value_2 = static_cast<float>(grayImage_2[image_r * numCols + image_j]) - static_cast<float>(gw_average_color_2[r * numCols + j]);
              float filter_value = filter[(filter_r + filterWidth / 2) * filterWidth + filter_c + filterWidth / 2];

              result += image_value_1 * image_value_2 * filter_value;

            }
          }
          //if (r==200 && c== 200 )
          //printf("%f ",result);
          gw_cross_correlation_3[r * numCols + c][j] = result / (filterWidth * filterWidth);

        }
      }

    }
  }/*
  for (int r = 200; r <220 ; ++r) { //(int)numRows
    for (int c = 200; c <220 ; ++c) { //(int)numCols
      printf("gw_auto_correlation_2[%d]=%f\n",r * numCols + c, gw_auto_correlation_2[r * numCols + c]);
    }
  }
  for (int j=0; j<max_disparity;j++){
    printf("gw_cross_correlation_3[239][%d]=%f\n",j, gw_cross_correlation_3[239][j]);
  }
  */


  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      disparity = (c >= max_disparity) ? max_disparity : c;

      if (disparity == max_disparity)
      {
        for (int j = 0; j < max_disparity; j++)
        {
          /*if (r*numCols+c==200)
          {
            printf("gw_cross_correlation_3[%d][%d]=%f\n",r * numCols + c,j,gw_cross_correlation_3[r * numCols + c][j]);
            printf("gw_auto_correlation_1[%d]=%d\n",r * numCols + c,gw_auto_correlation_1[r * numCols + c]);
            printf("gw_auto_correlation_2[%d]=%d\n",r * numCols + j,gw_auto_correlation_2[r *numCols + j]);
          }*/
          gw_normalized_score_4[r * numCols + c][j] = static_cast<float>(gw_cross_correlation_3[r * numCols + c][j]) / static_cast<float>(sqrt(static_cast<float>(gw_auto_correlation_1[r * numCols + c]) * static_cast<float>(gw_auto_correlation_2[r * numCols + c - max_disparity + j])));
        }
      }
      else
      {
        for (int j = 0; j <= c; j++)
        {
          /*if (r*numCols+c==200)
          {
            printf("gw_cross_correlation_3[%d][%d]=%f\n",r * numCols + c,j,gw_cross_correlation_3[r * numCols + c][j]);
            printf("gw_auto_correlation_1[%d]=%d\n",r * numCols + c,gw_auto_correlation_1[r * numCols + c]);
            printf("gw_auto_correlation_2[%d]=%d\n",r * numCols + j,gw_auto_correlation_2[r *numCols + j]);
          }*/
          gw_normalized_score_4[r * numCols + c][j] = static_cast<float>(gw_cross_correlation_3[r * numCols + c][j]) / static_cast<float>(sqrt(static_cast<float>(gw_auto_correlation_1[r * numCols + c]) * static_cast<float>(gw_auto_correlation_2[r * numCols + j])));

        }
      }

    }
  }
  /*
   for (int j=0; j<max_disparity;j++){
    printf("gw_normalized_score_4[50306][%d]=%f\n",156+j, gw_normalized_score_4[50306][j]);
  }*/


  /*
  for (int j=0; j<max_disparity;j++) {
     printf("gw_normalized_score_4[200][%d]=%f\n",j,gw_normalized_score_4[200][j]);
  }*/
  printf("\nFLT_MIN=%f\n", FLT_MIN);
  printf("\nINT_MIN=%d\n", INT_MIN);
  float max_match_score = -1000;
  int match_index = INT_MIN;
  int search_range;

  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      max_match_score = -100000;
      match_index = INT_MIN;
      search_range = (c >= max_disparity) ? max_disparity : c;
      for (int j = 0; j <= search_range; j++)
      {
        if (gw_normalized_score_4[r * numCols + c][j] > max_match_score)
        {
          max_match_score = gw_normalized_score_4[r * numCols + c][j];
          match_index = j;
        }
      }
      if (search_range == max_disparity)
      {
        match_matrix[r * numCols + c] = c - max_disparity + match_index;
      }
      else
      {
        match_matrix[r * numCols + c] = match_index;
      }
      /*
      if ((r==239) &&(c==0)){
        printf("search_range = %d",search_range);
        for (int j=0; j<=search_range;j++){
          printf("At position (%d ,%d)[%d] the w_normalized_score_4[r * numCols + c][j]=%f is\n",r,c,j,gw_normalized_score_4[r * numCols + c][j]);
        }
      }*/
    }
  }
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      disparity_map[r * numCols + c] = abs(match_matrix[r * numCols + c] - c);
      //depth_map[r * numCols + c]=max_founded_disparity*(255.0*(1.0/static_cast<float>(disparity_map[r * numCols + c])));
    }
  }
  int max_founded_disparity = INT_MIN;
  int min_founded_disparity = INT_MAX;
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      if (disparity_map[r * numCols + c] > max_founded_disparity)
      {
        max_founded_disparity = disparity_map[r * numCols + c];
      }
      if (disparity_map[r * numCols + c] < min_founded_disparity && disparity_map[r * numCols + c] != 0)
      {
        min_founded_disparity = disparity_map[r * numCols + c];

      }
      /*
      if (disparity_map[r * numCols + c]==0){
          printf("At position (%d ,%d) the disparity is zero\n",r,c);
      }
      if (disparity_map[r * numCols + c]==-2147483648){
          printf("At position (%d ,%d) the disparity is -2147483648 and match matrix=%d \n",r,c,match_matrix[r * numCols + c]);
      }*/
    }
  }
  if (min_founded_disparity == 0)
  {
    min_founded_disparity += 1;
  }
  for (int r = 0; r < (int)numRows; ++r)
  {
    for (int c = 0; c < (int)numCols; ++c)
    {
      //disparity_map[r * numCols + c]=abs(match_matrix[r * numCols + c]-c);
      if (disparity_map[r * numCols + c] == 0)
      {
        depth_map[r * numCols + c] = 255;

      }
      else
      {
        depth_map[r * numCols + c] = (float)(max_founded_disparity) * (static_cast<float>(1.0) / (static_cast<float>(disparity_map[r * numCols + c])));
      }
    }
  }
  printf("max_founded_disparity= %d\n", max_founded_disparity);
  printf("min_founded_disparity= %d\n", min_founded_disparity);
  printf("match_matrix[111,356]=%d\n", match_matrix[111 * numCols + 356]);

  /*
  for (int j=350;j<361;j++){
      printf("gw_normalized_score_4[111][%d]=%f\n",j,gw_normalized_score_4[111 * numCols + 356][j]);
  }

  printf("\nmatch matrix:\n");
  for (int r = 180; r < 210; ++r) {
      for (int c = 180; c < 210; ++c) {
  printf("%d ",match_matrix[r * numCols + c]);
      }
      printf("\n");
  }*/
  printf("\ndisparity_map:\n");
  int r, c;
  for (r = 180; r < 220; ++r)
  {
    for (c = 180; c < 220; ++c)
    {
      printf("%d ", disparity_map[r * numCols + c]);
    }

    printf(" babak%d", r);
    printf("\n");
  }
  printf("r= %d c= %d  Goodbye lenin \n", r, c);
  printf("r= %d c= %d  Goodbye lenin \n", r, c);
  printf("%s\n", "Goodbye lenin");
  printf("%s\n", "hi karen1");
  /*
  printf("\ndepth map:\n");
  for (int r = 200; r < 210; ++r) {
      for (int c = 200; c < 210; ++c) {
  printf("%d ",depth_map[r * numCols + c]);
      }
      printf("\n");
  }*/
  /*
  for (int j=0; j<max_disparity;j++){
    printf("%f ",gw_normalized_score_4[200 * numCols + 200][j]);
  }
  printf("\n");
  for (int j=0; j<max_disparity;j++){
    printf("%f ",gw_normalized_score_4[200 * numCols + 201][j]);
  }
  printf("\n");
  for (int j=0; j<max_disparity;j++){
    printf("%f ",gw_normalized_score_4[200 * numCols + 202][j]);

  }
  for (int j=0; j<max_disparity;j++){
    printf("%f ",gw_cross_correlation_3[200 * numCols + 200][j]);

  }
  printf("\n");
  for (int j=0; j<max_disparity;j++){
    printf("%f ",gw_cross_correlation_3[200 * numCols + 201][j]);
  }
  printf("\n");
  for (int j=0; j<max_disparity;j++){
    printf("%f ",gw_cross_correlation_3[200 * numCols + 202][j]);
  }
  printf("\n");
   */
  printf("%s\n", "hi karen2");
  int min_value = disparity_map[0], max_value = disparity_map[0];
  for (int i = 0; i < numRows * numCols; ++i)
  {
    //outputImage_1[i] =static_cast<int> (depth_map[i]);
    //printf("disparity_map[%d]=%d\n",i,disparity_map[i]);

    outputImage_1[i] = static_cast<uchar>(disparity_map[i]);
    if (disparity_map[i] == -2147483648)
    {
      printf("disparity_map[%d][%d]=%d\n", i / 450, i - ((i / 450) * 450), disparity_map[i]);
    }
    if (disparity_map[i] < min_value)
      min_value = disparity_map[i];
    if (disparity_map[i] > max_value)
      max_value = disparity_map[i];
    //printf("hi babak");
    //printf("outputImage_1[%d]=%d\n",i,outputImage_1[i]);

    //outputImage_2[i] = disparity_map_2[i];
  }
  printf("min_value=%d\n", min_value);
  printf("max_value=%d\n", max_value);

// delete[] grayfiltered_1;
// delete[] grayfiltered_2;


}

/************CPU_postprocess*******/
void CPU_postProcess(const std::string& output_file, uchar* data_ptr)
{
  cv::Mat output(numRows, numCols, CV_8UC1, (void*)data_ptr);

  Mat img_hist_equalized;
  equalizeHist(output, img_hist_equalized); //equalize the histogram

  //cv::Mat imageOutputBGR;
  //cv::cvtColor(output, imageOutputBGR, CV_RGBA2BGR);
  //cv::cvtColor(output, imageOutputBGR, CV_GRAY2BGR);
  //output the image
  printf("%s\n", "hi karen3");
  //cv::imwrite(output_file.c_str(), imageOutputBGR);

  cv::imwrite(output_file.c_str(), img_hist_equalized);

  //cout << output << endl;

  //cv::imwrite(output_file.c_str(), output);
}

/************preprocess**********/
void CPU_Preprocess(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2,
                    uchar **h_outputImageGray_1, uchar **h_outputImageGray_2,
                    float **h_filter, int *filterWidth, const std::string &filename_1, const std::string &filename_2)
{

  cv::Mat image_1 = cv::imread(filename_1.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::Mat image_2 = cv::imread(filename_2.c_str(), CV_LOAD_IMAGE_COLOR);

  if (image_1.empty() || image_2.empty())
  {
    std::cerr << "Couldn't open file: " << filename_1 << "or" << filename_2 << std::endl;
    exit(1);
  }

  //copy the transformed image from first parameter to the second one
  cv::cvtColor(image_1, imageInputGray_1, CV_BGR2GRAY);
  cv::cvtColor(image_2, imageInputGray_2, CV_BGR2GRAY);

  //allocate memory for the output

  imageOutputGray_1.create(image_1.rows, image_1.cols, CV_8UC1);
  imageOutputGray_2.create(image_2.rows, image_2.cols, CV_8UC1);

  if (!imageInputGray_1.isContinuous() || !imageOutputGray_1.isContinuous() ||
      !imageInputGray_2.isContinuous() || !imageOutputGray_2.isContinuous())
  {
    std::cerr << "Images aren't continuous!! Exiting." << std::endl;
    exit(1);
  }
  *h_inputImageGray_1  = (uchar *)imageInputGray_1.ptr<unsigned char>(0);
  *h_inputImageGray_2  = (uchar *)imageInputGray_2.ptr<unsigned char>(0);
  *h_outputImageGray_1 = (uchar *)imageOutputGray_1.ptr<unsigned char>(0);
  *h_outputImageGray_2 = (uchar *)imageOutputGray_2.ptr<unsigned char>(0);
  numRows = imageInputGray_1.rows;
  numCols = imageInputGray_1.cols;

  const int numPixels = numRows * numCols;
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


}

/**********cleanUp**********/
void CPU_cleanUp(uchar **h_inputImageGray_1, uchar **h_inputImageGray_2, uchar **h_outputImageGray_1, uchar **h_outputImageGray_2)
{


  delete[] h_inputImageGray_1;
  delete[] h_inputImageGray_2;
  delete[] h_outputImageGray_1;
  delete[] h_outputImageGray_2;
  delete[] h_filter__;

}

//***********************
void CPU_Part(int argc, char **argv)
{

  uchar *h_inputImageGray_1, *h_inputImageGray_2;
  uchar *h_outputImageGray_1, *h_outputImageGray_2;
  //unsigned char *d_GrayBlurred,*d_GrayBlurred_1,*d_redBlurred,*d_redBlurred_1, *d_greenBlurred,*d_greenBlurred_1, *d_blueBlurred,*d_blueBlurred_1;

  float *h_filter;
  int    filterWidth;

  std::string input_file_1;
  std::string input_file_2;
  std::string output_file_1;
  std::string output_file_2;
  std::string reference_file;
  double perPixelError = 0.0;
  double globalError   = 0.0;
  bool useEpsCheck = false;
  switch (argc)
  {
  case 2:
    input_file_1 = std::string(argv[1]);
    output_file_1 = "nwcc_output_1.png";
    output_file_2 = "nwcc_output_2.png";
    reference_file = "nwcc_reference.png";
    break;
  case 3:
    input_file_1  = std::string(argv[1]);
    input_file_2 = std::string(argv[2]);
    output_file_1 = "nwcc_output_1.png";
    output_file_2 = "nwcc_output_2.png";
    reference_file = "nwcc_reference.png";
    break;
  case 4:
    input_file_1  = std::string(argv[1]);
    output_file_1 = std::string(argv[2]);
    reference_file = std::string(argv[3]);
    break;
  case 6:
    useEpsCheck = true;
    input_file_1  = std::string(argv[1]);
    output_file_1 = std::string(argv[2]);
    reference_file = std::string(argv[3]);
    perPixelError = atof(argv[4]);
    globalError   = atof(argv[5]);
    break;
  default:
    std::cerr << "Usage: ./nwcc input_file_1  input_file_2  [output_filename] [reference_filename] [perPixelError] [globalError]" << std::endl;
    exit(1);
  }
  CPU_Preprocess(&h_inputImageGray_1, &h_inputImageGray_2,
                 &h_outputImageGray_1, &h_outputImageGray_2,
                 &h_filter, &filterWidth, input_file_1, input_file_2);


  CPU_referenceCalculation(h_inputImageGray_1, h_inputImageGray_2, h_outputImageGray_1, h_outputImageGray_2,
                           numRows, numCols,
                           h_filter, filterWidth);

  CPU_postProcess(output_file_1, h_outputImageGray_1);
  //CPU_postProcess(output_file_2, h_outputImageGray_2);


  //CPU_postProcess(reference_file, h_outputImageRGBA);
  //  Cheater easy way with OpenCV
  //generateReferenceImage(input_file, reference_file, filterWidth);
  //compareImages(reference_file, output_file, useEpsCheck, perPixelError, globalError);

//  CPU_cleanUp(&h_inputImageGray_1, &h_inputImageGray_2, &h_outputImageGray_1, &h_outputImageGray_2);
}
