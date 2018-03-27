/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/** \brief Adaptive Cost 2-pass Scanline Optimization Stereo Matching algorithm implementation
  * please see related documentation on stereo/stereo_matching.h
  *
  * \author Aras Dargazany
  * \ingroup stereo matching
  */

//#include "projects/icarus/sensor_processing/libstereo/tests/stereo_matching.h"
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

#include "projects/stereo_traversability_experiments/aras/libstereo/gpu/tDeviceWrapper.hpp"
#include "projects/stereo_traversability_experiments/aras/libstereo/tTimer.hpp"


#include <stdio.h> //stderr - fprintf
//#include <iostream> //cout - cerr
//#include <iomanip> //setprecision


const int radius_ = 5;
//const int smoothness_strong_ = 100;
//const int smoothness_weak_ = 20;
const int max_disp_ = 60;
const int x_off_ = 0;
const float numeric_limits_float_max = 1e+20; //std::numeric_limits<float>::max();

/////////////////////////////////////
// error checking routine
/////////////////////////////////////
void checkErrors(char *label)
{
  // we need to synchronise first to catch errors due to
  // asynchroneous operations that would otherwise
  // potentially go unnoticed

  cudaError_t err;

  err = cudaThreadSynchronize();
  if (err != cudaSuccess)
  {
    char *e = (char*) cudaGetErrorString(err);
    fprintf(stderr, "CUDA Error: %s (at %s)", e, label);
  }

  err = cudaGetLastError();
  if (err != cudaSuccess)
  {
    char *e = (char*) cudaGetErrorString(err);
    fprintf(stderr, "CUDA Error: %s (at %s)", e, label);
  }
}

/////////////////////////////////////
// kernel function (CUDA device)
/////////////////////////////////////
__global__ void compute_parallel(const unsigned char* ref_img, const unsigned char* trg_img,
                                 const unsigned int height_, const unsigned int width_,
                                 const float *ds, const float* lut,
                                 float* wl, float* acc,
                                 float *fwd,
                                 float *bck,
                                 short int* disp_map_)

{
  // compute the global index in the vector from
  // the number of the current block, blockIdx,
  // the number of threads per block, blockDim,
  // and the number of the current thread within the block, threadIdx
  //int i = blockIdx.x * blockDim.x + threadIdx.x; //rows -- y -- height

  // except for special cases, the total number of threads in all blocks
  // adds up to more than the vector length n, so this conditional is
  // EXTREMELY important to avoid writing past the allocated memory for
  // the vector y.
  /*
  * main loop to iterate through the entire left and right images
  */


  shared int* array;
  uint y_min = radius_ + 1;
  uint y_max = height_ - radius_;
  uint y = blockIdx.x + y_min;

  /*! scanning Y - row - height of the image*/
  if (y < y_max)
  {
    /*! 1st scan X - accumulator for SAD*/
    uint x_min = x_off_ + max_disp_ + 1;
    uint x_max = width_;
    uint x = threadIdx.x + x_min;

//    float wl[2 * radius_ + 1]; //  float *wl = new float [ 2 * radius_ + 1 ];
////    cudaMalloc((void**)&wl, (2 * radius_ + 1)* sizeof(float));
//    float acc[686 * max_disp_]; //width_ = 686
////    cudaMalloc((void**)&acc, width_ * max_disp_ * sizeof(float));
////    cudaMemset(acc, 0, width_ * max_disp_ * sizeof(float));


    if (x < x_max)
    {

      for (int j = -radius_; j <= radius_; j++)
      {
        wl[j + radius_] = lut[ abs(ref_img[(y + j) * width_ + x] - ref_img[y * width_ + x]) ] * ds[j + radius_];
      }// j

      for (int d = 0; d < max_disp_; d++)
      {
        float sumw  = 0.0;
        float num = 0.0;

        for (int j = -radius_; j <= radius_; j++)
        {
          float weight_r = lut[ abs(trg_img[(y + j) * width_ + x - d - x_off_] - trg_img[y * width_ + x - d - x_off_]) ] * ds[j + radius_];
          int sad = abs(ref_img[(y + j) * width_ + x] - trg_img[(y + j) * width_ + x - d - x_off_]);
          num += wl[j + radius_] * weight_r * static_cast<float>(sad);
          sumw += wl[j + radius_] * weight_r;
        }

        //acc[x][d] = num / sumw;
        acc[(x * max_disp_) + d] = num / sumw;
      }//d

    }//x

    /*! 4th scan X - last scan - to fill disp_map_*/
    if (x < x_max)
    {
      float c_min = numeric_limits_float_max;
      short int dbest = 0;

      for (int d = 0; d < max_disp_; d++)
      {
        //acc[(x * max_disp_) + d] = fwd[(x * max_disp_) + d] + bck[(x * max_disp_) + d];
        if (acc[(x * max_disp_) + d] < c_min)
        {
          c_min = acc[(x * max_disp_) + d];
          dbest = static_cast<short int>(d);
        }
      }

      disp_map_[(y * width_) + x] = static_cast<short int>(dbest * 16);

    } //x last scan for disp_map_


  }// y_iter - row - height of img

}// end kernel

//////////////////////////////////////////////////////////////////////////////
void
finroc::stereo_traversability_experiments::aras::libstereo::tTestACSO::compute_impl_cuda(unsigned char* ref_img, unsigned char* trg_img)
{

  /////////////////////////////////////
  // some variables and configuration section
  /////////////////////////////////////
  const unsigned long N = width_ * height_; //123456;
//  std::cout << "width_: " << width_ << std::endl;
//  static int numThreadsPerBlock = 256;
  static int selectedDevice = 0;   // device to use in case there is more than one
  //std::cout << "numeric_limits_float_max: " << numeric_limits_float_max << std::endl;



  /////////////////////////////////////
  // (1) initialisations:
  //     - perform basic sanity checks
  //     - set device
  /////////////////////////////////////
  int deviceCount;
  cudaGetDeviceCount(&deviceCount);
  if (deviceCount == 0)
  {
    fprintf(stderr, "Sorry, no CUDA device fount");
    //return 1;
  }
  if (selectedDevice >= deviceCount)
  {
    fprintf(stderr, "Choose device ID between 0 and %d\n", deviceCount - 1);
    //return 1;
  }
  cudaSetDevice(selectedDevice);
  checkErrors("initialisations");

  /////////////////////////////////////
  // (2) initialise data on the CPU
  /////////////////////////////////////
  /*
   * initializaing or constructing the variables
   */
  float *wl_gpu = new float [ 2 * radius_ + 1 ];
  cudaMalloc((void**)&wl_gpu, (2 * radius_ + 1)* sizeof(float));
  float* acc_gpu; //width_ = 686
  cudaMalloc((void**)&acc_gpu, width_ * max_disp_ * sizeof(float));
  cudaMemset(acc_gpu, 0, width_ * max_disp_ * sizeof(float));

  //spatial distance init
  float *ds = new float[ 2 * radius_ + 1 ];
  for (int j = -radius_; j <= radius_; j++)
    ds[j + radius_] = static_cast<float>(exp(- abs(j) / gamma_s_));

  //LUT for color distance weight computation
  float lut[256];
  for (int j = 0; j < 256; j++)
    lut[j] = float(exp(-j / gamma_c_));

  /////////////////////////////////////
  // (3) allocate memory on host (main CPU memory) and device,
  //     h_ denotes data residing on the host, d_ on device
  /////////////////////////////////////
  unsigned char *ref_img_gpu;
  cudaMalloc((void**)&ref_img_gpu, height_ * width_ * sizeof(unsigned char));
  unsigned char *trg_img_gpu;
  cudaMalloc((void**)&trg_img_gpu, height_ * width_ * sizeof(unsigned char));
  float* ds_gpu;
  cudaMalloc((void**)&ds_gpu, (2 * radius_ + 1) * sizeof(float));
  float* lut_gpu;

  cudaMalloc((void**)&lut_gpu, 256 * sizeof(float));
  float* fwd_gpu;
  cudaMalloc((void**)&fwd_gpu, width_ * max_disp_ * sizeof(float));
  cudaMemset(fwd_gpu, 0, width_ * max_disp_ * sizeof(float));
  float* bck_gpu;
  cudaMalloc((void**)&bck_gpu, width_ * max_disp_ * sizeof(float));
  cudaMemset(bck_gpu, 0, width_ * max_disp_ * sizeof(float));

  /*! final output disparity map*/
  short int* disp_map_gpu_;
  cudaMalloc((void**)&disp_map_gpu_, height_ * width_ * sizeof(short int));
  cudaMemset(disp_map_gpu_, 0, sizeof(short int)*height_ * width_);

  checkErrors("memory allocation");


  /////////////////////////////////////
  // (4) copy data to device
  /////////////////////////////////////
  cudaMemcpy(trg_img_gpu, trg_img, N * sizeof(uchar), cudaMemcpyHostToDevice);
  cudaMemcpy(lut_gpu,     lut, 256 * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(ds_gpu, ds, (2 * radius_ + 1) * sizeof(float), cudaMemcpyHostToDevice);
  checkErrors("copy data to device");

  /////////////////////////////////////
  // (6) perform computation on device
  //     - we use numThreadsPerBlock threads per block
  //     - the total number of blocks is obtained by rounding the
  //       vector length N up to the next multiple of numThreadsPerBlock
  /////////////////////////////////////

  /*/
  // device kernel setup
  /*/
  //dim3 block(256,1,1);
//  dim3 block(1024, 1, 1);

  uint x_min = x_off_ + max_disp_ + 1;
  uint x_max = width_;
  uint x_length = x_max - x_min;
  dim3 block(x_length, 1, 1);

  //uint x_length = x_max - x_min;
  //dim3 grid((unsigned)ceil((N) / (double)(block.x)), 1, 1);
//  dim3 grid((unsigned)ceil((y_length) / (double)(block.x)), 1, 1);
//  dim3 grid(1, 1, 1);
  uint y_min = radius_ + 1;
  uint y_max = height_ - radius_;
  uint y_length = y_max - y_min;
  dim3 grid(y_length, 1, 1);


  //dim3 grid(65535, 1, 1);  //67107840
  std::cout << "Launch config: block(";
  std::cout << block.x << "x" << block.y << "x" << block.z << "), grid(";
  std::cout << grid.x << "x" << grid.y << "x" << grid.z << ")" << std::endl;
  //int numBlocks = (N + numThreadsPerBlock - 1) / numThreadsPerBlock;

  /*/
  // run kernel
  /*/
  cudaDeviceSynchronize();
  Timer timer;
  timer.start();
//  compute_parallel <<< grid, block>>>(ref_img_gpu, trg_img_gpu, height_, width_,
//                                      ds_gpu, lut_gpu,
//                                      fwd_gpu,
//                                      bck_gpu,
//                                      disp_map_gpu_);

//  compute_parallel <<<grid, block>>>(ref_img_gpu, trg_img_gpu, height_, width_,
//                                     ds_gpu, lut_gpu,
//                                     wl_gpu, acc_gpu,
//                                     fwd_gpu,
//                                     bck_gpu,
//                                     disp_map_gpu_);
  cudaDeviceSynchronize();
  double cudatime = timer.stop();
  checkErrors("compute on device");

  /*/
  // print out timinigs
  /*/
  double throughput = 3.0 * N * sizeof(double) / (cudatime * 1e9);
  std::cout << "CUDA implementation     : ";
  std::cout << "Time: " << std::fixed << std::setprecision(4) << cudatime << "s";
  std::cout << ", throughput: " << std::setprecision(2) << throughput << " GB/s";
  std::cout << std::endl;


  /////////////////////////////////////
  // (7) read back result from device into temp vector
  /////////////////////////////////////
  cudaMemcpy(disp_map_, disp_map_gpu_, N * sizeof(short int), cudaMemcpyDeviceToHost);
  checkErrors("copy data from device");

  /////////////////////////////////////
  // (9) clean up, free memory on gpu
  /////////////////////////////////////
  cudaFree(ref_img_gpu);
  cudaFree(trg_img_gpu);
  cudaFree(disp_map_gpu_);
  cudaFree(ds_gpu);
  cudaFree(lut_gpu);
  cudaFree(fwd_gpu);
  cudaFree(bck_gpu);
  cudaFree(wl_gpu);
  cudaFree(acc_gpu);


  /*
   * deconstructing the variables on cpu
   */
  delete [] ds;

} //compute_impl_cuda
