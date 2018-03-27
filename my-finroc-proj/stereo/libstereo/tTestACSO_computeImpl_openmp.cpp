

#include "projects/stereo_traversability_experiments/aras/libstereo/tTestACSO.h"

// OpenMP
#include <omp.h>

using namespace finroc::stereo_traversability_experiments::aras::libstereo;

const float numeric_limits_float_max = std::numeric_limits<float>::max();

/////////////////////////////////////
// kernel function for openmp (CPU)
/////////////////////////////////////
void tTestACSO::compute_impl_parallel_openmp(const unsigned char* ref_img, const unsigned char* trg_img,
    const float *ds, const float* lut)
{

  /*
  * main loop to iterate through the entire left and right images
  */
  std::cout << "OpenMP version using " << omp_get_max_threads() << " threads" << std::endl;

  #pragma omp parallel for
  /*! scanning Y - row - height of the image*/
  for (unsigned y = radius_ + 1; y < height_ - radius_; y++) //original
  {

    /*
     * initializing or constructing the variables
     */
    float *acc = new float[width_ * max_disp_];
    memset(acc, 0, sizeof(float) * width_ * max_disp_);

    //left weight array alloc
    float *wl = new float [ 2 * radius_ + 1 ];

    //data structures for Scanline Optimization backward and forward
    float *bck = new float[width_ * max_disp_];
    memset(bck, 0, sizeof(float) * width_ * max_disp_);
    float *fwd = new float[width_ * max_disp_];
    memset(fwd, 0, sizeof(float) * width_ * max_disp_);

    /*! 1st scan X - accumulator for SAD*/
    for (unsigned x = x_off_ + max_disp_ + 1; x < width_; x++)
    {

      for (signed int j = -radius_; j <= radius_; j++)
      {
        wl[j + radius_] = lut[ abs(ref_img[(y + j) * width_ + x] - ref_img[y * width_ + x]) ] * ds[j + radius_];
      }// j

      for (unsigned int d = 0; d < max_disp_; d++)
      {
        float sumw  = 0.0;
        float num = 0.0;

        for (signed int j = -radius_; j <= radius_; j++)
        {
          float weight_r = lut[ abs(trg_img[(y + j) * width_ + x - d - x_off_] - trg_img[y * width_ + x - d - x_off_]) ] * ds[j + radius_];
          int sad = abs(ref_img[(y + j) * width_ + x] - trg_img[(y + j) * width_ + x - d - x_off_]);
          num += wl[j + radius_] * weight_r * static_cast<float>(sad);
          sumw += wl[j + radius_] * weight_r;
        }

        acc[(x * max_disp_) + d] = num / sumw;
      }//d
    }//x

    /*! 2st scan X - forward scanline*/
    for (unsigned d = 0; d < max_disp_; d++)
    {
      fwd[(x_off_ + max_disp_ + 1) * max_disp_ + d] = acc[(x_off_ + max_disp_ + 1) * max_disp_ + d];
    }

    for (unsigned int x = x_off_ + max_disp_ + 2; x < width_; x++)
    {
      float c_min = fwd[((x - 1) * max_disp_) + 0];
      for (int d = 1; d < max_disp_; d++)
        if (fwd[((x - 1) * max_disp_) + d] < c_min)
          c_min = fwd[((x - 1) * max_disp_) + d];

      /*! first */
      float min_main = std::min(fwd[((x - 1) * max_disp_) + 0],
                                std::min(fwd[((x - 1) * max_disp_) + 1] + static_cast<float>(smoothness_weak_),
                                         c_min + static_cast<float>(smoothness_strong_)));

      fwd[(x * max_disp_) + 0] =  acc[(x * max_disp_) + 0] - c_min + min_main;



      /*! middle */
      for (unsigned d = 1; d < max_disp_ - 1; d++)
      {

        min_main = std::min(std::min(fwd[((x - 1) * max_disp_) + d], fwd[((x - 1) * max_disp_) + d - 1] + static_cast<float>(smoothness_weak_)),
                            std::min(fwd[((x - 1) * max_disp_) + d + 1] + static_cast<float>(smoothness_weak_), c_min + static_cast<float>(smoothness_strong_)));

        fwd[(x * max_disp_) + d] = acc[(x * max_disp_) + d] - c_min + min_main;
      }

      /*! last*/
      min_main = std::min(fwd[((x - 1) * max_disp_) + max_disp_ - 1],
                          std::min(fwd[((x - 1) * max_disp_) + max_disp_ - 2] + static_cast<float>(smoothness_weak_),
                                   c_min + static_cast<float>(smoothness_strong_)));

      fwd[(x * max_disp_) + max_disp_ - 1] = acc[(x * max_disp_) + max_disp_ - 1] - c_min + min_main;
    }//x Forward


    /*! 3rd scan X - backward scanline*/
    for (unsigned d = 0; d < max_disp_; d++)
    {
      bck[(width_ - 1) * max_disp_ + d] = acc[(width_ - 1) * max_disp_ + d];
    }

    for (unsigned x = width_ - 2; x > max_disp_ + x_off_; x--)
    {

      float c_min = bck[((x + 1) * max_disp_) + 0];
      for (int d = 1; d < max_disp_; d++)
        if (bck[((x + 1) * max_disp_) + d] < c_min)
          c_min = bck[((x + 1) * max_disp_) + d];

      /*! first */
      float min_main = std::min(bck[((x + 1) * max_disp_) + 0],
                                std::min(bck[((x + 1) * max_disp_) + 1] + static_cast<float>(smoothness_weak_),
                                         c_min + static_cast<float>(smoothness_strong_)));

      bck[(x * max_disp_) + 0] =  acc[(x * max_disp_) + 0] - c_min + min_main;

      /*! middle */
      for (unsigned d = 1; d < max_disp_ - 1; d++)
      {
        min_main = std::min(std::min(bck[((x + 1) * max_disp_) + d],
                                     bck[((x + 1) * max_disp_) + d - 1] + static_cast<float>(smoothness_weak_)),
                            std::min(bck[((x + 1) * max_disp_) + d + 1] + static_cast<float>(smoothness_weak_),
                                     c_min + static_cast<float>(smoothness_strong_)));

        bck[(x * max_disp_) + d] = acc[(x * max_disp_) + d] - c_min + min_main;
      }

      /*! last*/
      min_main = std::min(bck[((x + 1) * max_disp_) + max_disp_ - 1],
                          std::min(bck[((x + 1) * max_disp_) + max_disp_ - 2] + static_cast<float>(smoothness_weak_),
                                   c_min + static_cast<float>(smoothness_strong_)));

      bck[(x * max_disp_) + max_disp_ - 1] = acc[(x * max_disp_) + max_disp_ - 1] - c_min + min_main;
    }//x Backward

    /*! 4th scan X - last scan - to fill disp_map_*/
    for (unsigned int x = x_off_ + max_disp_ + 1; x < width_; x++)
    {
      float c_min = numeric_limits_float_max;
      short int dbest = 0;

      for (uint d = 0; d < max_disp_; d++)
      {
        acc[(x * max_disp_) + d] = fwd[(x * max_disp_) + d] + bck[(x * max_disp_) + d]; //2pass scanline optimization

        if (acc[(x * max_disp_) + d] < c_min)
        {
          c_min = acc[(x * max_disp_) + d];
          dbest = static_cast<short int>(d);
        }
      }

      disp_map_[(y * width_) + x] = static_cast<short int>(dbest * 16);
    } //x last scan for disp_map_

    /*
     * deconstructing the variables on cpu
     */
    delete [] acc;
    delete [] wl;
    delete [] fwd;
    delete [] bck;

  }// main loop for y

}// end of compute parallel using openmp
