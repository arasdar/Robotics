

#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"

using namespace finroc::stereo_traversability_experiments::aras::libstereo;

//////////////////////////////////////////////////////////////////////////////
void
tTestStereoMatching::medianFilter(int radius)
{

  unsigned side = radius * 2 + 1;

  short int *out = new short int [width_ * height_];
  memset(out, 0, width_ * height_ * sizeof(short int));

  for (unsigned y = radius; y < height_ - radius; y++)
  {
    for (unsigned x = radius; x < width_ - radius; x++)
    {

      if (disp_map_[y * width_ + x] <= 0)
      {
        out[y * width_ + x] = disp_map_[y * width_ + x];
      }
      else
      {

        int n = 0;
        short int *v = new short int [side * side];

        for (signed j = -radius; j <= radius; j++)
        {
          for (signed i = -radius; i <= radius; i++)
          {
            if (disp_map_[(y + j)*width_ + x + i] > 0)
            {
              v[n] = disp_map_[(y + j) * width_ + x + i];
              n++;
            }
          }// for i
        }// for j

        std::sort(v, v + n);
        out[y * width_ + x] = v[n / 2];

        delete [] v;
      }// else
    }
  }

  short int* temp_ptr = out;
  out = disp_map_;
  disp_map_ = temp_ptr;

  delete [] out;
}
