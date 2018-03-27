
#include "projects/stereo_traversability_experiments/aras/libstereo/tTestStereoMatching.h"


using namespace finroc::stereo_traversability_experiments::aras::libstereo;

//////////////////////////////////////////////////////////////////////////////
void
tTestStereoMatching::preProcessing(unsigned char *img, unsigned char *pp_img)
{
  int radius = 4;              //default value, could be exported
  int n = 2 * radius + 1;
  int area = n * n;
  int threshold = 31;

  for (int y = 0; y <= radius; y++)
    for (int x = 0; x < width_; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

  for (int y = height_ - radius; y < height_; y++)
    for (int x = 0; x < width_; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

  for (int y = radius + 1; y < height_ - radius; y++)
    for (int x = 0; x <= radius; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

  for (int y = radius + 1; y < height_ - radius; y++)
    for (int x = width_ - radius; x < width_; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

  int *v = new int[width_];
  memset(v, 0, sizeof(int) * width_);

  for (int x = 0; x < width_; x++)
    for (int y = 0; y < n; y++)
    {
      v[x] += img[y * width_ + x];
    }

  for (int y = radius + 1; y < height_ - radius; y++)
  {

    int sum = 0;
    for (int x = 0; x < n; x++)
    {
      v[x] += img[(y + radius) * width_ + x] - img[(y - radius - 1) * width_ + x];
      sum += v[x];
    }

    for (int x = radius + 1; x < width_ - radius; x++)
    {
      v[x + radius] += img[(y + radius) * width_ + x + radius] - img[(y - radius - 1) * width_ + x + radius];
      sum += v[x + radius] - v[x - radius - 1];

      short int temp = static_cast<short int>(img[y * width_ + x] - (sum / area));

      if (temp < -threshold)
        pp_img[y * width_ + x] = 0;

      if (temp > threshold)
        pp_img[y * width_ + x] = static_cast<unsigned char>(threshold + threshold);

      if (temp > -threshold && temp < threshold)
        pp_img[y * width_ + x] = static_cast<unsigned char>(temp + threshold);
    }

  }// for y

  delete [] v;

}
