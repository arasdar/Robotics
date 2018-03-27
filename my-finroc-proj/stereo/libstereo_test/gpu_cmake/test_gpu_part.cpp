//#include "projects/stereo_traversability_experiments/aras/libstereo_test/gpu_finroc_test/gpu_part.h"
#include "gpu_part.h"


using namespace finroc::stereo_traversability_experiments::aras::libstereo_test::gpu;

int main(int argc, char **argv)
{
  gpu_part test;
  test.GPU_run(argc, argv);

  return 0;
}
