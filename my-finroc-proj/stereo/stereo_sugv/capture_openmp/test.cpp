
#include <FlyCapture2.h>

#include <omp.h>

#include <iostream>

using namespace std;
using namespace FlyCapture2;

class StereoVisionProcessing
{
private:

public:
  StereoVisionProcessing()
  { } // constructor

  ~StereoVisionProcessing()
  {  }

  void
  run()
  {

//      Image test;  // using this class will make u only have one thread available?????????????

    cout << "openmp trying to get max threads but the actual avaialable threads are : " << omp_get_max_threads() << endl;

    Image test;  // using this class will make u only have one thread available?????????????

    cout << "openmp trying to get max threads but the actual avaialable threads are : " << omp_get_max_threads() << endl;

  }// run

  void run_2()
  {
    cout << "openmp trying to get max threads but the actual avaialable threads are : " << omp_get_max_threads() << endl;

    Image test;  // using this class will make u only have one thread available?????????????

    cout << "openmp trying to get max threads but the actual avaialable threads are : " << omp_get_max_threads() << endl;

  }



};//class

int
main(int argc, char** argv)
{

  /*Process and display*/
  StereoVisionProcessing stereo_vision_processing; //(/*left_images, right_images, img_pairs_num, input_intrinsic_filename, input_extrinsic_filename*/);
  stereo_vision_processing.run();
  stereo_vision_processing.run_2();

  return 0;
}// main
