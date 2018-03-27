


//#include "UPD.h"
#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/registration/transforms.h>


namespace finroc
{
namespace mauro
{
namespace test
{

int  UPD::run(int argc, char** argv)
{
  //system ("clear");   //clear the shell screen

  string file_name;
//  struct timespec start, end;
  uint64_t timeElapsed;


  if (console::find_argument(argc, argv, "-m") >= 0)
  {
    ifstream readSavedFile;
    if (argc > 2)
    {
      readSavedFile.open(argv[2]);
      cout << "File list read ! " << endl;
    }
    else
    {
      console::print_error(" Attention File list not read !!!! \n\n");
    }
    char SavedFileName[100];
    struct timespec start, end;

    clock_gettime(CLOCK_MONOTONIC, &start);

    if (readSavedFile.is_open())
    {
      int first_file_flag = 0;
      visualization::PCLVisualizer viewer("viewer");

      do
      {
        if (first_file_flag == 0)
        {
          readSavedFile >> file_name;//pcd_input_filename;

          if (io::loadPCDFile(file_name, *image_cloud) == true)
            cout << "Loaded " << image_cloud->size() << " data points from " << file_name << endl;

          cout << "Read = " << file_name << endl;//pcd_input_filename << std::endl;
          upd();
          first_file_flag = 1;
          //////////////////////////inserire salvataggio!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }
        else
        {
          if (!viewer.updatePointCloud(image_cloud, "cloud"))
          {
            viewerOneOff(viewer);
          }

          if (!viewer.updatePointCloud(colored_cloud, "C-cloud"))
          {
            viewerPsycho(viewer);
          }

          viewer.spinOnce(1);

          readSavedFile >> file_name;//pcd_input_filename;
          if (io::loadPCDFile(file_name, *image_cloud) == true)
            cout << "Loaded " << image_cloud->size() << " data points from " << file_name << endl;
          cout << "Read = " << file_name << endl;//pcd_input_filename << std::endl;
          upd();

        }
      }
      while (!readSavedFile.eof());

      clock_gettime(CLOCK_MONOTONIC, &end);
      timeElapsed = timespecDiff(&end, &start);
      timeElapsed = timeElapsed * 0.000001;  //time is converted in msec
      cout << "Completed --- time elapsed ==> " << timeElapsed << " ms    CIAOOO!!!!\n" << endl;
    }
    return 0;
  }

  if (console::find_argument(argc, argv, "") <= 0)
  {
    cout << "\n\E[00;31m\033[1mArgument not recognized ... \033[0m\n\n" << endl;
    cout << "\E[00;34m\33[1m-m ---> Normal analysis on a file list\n";
    cout << "-n ---> Normal analysis test \n\033[0m" << endl;
    return 0;
  }
  return 0;
}

}
}
}
