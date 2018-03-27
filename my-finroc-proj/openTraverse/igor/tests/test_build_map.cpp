
#include "projects/stereo_traversability_experiments/openTraverse/igor/worker.h"

//#include <ad_hoc_normal_extraction/utils/command_args.h>

using namespace std;

using namespace finroc::stereo_traversability_experiments::openTraverse::igor;

//int main(int argc, char** argv)
int main()
{
//  int maxAngle;
//  float cellWidth, maxTravHeight, radiusNormals, robotHeight;
//  string outputImage;
//
//  // Input parameters handling.
//  g2o::CommandArgs arg;
//
//  //parameters for traversability estimation
//  arg.param("maxAngle", maxAngle, 30,"maximal traversable slope incline in degrees. [int]");
//  arg.param("cellWidth", cellWidth, 0.04f,"cell width in meters. [float]");
//  arg.param("maxTravHeight", maxTravHeight, 0.1f, "maximal traversable step height in meters. [float]");
//  arg.param("radiusNormals", radiusNormals, 0.1f,"radius for searching normals in meters. [float]");
//  arg.param("robotHeight", robotHeight,1.0f,"robot height in meters. [float]");
//  arg.param("outputImage", outputImage,"traversability.png","file name for 2d map output. [string]");
//
//  arg.parseArgs(argc, argv);

  int maxAngle = 30;
  float cellWidth = 0.04f,
        maxTravHeight = 0.1f,
        radiusNormals = 0.1f,
        robotHeight = 1.0f;
  string outputImage = "output.png";

  Worker worker;
  worker.processPCLToMap(
    maxAngle,
    cellWidth,
    maxTravHeight,
    radiusNormals,
    robotHeight,
    outputImage);

  return 0;
}
