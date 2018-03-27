
#include "projects/stereo_traversability_experiments/openTraverse/aras/tSSTA.h"

using namespace finroc::stereo_traversability_experiments::openTraverse::aras;

void tSSTA::processCloud_trav()
{

  processCloud_trav_segm2plane();
  processCloud_trav_slopeAnalysis();
  processCloud_trav_dominGroundPlane();
  processCloud_trav_stepAnalysis();
}



