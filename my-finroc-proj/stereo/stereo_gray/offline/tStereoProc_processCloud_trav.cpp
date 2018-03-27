/*
 * tStereoProc_processCloud_trav.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/aras/stereo_gray/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::aras::stereo_gray::offline;

void tStereoProcessing::processCloud_trav()
{

  processCloud_trav_segm2plane();
  processCloud_trav_slopeAnalysis();
  processCloud_trav_dominGroundPlane();
  processCloud_trav_stepAnalysis();
}



