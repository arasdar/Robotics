/*
 * tStereoProc_processCloud_trav.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_original/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_original::offline;

void tStereoProcessing::processCloud_trav()
{

  processCloud_trav_segm2plane();
  processCloud_trav_slopeAnalysis();
  processCloud_trav_dominGroundPlane();
  processCloud_trav_stepAnalysis();
}



