/*
 * tStereoProc_processCloud_trav.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */


#include "projects/stereo_traversability_experiments/daniel/stereo_color_hybrid/offline/tStereoProcessing.h"

using namespace finroc::stereo_traversability_experiments::daniel::stereo_color_hybrid::offline;

void tStereoProcessing::processCloud_trav()
{

  processCloud_trav_segm2plane();
  processCloud_trav_slopeAnalysis();
  processCloud_trav_dominGroundPlane();
  processCloud_trav_stepAnalysis();

  processCloud_trav_segm2plane_appear();
  processCloud_trav_slopeAnalysis_appear();
  processCloud_trav_stepAnalysis_appear();
  processCloud_trav_geom_fuse_and_map();
  processCloud_trav_slope_and_step_mapping_discrete();
  processCloud_trav_fuse_pessimistically();

}



