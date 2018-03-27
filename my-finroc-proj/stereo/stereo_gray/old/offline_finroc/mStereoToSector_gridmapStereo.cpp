
#include "projects/icarus/sensor_processing/stereo_gray/offline_finroc/mStereoToSectorMap.h"

using namespace finroc::icarus::sensor_processing::stereo_gray::offline;


/*//const double robot_width = 2, robot_height = 3.3; //this is based on vehicle size //lugv*/
const double robot_width = 0.5, robot_height = 1.0; //this is based on vehicle size //sugv
const double stereo_baseline = 0.21;


void inline drawStereo(rrlib::canvas::tCanvas2D& canvas);
void inline drawRobot(rrlib::canvas::tCanvas2D& canvas);

void mStereoToSectorMap::gridmapStereo()
{

  /* --------------> copy the gridmap <------------------------------------*/
  data_ports::tPortDataPointer<const rrlib::canvas::tCanvas2D> input_canvas = input_canvas_gridmap.GetPointer();
  data_ports::tPortDataPointer<rrlib::canvas::tCanvas2D> output_canvas = output_canvas_gridmap.GetUnusedBuffer();

  rrlib::canvas::tCanvas2D& temp = *output_canvas;
  memcpy(&temp, input_canvas, sizeof(rrlib::canvas::tCanvas2D));

  /*! draw robot in Current robot position */
  /*
  //  const rrlib::math::tPose2D stereo_leftCamera_pose = rrlib::math::tPose2D(stereo_baseline * 0.5,
  //      -1 * robot_height * 0.5, 0);
  //  output_canvas->SetAlpha(200);
  //  output_canvas->Transform(stereo_leftCamera_pose);
  //  drawRobot(*output_canvas);
  //  drawStereo(*output_canvas);
  */

  drawSectorPolar(*output_canvas, *sectormap_polar);

  output_canvas_gridmap.Publish(output_canvas);
}// gridmap


inline void drawRobot(rrlib::canvas::tCanvas2D& canvas)
{
  canvas.SetFill(true);
  canvas.SetEdgeColor(0, 0, 0);
  canvas.SetFillColor(200, 200, 255);

  rrlib::math::tVec2d robot_bottom_left = rrlib::math::tVec2d(-robot_width / 2, -robot_height / 2);

  rrlib::math::tVec2d robot_pose_arrow_start = rrlib::math::tVec2d(0.0, -robot_height / 2);
  rrlib::math::tVec2d robot_pose_arrow_end   = rrlib::math::tVec2d(0.0, robot_height / 2);

  canvas.DrawBox(robot_bottom_left, robot_width, robot_height);
  canvas.DrawArrow(robot_pose_arrow_start, robot_pose_arrow_end);
}

inline void drawStereo(rrlib::canvas::tCanvas2D& canvas)
{

  const rrlib::math::tVec2d left = rrlib::math::tVec2d(-1 * stereo_baseline * 0.5, robot_height * 0.5);
  rrlib::math::tVec2d left_arrow_start = rrlib::math::tVec2d(0.0, -0.1);
  rrlib::math::tVec2d left_arrow_end   = rrlib::math::tVec2d(0.0, 0.0);
  left_arrow_start += left;
  left_arrow_end += left;
  canvas.DrawArrow(left_arrow_start, left_arrow_end);

  const rrlib::math::tVec2d right = rrlib::math::tVec2d(stereo_baseline * 0.5, robot_height * 0.5);
  rrlib::math::tVec2d right_arrow_start = rrlib::math::tVec2d(0.0, -0.1);
  rrlib::math::tVec2d right_arrow_end   = rrlib::math::tVec2d(0.0, 0.0);
  right_arrow_start += right;
  right_arrow_end += right;
  canvas.DrawArrow(right_arrow_start, right_arrow_end);

}


