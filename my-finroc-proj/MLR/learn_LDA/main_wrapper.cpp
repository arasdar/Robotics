/*
 * Copyright (c) 2011. Philipp Wagner <bytefish[at]gmx[dot]de>.
 * Released to public domain under terms of the BSD Simplified license.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the organization nor the names of its contributors
 *     may be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *   See <http://www.opensource.org/licenses/bsd-license>
 */

//#include "opencv2/opencv.hpp"
//#include "opencv2/highgui/highgui.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

#include "projects/stereo_traversability_experiments/mlr/learn_LDA/LinearDiscriminantAnalysis.hpp"

using namespace cv;
using namespace std;

int main(int argc, const char *argv[])
{
  // Example for a Linear Discriminant Analysis
  // (example taken from: http://www.bytefish.de/wiki/pca_lda_with_gnu_octave)
  double d[11][2] =
  {
    {2, 3},
    {3, 4},
    {4, 5},
    {5, 6},
    {5, 7},
    {2, 1},
    {3, 2},
    {4, 2},
    {4, 3},
    {6, 4},
    {7, 6}
  };
  int c[11] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
  // convert into OpenCV representation
  Mat _data = Mat(11, 2, CV_64FC1, d).clone();
  vector<int> _classes(c, c + sizeof(c) / sizeof(int));
  // perform the lda
  finroc::stereo_traversability_experiments::mlr::learn_LDA::LinearDiscriminantAnalysis lda(_data, _classes);
  // GNU Octave finds the following Eigenvalue:
  //octave> d
  //d =
  //   1.5195e+00
  //
  // Eigen finds the following Eigenvalue:
  // [1.519536390756363]
  //
  // Since there's only 1 discriminant, this is correct.
  cout << "Eigenvalues:" << endl << lda.eigenvalues() << endl;
  // GNU Octave finds the following Eigenvectors:
  //  octave:13> V(:,1)
  //  V =
  //
  //     0.71169  -0.96623
  //    -0.70249  -0.25766
  //
  // Eigen finds the following Eigenvector:
  // [0.7116932742510111;
  //  -0.702490343980524 ]
  //
  cout << "Eigenvectors:" << endl << lda.eigenvectors() << endl;
  // project a data sample onto the subspace identified by LDA
  Mat x = _data.row(0);
  cout << "Projection of " << x << ": " << endl;
  cout << lda.project(x) << endl;

  return 0;
}
