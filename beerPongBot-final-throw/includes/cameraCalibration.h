#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <vector>
#include "opencv2/core.hpp"

void calibrateCamera ();

bool calibrateCorners (cv::Mat frame, bool cups, std::vector<int>& points);

#endif