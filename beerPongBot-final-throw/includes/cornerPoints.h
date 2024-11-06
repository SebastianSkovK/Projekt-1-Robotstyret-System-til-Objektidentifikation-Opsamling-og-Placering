#ifndef CORNERPOINTS_H
#define CORNERPOINTS_H

#include "opencv2/core.hpp"
#include <vector>

bool cornerPoints(cv::Mat &frame, bool cup,std::vector<int>& points);

#endif