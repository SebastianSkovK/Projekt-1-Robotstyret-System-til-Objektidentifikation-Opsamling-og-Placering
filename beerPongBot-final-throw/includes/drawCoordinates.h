#ifndef DRAWCOORDINATES_H
#define DRAWCOORDINATES_H

#include "opencv2/core.hpp"
#include <vector>

void drawCoordinate(const cv::Point3f &coordinates, cv::Mat &frame,const std::vector<int>& points);

#endif