#ifndef HOMOGRAF_H
#define HOMOGRAF_H

#include "opencv2/core.hpp"
#include <vector>

cv::Point2f hom(const cv::Point2f &inputPoint,const std::vector<int>& points);

#endif