#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include "opencv2/core.hpp"
#include <vector>

bool detectObject(const cv::Mat &frame, std::vector<cv::Vec3f> &center, bool cup, std::vector<int> &pixels);

#endif