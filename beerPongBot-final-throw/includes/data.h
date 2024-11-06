#ifndef DATA_H
#define DATA_H

#include "objectDetection.h"
#include "homograf.h"
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

bool readPoints(cv::Mat &frame, std::vector<int> &points,bool cup,std::vector<std::vector<double>>& container);


#endif