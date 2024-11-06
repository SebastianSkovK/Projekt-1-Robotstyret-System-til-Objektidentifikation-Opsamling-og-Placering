#ifndef JOINTVELOCITY_H
#define JOINTVELOCITY_H

#include "opencv2/opencv.hpp"
#include <vector>

cv::Matx61d getJointVelocities (cv::Matx61d q, cv::Matx61d endEffectorVelocity);
cv::Matx31d getJointVelocitiesSEW (cv::Matx61d q, cv::Matx61d endEffectorVelocity);
cv::Matx31d getTcpPosToGivenQ (cv::Matx61d q);
cv::Matx61d vecToMat (std::vector<double> vec);
std::vector<double> matToVec (cv::Matx61d mat);
cv::Matx41d d3Tod4 (cv::Matx31d);
cv::Matx31d d4Tod3 (cv::Matx41d);

#endif