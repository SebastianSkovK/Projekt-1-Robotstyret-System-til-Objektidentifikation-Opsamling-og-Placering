#include "angle.h"
#include <cmath>

cv::Matx31d getRobotPos()
{
    return cv::Matx31d(0.925,0.375,0);
};

double angle(cv::Matx31d robotVector, cv::Matx31d cup)
{
    robotVector(2) = cup(2) = 0;

    double a = std::acos (robotVector.dot(cup) / (cv::norm(robotVector) * cv::norm(cup)));

    return cup(1) < 0 ? a : -a;
};

cv::Matx31d vectormaker(cv::Matx31d cup)
{
    cv::Matx31d robot = getRobotPos();

    return cup - robot;
};

