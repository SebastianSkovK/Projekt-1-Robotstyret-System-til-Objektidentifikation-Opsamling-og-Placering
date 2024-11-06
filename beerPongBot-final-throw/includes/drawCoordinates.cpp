#include "drawCoordinates.h"
#include <iostream>
#include "homograf.h"
#include <iomanip>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

void drawCoordinate(const cv::Point3f &coordinates, cv::Mat &frame,const std::vector<int>& points)
{
    if(points.empty())
        return;
    std::ostringstream streamXP, streamYP, streamYM, streamXM, streamR, streamXR;
    // udregn radius for kop eller bold
    float radius = hom(cv::Point2f(coordinates.x+coordinates.z,coordinates.y),points).x - hom(cv::Point2f(coordinates.x,coordinates.y),points).x;
    float xCM = hom(cv::Point2f(coordinates.x,coordinates.y),points).x;
    float yCM = hom(cv::Point2f(coordinates.x,coordinates.y),points).y;
    // udregn radius for kop eller bold
    streamXP << std::fixed << std::setprecision(1) << coordinates.x;
    streamYP << std::fixed << std::setprecision(1) << coordinates.y;
    streamXM << std::fixed << std::setprecision(1) << xCM;
    streamYM << std::fixed << std::setprecision(1) << yCM;
    streamR << std::fixed << std::setprecision(1) << coordinates.z;
    streamXR << std::fixed << std::setprecision(1) << radius;
    std::string pointPixel = "(" + streamXP.str() + "," + streamYP.str() + ")[Pixel]";
    std::string pointMeter = "(" + streamXM.str() + "," + streamYM.str() + ")[CM]";
    std::string radiusCM = "(" + streamR.str() + "," + streamXR.str() + ")[Radius]";
    cv::Point2f textPosM(coordinates.x + 10,coordinates.y - 15);
    cv::Point2f textPosP(coordinates.x + 10,coordinates.y + 15);
    cv::Point2f textPosR(coordinates.x + 10,coordinates.y + 45);
    cv::putText(frame, pointMeter, textPosM, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 229, 180), 2);
    cv::putText(frame, pointPixel, textPosP, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 128), 2);
    cv::putText(frame, radiusCM, textPosR, cv::FONT_HERSHEY_SIMPLEX,1, cv::Scalar(255, 255,0), 2);
    cv::circle(frame, cv::Point2f(coordinates.x,coordinates.y), 1, cv::Scalar(32, 156, 5), 5);
    
};
