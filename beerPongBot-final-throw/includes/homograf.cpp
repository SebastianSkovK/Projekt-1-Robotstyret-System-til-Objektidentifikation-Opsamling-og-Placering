#include "homograf.h"
#include "opencv2/imgproc.hpp"

cv::Point2f hom(const cv::Point2f &inputPoint,const std::vector<int>& points)
{
    if(points.empty())
        return inputPoint;
    int xM = 16 * 5, yM = 16 * 5;
    // Dannes punkter for homografi til pixelplane og worldframe
    cv::Mat perspectiveContainer;
    // top left square
    std::vector<cv::Point2f> cornerSets{cv::Point2f(points[0],points[1]), cv::Point2f(points[2], points[1]), cv::Point2f(points[2], points[3]), cv::Point2f(points[0], points[3])};
    std::vector<cv::Point2f> worldPoints{cv::Point2f(2.5, 2.5), cv::Point2f(xM-2.5, 2.5), cv::Point2f(xM-2.5, yM-2.5), cv::Point2f(2.5, yM-2.5)};

    perspectiveContainer = cv::getPerspectiveTransform(cornerSets, worldPoints);

    std::vector<cv::Point2f> input;
    input.push_back(inputPoint);
    std::vector<cv::Point2f> output;
    // Transfomere punktet fra pixel til kartetiske koordinater i cm
    cv::perspectiveTransform(input, output, perspectiveContainer);
    return output.front();
}