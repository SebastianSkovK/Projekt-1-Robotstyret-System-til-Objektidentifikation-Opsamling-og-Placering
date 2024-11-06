#include "objectDetection.h"
#include "opencv2/opencv.hpp"

bool detectObject(const cv::Mat &frame, std::vector<cv::Vec3f> &center, bool cup, std::vector<int> &pixels)
{
    cv::Mat gray_image;
    cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);
    if (!pixels.empty())
    {
        cv::Rect roi(cv::Point2f(pixels[0], pixels[1]), cv::Point2f(pixels[2], pixels[3]));
        gray_image = gray_image(roi);
    }
    cv::rectangle(frame, cv::Point2f(pixels[0], pixels[1]), cv::Point2f(pixels[2], pixels[3]), cv::Scalar(0, 255, 0), 2);
    std::string msgNoDetect = "No balls detected";
    int minRadius = 10, maxRadius = 20;
    if (cup)
    {
        msgNoDetect = "No cups detected";
        minRadius = 35, maxRadius = 50;
    }
    cv::Mat blurred_image;

    cv::GaussianBlur(gray_image, blurred_image, cv::Size(9, 9), 2, 2);
    cv::HoughCircles(blurred_image, center, cv::HOUGH_GRADIENT, 1, 30, 100, 30, minRadius, maxRadius);
    if (center.empty())
    {
        // tjekker om den fandt et match, hvis den ikke gjorde, returner false
        cv::putText(frame, msgNoDetect, cv::Point2f(15, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(213, 0, 84), 2);
        return false;
    }
    if (cup)
    {
        cv::putText(frame, "Cups detected: " + std::to_string(center.size()), cv::Point2f(15, 45), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(213, 0, 84), 2);
        // cv::rectangle(frame, roi, cv::Scalar(0, 255, 0), 2);
    }
    // hvis den fandt et punkt indenfor bordet, retúrnere true og gem punktet i referencen.
    return true;
}
