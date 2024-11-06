#include "undistortion.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

void undistort(const cv::Mat &frame)
{
    // Danner intrinsic og koeff objekter og læser calibrations filen som blevet dannet under kalibration og kan hermed bruges i metoden
    
    cv::Matx33f intrinsicMatrix;
    cv::Vec<float, 5> distortionCoefficients;
    cv::FileStorage fr("../calibration.yaml", cv::FileStorage::READ);
    fr["ins"] >> intrinsicMatrix;
    fr["Dis"] >> distortionCoefficients;
    fr.release();
    cv::Size frameSize(1080, 1080);
    cv::Mat mapX, mapY;
    // Danner map1 og map2 ud fra matricerne og deres værdier
    cv::initUndistortRectifyMap(intrinsicMatrix, distortionCoefficients, cv::Matx33f::eye(), intrinsicMatrix, frameSize, CV_32FC1, mapX, mapY);
    // "flader" frame ud med mapX og mapY og gemmes i et andet object som returneres.
    cv::remap(frame, frame, mapX, mapY, cv::INTER_LINEAR);
};