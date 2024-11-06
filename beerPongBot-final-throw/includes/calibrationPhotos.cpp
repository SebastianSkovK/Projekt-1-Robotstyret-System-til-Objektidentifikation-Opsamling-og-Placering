#include "calibrationPhotos.h"
#include "opencv2/highgui.hpp"
#include <iostream>

bool calibrationPhotos(cv::Mat& stream, int counter)
{
    // Tager billeder og gemmer i mappe og tæller op ved tryk på s
    if (cv::waitKey(10) == 's')
    {
        std::string fileName = "../caliPic/frame" + std::to_string(counter) + ".png";
        cv::imwrite(fileName, stream);
        std::cout << fileName << " has been saved" << std::endl;
        return true;
    }
    // Billedtagning afsluttes ved tryk på q
    else if (cv::waitKey(10) == 'q')
    {
        std::cout << "Frame capturing ended" << std::endl;
        return false;
    }
    return true;
};