#include "readVar.h"
#include <string>
#include "opencv2/core.hpp"

void readVar(bool cup, std::vector<int>& points)
{
    int xMin, yMin, xMax, yMax;
    
    std::string path = "../cupVar.yaml";
    if(!cup)
        path = "../ballVar.yaml";

    cv::FileStorage fr(path, cv::FileStorage::READ);
    fr["xMin"] >> xMin;
    fr["yMin"] >> yMin;
    fr["xMax"] >> xMax;
    fr["yMax"] >> yMax;
    fr.release();
    points = {xMin,yMin, xMax,yMax};
};