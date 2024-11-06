#include "cornerPoints.h"
#include "undistortion.h"
#include "objectDetection.h"
#include "drawCoordinates.h"

bool cornerPoints(cv::Mat &frame, bool cup,std::vector<int>& points)
{
    std::vector<cv::Vec3f> center;
    if (detectObject(frame, center, cup,points))
    {
        for (const auto& c : center)
        {
            drawCoordinate (cv::Point3f(c[0],c[1],c[2]), frame, { 0, 0, 1080, 1080 });
        }

        if (center.size() == 2)
        {
            cv::Point2i topLeft(0,0);
            cv::Point2i bottomRight(0,0);
            std::string path = "../cupVar.yaml";
            if (center[0][0] > center[1][0])
            {
                topLeft.x = center[1][0];
                topLeft.y = center[1][1];
                bottomRight.x = center[0][0];
                bottomRight.y = center[0][1];
            }
            else
            {
                topLeft.x = center[0][0];
                topLeft.y = center[0][1];
                bottomRight.x = center[1][0];
                bottomRight.y = center[1][1];
            }
            if(topLeft.x == 0 || bottomRight.x == 0)
            {
                return false;
            }
            if(!cup)
            {
                path = "../ballVar.yaml";
            }
                points = {topLeft.x,topLeft.y,bottomRight.x,bottomRight.y};

                cv::FileStorage fs(path, cv::FileStorage::WRITE);
                fs << "xMin" << static_cast<int>(topLeft.x);
                fs << "yMin" << static_cast<int>(topLeft.y);
                fs << "xMax" << static_cast<int>(bottomRight.x);
                fs << "yMax" << static_cast<int>(bottomRight.y);
                fs.release();
                return true;
        }
        return false;
    }
    return false;
}