#include "data.h"

bool readPoints(cv::Mat &frame, std::vector<int> &points,bool cup,std::vector<std::vector<double> > &container)
{
    std::string type = "ball";
    if(cup)
    {
        type = "cup";
    }
    std::vector<cv::Vec3f> dataPoints;
    if(detectObject(frame,dataPoints,cup,points))
    {
        if(dataPoints.size() == 3)
        {
            for(int i = 0; i < dataPoints.size(); i++)
            {
                std::vector<double> temp;
                temp.push_back(hom(cv::Point2f(points[0] + dataPoints[i][0], points[1] + dataPoints[i][1]), points).x);
                std::cout << "Points saved: " << temp.front(); 
                temp.push_back(hom(cv::Point2f(points[0] + dataPoints[i][0], points[1] + dataPoints[i][1]), points).y);
                std::cout << " " <<temp.front() << std::endl;
                container.push_back(temp);
            }
            
            return true;
            
        }
        else
        {
            std::cout << "couldn't find 3 points" << std::endl;
            return false;
        }
    }
    else
        return false;
}