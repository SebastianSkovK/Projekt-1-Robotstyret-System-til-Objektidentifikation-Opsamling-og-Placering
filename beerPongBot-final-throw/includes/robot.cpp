#include "robot.h"
#include <iostream>

Robot::Robot (const std::string& ip) : control (ip), receive (ip)
{
    
}

Robot::~Robot()
{
    
}

cv::Matx44d calculateTransformationMatrix (const std::vector<cv::Matx31d>& robotFramePoints, const std::vector<cv::Matx31d>& worldFramePoints)
{
    cv::Matx31d centroidRobotFrame = cv::Matx31d::zeros ();

    for (int i = 0; i < robotFramePoints.size (); i++)
    {
        centroidRobotFrame += robotFramePoints[i];
    }

    centroidRobotFrame *= 1.0 / robotFramePoints.size ();

    std::vector<cv::Matx31d> robotFramePointsCentered;

    for (int i = 0; i < robotFramePoints.size (); i++)
    {
        robotFramePointsCentered.push_back (robotFramePoints[i] - centroidRobotFrame);
    }

    cv::Matx31d centroidWorldFrame = cv::Matx31d::zeros ();

    for (int i = 0; i < worldFramePoints.size (); i++)
    {
        centroidWorldFrame += worldFramePoints[i];
    }

    centroidWorldFrame *= 1.0 / worldFramePoints.size ();

    std::vector<cv::Matx31d> worldFramePointsCentered;

    for (int i = 0; i < worldFramePoints.size (); i++)
    {
        worldFramePointsCentered.push_back (worldFramePoints[i] - centroidWorldFrame);
    }

    cv::Matx33d H = cv::Matx33d::zeros ();

    for (int i = 0; i < robotFramePointsCentered.size (); i++)
    {
        H += robotFramePointsCentered[i] * worldFramePointsCentered[i].t ();
    }

    //std::cout << "H: " << std::endl << H << std::endl;

    cv::Matx33d U;

    cv::Matx31d w;

    cv::Matx33d Vt;
   
    cv::SVD::compute (H, w, U, Vt);

    //std::cout << "U: " << std::endl << U << std::endl;
    //std::cout << "w: " << std::endl << w << std::endl;
    //std::cout << "Vt: " << std::endl << Vt << std::endl;

    cv::Matx33d R = Vt * U.t ();

    cv::Matx31d t = centroidWorldFrame - R * centroidRobotFrame;

    cv::Matx44d T = cv::Matx44d (R (0, 0), R (0, 1), R (0, 2), t (0),
                                 R (1, 0), R (1, 1), R (1, 2), t (1),
                                 R (2, 0), R (2, 1), R (2, 2), t (2),
                                 0, 0, 0, 1);


    //std::cout << "Calibration Matrix: " << std::endl << T << std::endl;

    cv::Matx31d error;

    for (int i = 0; i < robotFramePoints.size (); i++)
    {
        cv::Matx31d calculatedWorldFramePoint = R * robotFramePoints[i] + t;

        //std::cout << "Recorded robot frame point: " << robotFramePoints[i].t () << std::endl;
        //std::cout << "Recorded world frame point: " << worldFramePoints[i].t () << std::endl;
        //std::cout << "Calculated world frame point: " << calculatedWorldFramePoint.t () << std::endl;
        
        error += calculatedWorldFramePoint - worldFramePoints[i];

        //std::cout << std::endl;
    }

    //std::cout << "Total error: " << error.t () << std::endl;
    
    return T;
}

void Robot::calibrate ()
{
    std::vector<cv::Matx31d> robotFramePoints;
    std::vector<cv::Matx31d> worldFramePoints;

    char c;

    while (true)
    {
        std::cout << "Move the arm to desired point and enter 'a' to store point, enter 'b' to end calibration" << std::endl;

        std::cin >> c;

        if (c == 'a')
        {
            std::vector<double> urPoint = receive.getActualTCPPose ();

            cv::Matx31d point = cv::Matx31d (urPoint[0], urPoint[1], urPoint[2]);

            std::cout << "Robot Frame Point stored: " << point << std::endl;

            robotFramePoints.push_back (point);

            std::cout << "Enter the corresponding world frame point:" << std::endl;

            std::cin >> point (0) >> point (1) >> point (2);

            std::cout << "World Frame Point stored: " << point << std::endl;

            worldFramePoints.push_back (point);
        } else if (c == 'b')
        {
            break;
        }
    }

    std::cout << "Saving calibration points to file..." << std::endl;

    std::ofstream file ("../table_calibration_points.txt");

    for (int i = 0; i < robotFramePoints.size (); i++)
    {
        file << robotFramePoints[i](0) << " " << robotFramePoints[i](1) << " " << robotFramePoints[i](2) << std::endl;
        file << worldFramePoints[i](0) << " " << worldFramePoints[i](1) << " " << worldFramePoints[i](2) << std::endl;
    }

    file.close ();

    robotToWorld = calculateTransformationMatrix (robotFramePoints, worldFramePoints);
    worldToRobot = robotToWorld.inv ();
}

void Robot::loadTransformationMatrix ()
{
    std::cout << "Loading calibration matrix from file..." << std::endl;

    std::ifstream file ("table_calibration_matrix.txt");

    cv::Matx44d T;

    robotToWorld = T;
    worldToRobot = robotToWorld.inv ();
}

void Robot::loadPointsAndCalculateMatrix ()
{
    std::cout << "Loading calibration points from file..." << std::endl;

    std::ifstream file ("../table_calibration_points.txt");

    std::vector<cv::Matx31d> robotFramePoints;

    std::vector<cv::Matx31d> worldFramePoints;

    do
    {
        cv::Matx31d robotFramePoint;

        file >> robotFramePoint(0) >> robotFramePoint(1) >> robotFramePoint(2);

        robotFramePoints.push_back (robotFramePoint);

        cv::Matx31d worldFramePoint;

        file >> worldFramePoint(0) >> worldFramePoint(1) >> worldFramePoint(2);

        worldFramePoints.push_back (worldFramePoint);
    } while (!file.eof ());

    file.close ();

    robotToWorld = calculateTransformationMatrix (robotFramePoints, worldFramePoints);
    worldToRobot = robotToWorld.inv ();
}

void Robot::move (cv::Matx31d worldPoint, const cv::Matx31d& rotation)
{
    worldPoint(2) += 0.17;

    cv::Matx41d point4 = cv::Matx41d (worldPoint (0), worldPoint (1), worldPoint (2), 1);

    cv::Matx41d robotPoint = worldToRobot * point4;

    std::vector<double> pose = { robotPoint(0), robotPoint(1), robotPoint(2), rotation(0), rotation(1), rotation(2) };

    control.moveL (pose, 0.5, 1);

    std::vector<double> actualPose = receive.getActualTCPPose ();
    cv::Matx41d actualRobotPoint = cv::Matx41d (actualPose[0], actualPose[1], actualPose[2], 1);

    cv::Matx41d actualWorldPoint = robotToWorld * actualRobotPoint;

    //std::cout << "Actual robot point: " << actualRobotPoint.t () << std::endl;
    //std::cout << "Actual world point: " << actualWorldPoint.t () << std::endl;
    //std::cout << "Rotation: " << actualPose[3] << " " << actualPose[4] << " " << actualPose[5] << std::endl;
}

// void Robot::moveJ (std::vector<double> q_angles)
// {
//     control.moveJ(q_angles);
// }

ur_rtde::RTDEControlInterface& Robot::getControl (){
    return control;
}


ur_rtde::RTDEReceiveInterface& Robot::getReceive (){
    return receive;
}

cv::Matx31d Robot::getTCPWorld(){
    std::vector<double> tcpPos = receive.getActualTCPPose();
    cv::Matx41d matTcp= cv::Matx41d(tcpPos[0], tcpPos[1], tcpPos[2], 1);
    matTcp = robotToWorld * matTcp;

    return cv::Matx31d (matTcp(0), matTcp(1), matTcp(2));
}

const cv::Matx44d& Robot::getRobotToWorld() const{
    return robotToWorld;
}

const cv::Matx44d& Robot::getWorldToRobot() const{
    return worldToRobot;
}