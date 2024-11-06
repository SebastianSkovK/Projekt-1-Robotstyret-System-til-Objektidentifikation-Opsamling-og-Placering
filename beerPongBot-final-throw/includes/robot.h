#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <vector>
#include "opencv2/core.hpp"
#include "ur_rtde/rtde.h"
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"

class Robot
{
    private:
        ur_rtde::RTDEControlInterface control;
        ur_rtde::RTDEReceiveInterface receive;
        cv::Matx44d robotToWorld;
        cv::Matx44d worldToRobot;

    public:
        Robot (const std::string& ipcalibration);
        ~Robot();
        void calibrate ();
        void loadTransformationMatrix ();
        void loadPointsAndCalculateMatrix ();
        void move (cv::Matx31d point, const cv::Matx31d& rotation);
        // void moveJ (std::vector<double> q_angles);
        ur_rtde::RTDEControlInterface& getControl ();
        ur_rtde::RTDEReceiveInterface& getReceive ();
        cv::Matx31d getTCPWorld();
        const cv::Matx44d& getRobotToWorld() const;
        const cv::Matx44d& getWorldToRobot() const;
};

#endif