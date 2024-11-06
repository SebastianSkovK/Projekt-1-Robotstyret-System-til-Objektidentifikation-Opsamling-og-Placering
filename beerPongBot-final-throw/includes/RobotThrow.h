#ifndef ROBOTTHROW_H
#define ROBOTTHROW_H

#pragma once

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <ur_rtde/rtde_control_interface.h>
#include "CalcVelocity.h"
#include "gripper.h"
#include <thread>
#include <chrono>
#include "jointVelocity.h"
#include "opencv2/opencv.hpp"
#include "throwVector.h"


class RobotThrow
{
public:
// ur_rtde::RTDEControlInterface& robot,
    RobotThrow(ur_rtde::RTDEControlInterface& robot, Gripper gripper, std::vector<double> q_start, std::vector<double> q_end, double l, double h);
    ~RobotThrow();
    void printVector(std::vector<double> q);
    void wait(int seconds);
    double degToRad(int deg);

private:
    std::vector<double> q_start, q_end;
    std::vector<double> deltaPos, positiveOrNeative;
    double acceleration; // rad/s²
    double dt; // 8ms
    std::vector<double> joint_speed, jacobian_d_q;
    ur_rtde::RTDEControlInterface& robot;
    Gripper gripper;
};

#endif