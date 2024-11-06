#ifndef CALCVELOCITY_H
#define CALCVELOCITY_H

#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

class CalcVelocity
{
public:
    CalcVelocity(std::vector<double> q_start, std::vector<double> q_end, std::vector<double> jacobian_d_q, double acceleration, double dt);
    ~CalcVelocity();

    std::vector<double> jointVelocity(int joint);
    std::vector<double> getTimesOfEachJoint() const;
    void calMaxJointTime();
    double getMaxJointTime() const;
    std::vector<double> getAddToJointsPosStart() const;

private:
    std::vector<double> q_start, q_end, jacobian_d_q;
    double acceleration, dt, maxJointTime;
    std::vector<double> deltaPos, positiveOrNeative;
    std::vector<double> timesOfEachJoint = {0,0,0,0,0,0};
    std::vector<double> addToJointsPosStart = {0,0,0,0,0,0};
};

#endif