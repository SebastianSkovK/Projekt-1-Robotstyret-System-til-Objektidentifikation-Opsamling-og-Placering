#ifndef THROWVECTOR_H
#define THROWVECTOR_H

#pragma once
#include <vector>
#include <cmath>
#include <iostream>

double ballHeight (double x, double h, double speed, double theta);

class throwVector
{
public:
    throwVector(double releaseAngle, double g, double heightDiff, double lenghtDiff, double baseAngle);
    ~throwVector();
    void cal3D_Velocity();
    std::vector<double> get3D_Velocity() const;

private:
    double releaseAngle, g, heightDiff, lenghtDiff, baseAngle;
    std::vector<double> velocityXYZ;
};

#endif