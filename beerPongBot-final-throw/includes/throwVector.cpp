#include "throwVector.h"

double ballHeight (double x, double h, double speed, double theta)
{
    return h - 9.82 / 2 * (x / (speed * cos(theta))) * tan(theta) * x;
}

throwVector::throwVector(double releaseAngle, double g, double heightDiff, double lenghtDiff, double baseAngle)
: releaseAngle(releaseAngle), g(g), heightDiff(heightDiff), lenghtDiff(lenghtDiff), baseAngle(baseAngle)
{
    velocityXYZ = {0,0,0};
}

throwVector::~throwVector()
{

}

void throwVector::cal3D_Velocity()
{
    double twoD_velocity = 0.0;
    twoD_velocity = ( pow(2, 0.5) * lenghtDiff * pow( ( g / (heightDiff + lenghtDiff*tan( (M_PI * releaseAngle) / 180.0) ) ) , 0.5) / (2*cos( (M_PI*releaseAngle)/180 ) ) );
    std::cout << "twoD_velocity = " << twoD_velocity << std::endl;

    velocityXYZ[0] = twoD_velocity * sin(releaseAngle) * cos(baseAngle); // X
    velocityXYZ[1] = twoD_velocity * sin(releaseAngle) * sin(baseAngle); // Y
    velocityXYZ[2] = twoD_velocity * cos(releaseAngle); // Z

}


std::vector<double> throwVector::get3D_Velocity() const
{
    return velocityXYZ;
}