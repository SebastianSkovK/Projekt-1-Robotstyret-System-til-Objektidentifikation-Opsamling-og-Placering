#include "CalcVelocity.h"

CalcVelocity::CalcVelocity(std::vector<double> q_start, std::vector<double> q_end, std::vector<double> jacobian_d_q, double acceleration, double dt)
: q_start(q_start), q_end(q_end), jacobian_d_q(jacobian_d_q), acceleration(acceleration), dt(dt)
{
    maxJointTime = 0;
    deltaPos = {};
    positiveOrNeative = {};
    for (int i = 0; i < q_start.size(); ++i) {
        deltaPos.emplace_back(q_end[i] - q_start[i]);
        if (deltaPos[i] >= 0 ){
            positiveOrNeative.emplace_back(1.0);
        } else {
            positiveOrNeative.emplace_back(-1.0);
        }
    }
}

CalcVelocity::~CalcVelocity()
{

}

std::vector<double> CalcVelocity::jointVelocity(int joint)
{
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // calculate path for trow (udgangspunkt led 1)
    // -------------------------
    double time_a = fabs( jacobian_d_q[joint] /  acceleration );
    std::cout << "time_a = " << time_a << std::endl;
    std::vector<double> robot_q_pos = q_start;
    std::cout << "pos neg : " << positiveOrNeative[joint] << std::endl;
    std::vector<double> result_v = {};
    std::vector<double> result_p = {};
    for (double t = 0.0; t < 2.0; t += dt) {
        double v = 0;
        double p = 0;
        if ( t < time_a) {
            v = positiveOrNeative[joint] * acceleration * t;
            p = positiveOrNeative[joint] * 0.5 * acceleration * pow(t, 2);
            p += q_start[joint];
        } else {
            v = jacobian_d_q[joint]; // v_max
            p = positiveOrNeative[joint] * ( 0.5 * acceleration * pow(t, 2) + pow(jacobian_d_q[joint], 2) * (t-time_a) );
            p += q_start[joint];
        }
        
        result_v.emplace_back(v);
        result_p.emplace_back(p);
        std::cout << "second: " << std::setw(6) << t << "  ,  v = " << std::setw(10) <<  v << "  ,  p =" << std::setw(10) << p*180/M_PI << " , goal = " << q_end[joint]*180/M_PI<< std::endl;
        // negativ sammenling
        if ( p <=  q_end[joint] && positiveOrNeative[joint] == -1){
            timesOfEachJoint[joint] = t;
            std::cout << "add to start pos of joint " << joint << " : " << p - q_end[joint] << std::endl;
            addToJointsPosStart[joint] = p - q_end[joint];
            break;
        } else if( p >=  q_end[joint] && positiveOrNeative[joint] == 1 ){
            timesOfEachJoint[joint] = t;
            std::cout << "add to start pos of joint " << joint << " : " << q_end[joint] - p << std::endl;
            addToJointsPosStart[joint] = q_end[joint] - p;
            break;
        }
    }

    std::cout << "time of movement in joint 1: " << timesOfEachJoint[joint] << std::endl << std::endl << std::endl;

    return result_v;
}

std::vector<double> CalcVelocity::getTimesOfEachJoint() const{
    return timesOfEachJoint;
}

void CalcVelocity::calMaxJointTime(){
    maxJointTime = 0;
    for (int i = 0; i < timesOfEachJoint.size(); ++i){
        if( timesOfEachJoint[i] > maxJointTime){
            maxJointTime = timesOfEachJoint[i];
        }
    }
}

double CalcVelocity::getMaxJointTime() const{
    return maxJointTime;
}

std::vector<double> CalcVelocity::getAddToJointsPosStart() const{
    return addToJointsPosStart;
}