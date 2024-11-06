#include "RobotThrow.h"
#include "opencv2/core.hpp"

void RobotThrow::printVector(std::vector<double> q){
    for(int i = 0; i < q.size(); ++i){
        std::cout << std::setw(12) << q[i] << " ";
    }
    std::cout << std::endl;
}

void RobotThrow::wait(int seconds){
    // Wait for 3 seconds
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    //std::cout << "Resuming after waiting." << std::endl;
}

double RobotThrow::degToRad(int deg){
    return deg * (M_PI / 180);
}

RobotThrow::RobotThrow(ur_rtde::RTDEControlInterface& robot, Gripper gripper, std::vector<double> q_start, std::vector<double> q_end, double l, double h)
: robot(robot), gripper(gripper), q_start(q_start), q_end(q_end)
{
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
    printVector(deltaPos);
    printVector(positiveOrNeative);

    

    // Parameters
    acceleration = 20.0; // 9.0; // rad/s²
    dt = 1.0/125; // 8ms
    joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // jacobian_d_q = {0.0, -2.1779, 3.2594, -1.0815, 0.0, 0.0};

    // calculate velocity 3d vector
    // throwVector calVel(45.0, 9.82, 0.5, 0.3, degToRad(-67.0));
    throwVector calVel(45.0, 9.82, h, l, q_start[0]);
    calVel.cal3D_Velocity();
    std::vector<double> Vel3D = calVel.get3D_Velocity();

    // dynamisk calculate Jacobian 
    cv::Matx61d mat_qend = cv::Matx61d(q_end[0], q_end[1], q_end[2], q_end[3], q_end[4], q_end[5]);

    cv::Matx61d mat_throwSpeed = cv::Matx61d(Vel3D[0], Vel3D[1], Vel3D[2], 0.0, 0.0, 0.0);

    cv::Matx61d jointVelQ = getJointVelocities(mat_qend, mat_throwSpeed);
    // std::cout << jointVelQ << std::endl;

    std::vector<double> jacobian_d_q = {0, jointVelQ(1), jointVelQ(2), jointVelQ(3), 0, 0};
    for(int i = 0; i < jacobian_d_q.size(); ++i){
        std::cout << "q" << i+1 << ":  " << jacobian_d_q[i] << std::endl;
    }
    std::cout << std::endl;


    // --------------------------------------------
    // CalcVelocity(std::vector<double> q_start, std::vector<double> q_end, std::vector<double> jacobian_d_q, double acceleration, double dt);
    CalcVelocity FindVel(q_start, q_end, jacobian_d_q, acceleration, dt);
    std::vector<double> vel_q_1 = FindVel.jointVelocity(1);
    std::vector<double> vel_q_2 = FindVel.jointVelocity(2);
    std::vector<double> vel_q_3 = FindVel.jointVelocity(3);

    std::vector<double> jointTimes = FindVel.getTimesOfEachJoint();
    std::cout << "JointTimes: ";
    printVector(jointTimes);
    std::cout << std::endl;
    FindVel.calMaxJointTime();
    std::vector<double> addToJointStartPos = FindVel.getAddToJointsPosStart();
    std::cout << "addToJointStartPos: ";
    printVector(addToJointStartPos);
    std::cout << std::endl;

    // update joints start position
    for (int i = 0; i < 6; ++i){
        q_start[i] += addToJointStartPos[i];
    }
    printVector(q_start);

    return;

    // --------------------------------------------

    // // Move to initial joint position with a regular moveJ
    robot.moveJ(q_start);    

    // Execute 125Hz control loop for 2 seconds, each cycle is ~8ms
    double seconds = FindVel.getMaxJointTime(); // seconds
    std::cout << "max joint time= " << seconds << std::endl;
    double skalar = 1;
    int countJoint1 = 0;
    int countJoint2 = 0;
    int countJoint3 = 0;

    for (unsigned int i=0; i<seconds*125 - 1; i++)
    {
        std::cout << "time = " << i << std::endl;
        if((seconds - jointTimes[1])*125 <= i + 1){
            ++countJoint1;
        }
        if((seconds - jointTimes[2])*125 <= i + 1){
            ++countJoint2;
        }
        if((seconds - jointTimes[3])*125 <= i + 1){
            ++countJoint3;
        }

        std::chrono::steady_clock::time_point t_start = robot.initPeriod();
        robot.speedJ(joint_speed, acceleration, dt);
        joint_speed[1] = vel_q_1[countJoint1];
        joint_speed[2] = vel_q_2[countJoint2];
        joint_speed[3] = vel_q_3[countJoint3];
        robot.waitPeriod(t_start);

        // open gripper to throw ball
        if (i > (seconds*125)-2){
            gripper.open();
        }
    }
    for (unsigned int i=0; i<2; i++){
        std::chrono::steady_clock::time_point t_start = robot.initPeriod();
        robot.speedJ(joint_speed, acceleration, dt);
        joint_speed[1] = vel_q_1[countJoint1];
        joint_speed[2] = vel_q_2[countJoint2];
        joint_speed[3] = vel_q_3[countJoint3];
        robot.waitPeriod(t_start);
    }

    robot.speedStop();
    robot.stopScript();
}

RobotThrow::~RobotThrow()
{

}

