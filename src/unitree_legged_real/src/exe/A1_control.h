#ifndef A1_ROBOT_CONTRL
#define A1_ROBOT_CONTRL


#include <iostream>
#include <string>
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include "A1Params.h"
#include "A1_QP_MPC.h"
#include "filter.hpp"
#include "Bezier_curve.h"

class robot_control{
public:
    robot_control();

    void init_control();

    void get_desired_foot_pos(Robot_State &state, double dt);

    void swing_leg_control(Robot_State &state, double dt);

    void calculate_joint_torques(Robot_State &state);

    Eigen::Matrix<double,3, NUM_LEG> stance_leg_control(Robot_State &state, double dt);

    Eigen::Vector3d terrain_estimation(Robot_State &state);
private:
    Bezier_Curve leg_bezier_curve[NUM_LEG];

    Eigen::Matrix<double, 6 , 1> root_acc;

    int mpc_wait;

    std::string use_sin_time;

    MovingWindowFilter terrain_angle_filter;
    MovingWindowFilter recent_contact_x_filter[NUM_LEG];
    MovingWindowFilter recent_contact_y_filter[NUM_LEG];
    MovingWindowFilter recent_contact_z_filter[NUM_LEG];
    
    OsqpEigen::Solver solver;

};

#endif