#ifndef A1_CPP_Gazebo
#define A1_CPP_Gazebo


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

#include <cstdio>
#include <cstring>

#include <omp.h>
#include <Eigen/Dense>
#include <EigenRand/EigenRand>

#include <math.h>
#include <chrono>

#include "A1Params.h"
#include "Robot_State.h"
#include "A1_control.h"
#include "A1_state_estimation.h"
#include "A1_dynamics.h"
#include "Utils.h"

#include "filter.hpp"

class A1_Gazebo{
public:
    A1_Gazebo(ros::NodeHandle &_nh);
    
    bool get_stance_leg_forces(double dt);

    bool main_update(double t, double dt);

    bool send_torque_cmd();

    bool mujoco_data_received();

    // callback functions
    void gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom);

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state);

    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);

    void marker_callback(const ros::TimerEvent&);

private:
    ros::NodeHandle nh;

    // 0,  1,  2: FL_hip, FL_thigh, FL_calf
    // 3,  4,  5: FR_hip, FR_thigh, FR_calf
    // 6,  7,  8: RL_hip, RL_thigh, RL_calf
    // 9, 10, 11: RR_hip, RR_thigh, RR_calf
    ros::Publisher pub_joint_cmd[12];
    ros::Subscriber sub_joint_msg[12];
    ros::Publisher pub_euler_d;

    // 0, 1, 2, 3: FL, FR, RL, RR
    ros::Subscriber sub_foot_contact_msg[4];
    // ros::Subscriber sub_gt_pose_msg;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber sub_joy_msg;

    // debug estimation
    ros::Publisher pub_estimated_pose;
    ros::Publisher pub_euler_desired;
    ros::Publisher pub_root_lin_vel_desired;
    ros::Publisher pub_root_lin_vel;

    ros::Publisher pub_root_lin_vel_desired_x;
    ros::Publisher pub_root_lin_vel_x;
    ros::Publisher pub_root_lin_vel_desired_y;
    ros::Publisher pub_root_lin_vel_y;
    ros::Publisher pub_root_yaw_desired;
    ros::Publisher pub_root_yaw;


    ros::Publisher pub_root_ang_vel_desired;
    ros::Publisher pub_root_pos_desired;
    ros::Publisher pub_desired_height_joy;
    ros::Publisher pub_mpc_height;
    ros::Publisher rviz_marker_pub;
    ros::Publisher joint_pub_rviz;
    ros::Publisher foot_plan_discrete_pub;

    ros::Timer marker_timer;
    visualization_msgs::MarkerArray marker_array;
    sensor_msgs::JointState joint_msg;


    // joystick command
    double joy_cmd_velx = 0.0;
    double joy_cmd_velx_forward = 0.0;
    double joy_cmd_velx_backward = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;

    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;

    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.30;

    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    A1_dynamics a1_dynamics_state;
    Robot_State robot_state;
    // A1_dynamics_pino a1_dynamics;
    A1_state_estimation a1_estimation;
    robot_control a1_control;

    MovingWindowFilter acc_x;
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
    MovingWindowFilter quat_w;
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;
};

#endif