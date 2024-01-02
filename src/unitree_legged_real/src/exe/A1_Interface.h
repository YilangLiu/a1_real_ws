#ifndef A1_INTERFACE
#define A1_INTERFACE

// a1 hardware
#include "A1_dynamics_pino.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include "Robot_State.h"
#include "A1_dynamics.h"
#include "A1_state_estimation.h"
#include "A1_control.h"
#include "A1Params.h"
// std

#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <a1_msgs/Contact.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>


// #define JOY_CMD_BODY_HEIGHT_MAX 0.32     // m
// #define JOY_CMD_BODY_HEIGHT_MIN 0.1     // m
// #define JOY_CMD_BODY_HEIGHT_VEL 0.04    // m/s
// #define JOY_CMD_VELX_MAX 0.6         // m/s
// #define JOY_CMD_VELY_MAX 0.3            // m/s
// #define JOY_CMD_YAW_MAX 0.8             // rad
// #define JOY_CMD_PITCH_MAX 0.4           // rad
// #define JOY_CMD_ROLL_MAX 0.4            // rad
// #define MAIN_UPDATE_FREQUENCY 2.5 // ms
// #define HARDWARE_FEEDBACK_FREQUENCY 2.0  // ms
#define FOOT_FILTER_WINDOW_SIZE 5

class A1_Interface{
public: 
    A1_Interface(ros::NodeHandle &_nh);

    ~A1_Interface(){
        destruct = true;
        thread_.join();
        thread2_.join();
    }

    void lcm_init();
    bool send_position_cmd();
    bool send_torque_cmd();
    bool send_stop();
    bool joint_test();
    bool standing_up();
    bool main_update(double t, double dt);
    double jointLinearInterpolation(double initPos, double targetPos, double rate);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state);
    void joystick_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void marker_callback(const ros::TimerEvent&);
    void receive_motor_state();
    bool get_stance_leg_forces(double dt);
    bool warm_start();

private:
    // define ROS node for publishing
    ros::NodeHandle nh;
    ros::Publisher joint_pub_rviz;
    ros::Publisher joint_angle_pub;
    ros::Publisher imu_pub;
    ros::Publisher contact_pub;
    ros::Publisher pose_pub;
    ros::Publisher rviz_marker_pub;

    ros::Publisher pub_root_lin_vel_desired;
    ros::Publisher pub_root_lin_vel;


    ros::Subscriber joy_msg_sub;
    ros::Subscriber joint_sub;

    ros::Timer marker_timer;

    visualization_msgs::MarkerArray marker_array;

    // Hardware Definition
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::UDP udp;
    unitree_legged_msgs::LowState RecvLowROS;
    unitree_legged_msgs::LowCmd SendLowROS;

    UNITREE_LEGGED_SDK::LowCmd SendLowLCM = {0};
    UNITREE_LEGGED_SDK::LowState RecvLowLCM = {0};
    //UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);


    // foot filter with certain windowsize
    Eigen::Matrix<double, 4, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, 4, 1> foot_force_idx;
    Eigen::Matrix<double, 4, 1> foot_force_filters_sum;

    // msg definition
    sensor_msgs::JointState joint_msg;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::JointState joint_msg_rviz;
    geometry_msgs::PointStamped lin_vel_desired_debug, lin_vel_debug;
    std_msgs::Float64 lin_vel_d_x, lin_vel_d_y, lin_vel_x, lin_vel_y;

    ros::Publisher pub_root_lin_vel_desired_x;
    ros::Publisher pub_root_lin_vel_x;
    ros::Publisher pub_root_lin_vel_desired_y;
    ros::Publisher pub_root_lin_vel_y;

    a1_msgs::Contact contact_msg;

    bool destruct = false;
    bool initiated_flag = false;
    bool standing_finished = false;

    std::thread thread_;
    std::thread thread2_;

    

    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;


    // joystick command
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.10;


    int power_level = 8;

    // joint test
    float qDes[12]={0};
    float qMujoco[12] = {0};
    float qInit[12] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    int count = 0;
    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;


    float Kp[12] = {0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0};

    float Kd[12] = {0.0, 0.0, 0.0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0};
                    
    float standing_pose[12] = {0, 0.9, -1.8, 
                               0, 0.9, -1.8, 
                               0, 0.9, -1.8, 
                               0, 0.9, -1.8};
    float torque[12] = {0};

    // a1 hardware switch foot order
    Eigen::Matrix<int, 12, 1> swap_joint_indices;
    Eigen::Matrix<int, 4, 1> swap_foot_indices;
    Eigen::Matrix<double, 12, 1> joint_pos;
    Eigen::Matrix<double, 12, 1> joint_vel;
    Eigen::Matrix<double, 4, 3> hip_offsets;
    Eigen::Matrix<double, 3, 1> com_offsets;
    

    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    std::vector<Eigen::VectorXd> leg_offset_list;
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};

    A1_dynamics a1_dynamics_state;
    Robot_State robot_state;
    // A1_dynamics_pino a1_dynamics;
    A1_state_estimation a1_estimation;
    robot_control a1_control;
};

#endif 