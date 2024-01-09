#ifndef A1_REALSENSE
#define A1_REALSENSE

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include "Robot_State.h"
#include "A1_dynamics.h"
#include "A1_state_estimation.h"
#include "A1_control.h"
#include "A1Params.h"

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
#include <math.h>

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
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include "filter.hpp"


#define FOOT_FILTER_WINDOW_SIZE 5

class A1_realsense{
public:
    A1_realsense(ros::NodeHandle &_nh);

    void camera_pos_reset();
    void pos_update();
    void receive_motor_state();
    void lcm_init();
    void send_cmd();
    void start_moving();
    void start_counting();
    bool send_position_cmd();
    void target_pos_update();
    bool counting_finished = false;
    bool Initialized_flag = false;


private:
    // define ROS node for publishing
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher joint_pub_rviz;

    tf::StampedTransform tag2_transform;
    tf::StampedTransform tag6_transform;
    tf::StampedTransform quad_transform;
    tf::StampedTransform foot_sphere_rel;
    tf::Transform tag2_trans;
    tf::Quaternion yaw_quater;
    tf::TransformListener listener;
    visualization_msgs::Marker sphere_marker;

    Bezier_Curve FL_Leg_curve;

    // Hardware Definition
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::UDP udp;
    unitree_legged_msgs::LowState RecvLowROS;
    unitree_legged_msgs::LowCmd SendLowROS;

    UNITREE_LEGGED_SDK::LowCmd SendLowLCM = {0};
    UNITREE_LEGGED_SDK::LowState RecvLowLCM = {0};
    //UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);

    // msg definition
    sensor_msgs::JointState joint_msg;
    sensor_msgs::Imu imu_msg;

    bool destruct = false;
    bool initiated_flag = false;
    bool standing_finished = false;

    std::thread thread_;
    std::thread thread2_;


    // foot filter with certain windowsize
    Eigen::Matrix<double, 4, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, 4, 1> foot_force_idx;
    Eigen::Matrix<double, 4, 1> foot_force_filters_sum;

    int power_level = 8;
    int initial_counting = 0;

    float default_camera_pos[12] = {0,0,0.88};


    // joint test
    
    float qMujoco[12] = {0};
    float qInit[12] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    int count = 0;
    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float control_period = 20000;
    


    float Kp[3];  
    float Kd[3];
                    
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
    Eigen::Matrix<double, 12, 1> qDes;
    Eigen::Vector3d desired_FL_foot_pos;
    Eigen::Vector3d joint_angle_FL;

    Eigen::Vector3d target_pos;
    
    A1_dynamics a1_dynamics_state;
    Robot_State robot_state;
    // A1_dynamics_pino a1_dynamics;

    MovingWindowFilter tag_x;
    MovingWindowFilter tag_y;
    MovingWindowFilter tag_z;
    MovingWindowFilter a1_x;
    MovingWindowFilter a1_y;
    MovingWindowFilter a1_z;
    MovingWindowFilter quat_w;
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;
};





#endif 