#include "A1_realsense.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <ros/ros.h>
#include <string>
#include <stdio.h>  
#include <thread>
#include <chrono>
// #include <std_msgs/Float64.h>
#include <iostream>
#include <Eigen/Dense>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}


// void camera_pos_reset(){
//     static tf::TransformBroadcaster br_camera;
//     tf::Transform transform_camera;
//     transform_camera.setOrigin(tf::Vector3(0,
//                                     0,
//                                     0.945));
//     tf::Quaternion q_camera;
//     q_camera.setRPY(0,M_PI/2,0);
//     transform_camera.setRotation(q_camera);

//     br_camera.sendTransform(tf::StampedTransform(transform_camera, 
//                     ros::Time::now(), 
//                     "base", 
//                     "camera_link"));
// }

// void pos_update(){
//     static tf::TransformBroadcaster br_quad;
//     tf::Transform transform_quad;
//     transform_quad.setOrigin(tf::Vector3(0,
//                                     0,
//                                     1.0));
//     tf::Quaternion q_quad;
//     q_quad.setRPY(0,0,0);
//     transform_quad.setRotation(q_quad);

//     br_quad.sendTransform(tf::StampedTransform(transform_quad, 
//                     ros::Time::now(), 
//                     "base", 
//                     "imu_link"));
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "a1_realsense_april");
    ros::NodeHandle nh;

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::unique_ptr<A1_realsense> A1 = make_unique<A1_realsense>(nh);

    std::atomic<bool> control_alive{};
    control_alive.store(true, std::memory_order_release);


    std::thread refresh_robot_tag_pos([&](){
    ros::Time start = ros::Time::now();
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();  // bool res = app.exec();
    ros::Duration dt(0);
    ros::Duration dt_solver_time(0);
    // static tf::TransformBroadcaster br;
    // tf::TransformListener listener;
    // visualization_msgs::Marker sphere_marker;
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    
    // sphere_marker.type = visualization_msgs::Marker::CUBE;

    while (control_alive.load(std::memory_order_acquire) && ros::ok()){
        
        A1->target_pos_update();

        now = ros::Time::now();
        dt = now - prev;
        // std::cout<<"dt_1: "<<dt.toSec() <<std::endl;
        prev = now;
        bool update_running = A1->counting_finished;
        if(update_running){
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
        }
        dt_solver_time = ros::Time::now() - now;
        if (dt_solver_time.toSec() < MAIN_UPDATE_FREQUENCY / 1000) {
                ros::Duration( MAIN_UPDATE_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
        }
    }
    });

std::thread robot_control([&](){
    ros::Time start = ros::Time::now();
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();  // bool res = app.exec();
    ros::Duration dt(0);
    ros::Duration dt_solver_time(0);
    while (control_alive.load(std::memory_order_acquire) && ros::ok()){
        
        
        bool Initialized = A1->Initialized_flag;
        if (Initialized){
            A1->start_counting();
            A1->send_position_cmd();
        }
        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        // std::cout<<"dt_2: "<<dt.toSec() <<std::endl;
        bool control_running = A1->counting_finished;
        if(control_running){
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
        }
        dt_solver_time = ros::Time::now() - now;
        if (dt_solver_time.toSec() < GRF_UPDATE_FREQUENCY/1000){
                ros::Duration(GRF_UPDATE_FREQUENCY /1000 - dt_solver_time.toSec()).sleep();
        }
    }
    });

    ros::AsyncSpinner spinner(12);
    spinner.start();
    refresh_robot_tag_pos.join();
    robot_control.join();
    return 0;   
}