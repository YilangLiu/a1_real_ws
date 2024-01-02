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

    // std::cout << "WARNING: Control level is set to LOW-level." << std::endl
    //           << "Make sure the robot is hung up." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    std::unique_ptr<A1_realsense> A1 = make_unique<A1_realsense>(nh);

    std::thread refresh_robot_tag_pos([&](){
    
    // static tf::TransformBroadcaster br;
    // tf::TransformListener listener;
    ros::Rate rate(400);
    // visualization_msgs::Marker sphere_marker;
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    
    // sphere_marker.type = visualization_msgs::Marker::CUBE;

    while (ros::ok()){
        // A1->camera_pos_reset();
        // A1->pos_update();
        // camera_pos_reset();
        // //pos_update();
        // tf::StampedTransform tag2_transform;
        // tf::StampedTransform quad_transform;
        // tf::Transform tag2_trans;
        // tf::Quaternion yaw_quater;

        // try{
        //     listener.lookupTransform("base", "tag_2", ros::Time(0), tag2_transform);
        //     listener.lookupTransform("base", "tag_link", ros::Time(0),quad_transform);
        //     tag2_trans.setOrigin(tf::Vector3(tag2_transform.getOrigin().x(),
        //                                     tag2_transform.getOrigin().y(),
        //                                     tag2_transform.getOrigin().z()));
            

        //     // temporarily fix pitch and roll 
        //     tf::Matrix3x3 m(tag2_transform.getRotation());
        //     double roll, pitch, yaw;
        //     m.getRPY(roll, pitch, yaw);
        //     yaw_quater.setRPY(0,0,yaw);
        //     // tag2_trans.setRotation(tag2_transform.getRotation());
        //     tag2_trans.setRotation(yaw_quater);
            
        //     sphere_marker.header.stamp = ros::Time::now();
        //     sphere_marker.ns = "CUBE";
        //     sphere_marker.action = visualization_msgs::Marker::ADD;
        //     sphere_marker.header.frame_id = "tag_3";
        //     sphere_marker.pose.position.x = 0;
        //     sphere_marker.pose.position.y = 0;
        //     sphere_marker.pose.position.z = 0;
        //     sphere_marker.pose.orientation.x = 0.0;
        //     sphere_marker.pose.orientation.y = 0.0;
        //     sphere_marker.pose.orientation.z = 0.0;
        //     sphere_marker.pose.orientation.w = 1.0;
        //     sphere_marker.color.r = 0.0f;
        //     sphere_marker.color.g = 1.0f;
        //     sphere_marker.color.b = 0.0f;
        //     sphere_marker.color.a = 1.0;
        //     sphere_marker.scale.x = 0.05;
        //     sphere_marker.scale.y = 0.05;
        //     sphere_marker.scale.z = 0.05;
        //     marker_pub.publish(sphere_marker);
        // }
        // catch (tf::TransformException &ex){
        //     ROS_ERROR("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }
        // br.sendTransform(tf::StampedTransform(tag2_trans, ros::Time::now(), "base", "trunk"));
        rate.sleep();
    }
    });


    
    ros::AsyncSpinner spinner(12);
    spinner.start();
    refresh_robot_tag_pos.join();
    return 0;   
}