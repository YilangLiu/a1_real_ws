#include "A1_Gazebo.h"
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>

// control parameters
#include "A1Params.h"

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gazebo_a1_qp_ctrl");
    ros::NodeHandle nh;
    // ros::Rate loop_rate(500);

    std::string use_sim_time;
    if (ros::param::get("/use_sim_time", use_sim_time)){
        if (use_sim_time != "false"){
            std::cout<<"A1 hardware is not in real time mode"<<std::endl;
        }
    }
    std::unique_ptr<A1_Gazebo> A1 = make_unique<A1_Gazebo>(nh);
    // std::cout<< "Here"<<std::endl;

    std::atomic<bool> control_alive{};
    control_alive.store(true, std::memory_order_release);
    std::thread stance_leg_control_thread([&](){
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);
        ros::Duration dt_solver_time(0);

        while (control_alive.load(std::memory_order_acquire) && ros::ok()){
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            bool data_recv = A1->mujoco_data_received();
            // ros::Duration(GRF_UPDATE_FREQUENCY / 1000).sleep();
            // std::cout<<"data recv" << data_recv <<std::endl;
            if (data_recv){
                auto t1 = std::chrono::high_resolution_clock::now();
                bool running = A1->get_stance_leg_forces(dt.toSec());
                auto t2 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> ms_double = t2 - t1;
                // std::cout << "MPC solution is updated in " << ms_double.count() << "ms" << std::endl;
                dt_solver_time = ros::Time::now() - now;
                 if (dt_solver_time.toSec() < GRF_UPDATE_FREQUENCY/1000){
                ros::Duration(GRF_UPDATE_FREQUENCY /1000 - dt_solver_time.toSec()).sleep();
                }
                if (!running) {
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
                 }
            }
        }
    });

    std::thread main_thread([&](){
        //Initialize the variables for control
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();
        ros::Duration dt(0);
        ros::Duration dt_solver_time(0);

        while(control_alive.load(std::memory_order_acquire) && ros::ok()){
            auto t3 = std::chrono::high_resolution_clock::now();
            // ros::Duration(MAIN_UPDATE_FREQUENCY / 1000).sleep();

            now = ros::Time::now();
            dt = now-prev;
            prev = now;
            ros::Duration elapsed = now - start;
            bool data_recv = A1->mujoco_data_received();
            // std::cout<< "data_recv: "<<data_recv <<std::endl;
            if (data_recv){
                bool main_update_running = A1->main_update(elapsed.toSec(), dt.toSec());
                // bool robot_joint = A1->joint_test();
                // bool robot_joint = A1->standing_up();
                bool send_cmd_running = A1->send_torque_cmd();
                // dt_solver_time = ros::Time::now() - now;
                // if (dt_solver_time.toSec() < 2.5/1000){
                //     ros::Duration(2.5/1000 - dt_solver_time.toSec()).sleep();
                // || !send_cmd_running
                auto t4 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> ms_double = t4 - t3;
                dt_solver_time = ros::Time::now() - now;
                if (dt_solver_time.toSec() < MAIN_UPDATE_FREQUENCY / 1000) {
                    ros::Duration( MAIN_UPDATE_FREQUENCY / 1000 - dt_solver_time.toSec() ).sleep();
                }
                if (!main_update_running || !send_cmd_running) {
                    std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
                    ros::shutdown();
                    std::terminate();
                    break;
                }
            }
        }
    });
    ros::AsyncSpinner spinner(12);
    spinner.start();
    stance_leg_control_thread.join();
    main_thread.join();
    return 0;
}