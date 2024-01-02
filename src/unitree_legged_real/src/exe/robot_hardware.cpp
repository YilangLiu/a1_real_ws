#include "A1_Interface.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <ros/ros.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"


#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include "convert.h"
#include "A1Params.h"



#ifdef SDK3_1
using namespace aliengo;
#endif

#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_main_loop");
    ros::NodeHandle nh;
    // ros::Rate loop_rate(500);

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    // std::cout<< "Here"<<std::endl;
    std::unique_ptr<A1_Interface> A1 = make_unique<A1_Interface>(nh);
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

            bool control_running = A1->get_stance_leg_forces(dt.toSec());
            // std::cout<<"control running? " << control_running<<std::endl;
            // bool warm_start = A1->warm_start();
            // std::cout<<"warm start?" << warm_start<<std::endl;
            dt_solver_time = ros::Time::now() - now;
            if(!control_running){
                std::cout << "Thread 1 loop is terminated because of errors." << std::endl;
                ros::shutdown();
                std::terminate();
                break;
            }
            if (dt_solver_time.toSec() < GRF_UPDATE_FREQUENCY/1000){
                ros::Duration(GRF_UPDATE_FREQUENCY /1000 - dt_solver_time.toSec()).sleep();
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

            now = ros::Time::now();
            dt = now-prev;
            prev = now;
            ros::Duration elapsed = now - start;
            
            bool main_update_running = A1->main_update(elapsed.toSec(), dt.toSec());
            
            // bool robot_joint = A1->joint_test();
            // bool robot_joint = A1->standing_up();
            // bool warm_start = A1->warm_start();
            // dt_solver_time = ros::Time::now() - now;
            // if (dt_solver_time.toSec() < 2.5/1000){
            //     ros::Duration(2.5/1000 - dt_solver_time.toSec()).sleep();
            // || !send_cmd_running

            bool send_cmd_running = A1->send_torque_cmd();
            // || !send_cmd_running
            if (!main_update_running || !send_cmd_running) {
                std::cout << "Thread 2 loop is terminated because of errors." << std::endl;
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
    ros::AsyncSpinner spinner(12);
    spinner.start();
    stance_leg_control_thread.join();
    main_thread.join();
    return 0;
}