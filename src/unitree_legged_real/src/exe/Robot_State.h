#ifndef A1_ROBOT_CONTRL_STATE
#define A1_ROBOT_CONTRL_STATE

#include <Eigen/Dense>
#include <ros/ros.h>
#include "A1Params.h"
#include <a1_msgs/FootState.h>
#include <a1_msgs/MultiFootPlanDiscrete.h>


class Robot_State{
public:
    Robot_State(){
        reset();
    }

    void reset(){
        motion_mode = 0;
        counter = 0;
        joint_pos.setZero();
        joint_vel.setZero();
        j_foot.setIdentity();


        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();
        root_lin_vel_desired.setZero();

        root_ang_vel_desired.setZero();
        root_pos_desired.setZero();
        root_euler_desired.setZero();

        root_rot_matrix.setZero();
        root_rot_mat_z.setZero();

        foot_force.setZero();

        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();

        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        foot_vel_world.setZero();

        foot_pos_desired_rel.setZero();
        foot_pos_desired_rel_next.setZero();
        foot_pos_desired_abs.setZero();
        foot_pos_desired_world.setZero();

        double a1_default_foot_pos_FL_x;
        double a1_default_foot_pos_FL_y;
        double a1_default_foot_pos_FL_z;

        double a1_default_foot_pos_FR_x;
        double a1_default_foot_pos_FR_y;
        double a1_default_foot_pos_FR_z;

        double a1_default_foot_pos_RL_x;
        double a1_default_foot_pos_RL_y;
        double a1_default_foot_pos_RL_z;

        double a1_default_foot_pos_RR_x;
        double a1_default_foot_pos_RR_y;
        double a1_default_foot_pos_RR_z;

        a1_default_foot_pos_FL_x= 0.17;
        a1_default_foot_pos_FL_y= 0.14;
        a1_default_foot_pos_FL_z= -0.35;

        a1_default_foot_pos_FR_x= 0.17;
        a1_default_foot_pos_FR_y= -0.14;
        a1_default_foot_pos_FR_z= -0.35;

        a1_default_foot_pos_RL_x= -0.17;
        a1_default_foot_pos_RL_y= 0.14;
        a1_default_foot_pos_RL_z= -0.35;

        a1_default_foot_pos_RR_x= -0.17;
        a1_default_foot_pos_RR_y= -0.14;
        a1_default_foot_pos_RR_z= -0.35;

        foot_pos_default<<a1_default_foot_pos_FL_x, a1_default_foot_pos_FR_x, a1_default_foot_pos_RL_x, a1_default_foot_pos_RR_x,
                a1_default_foot_pos_FL_y, a1_default_foot_pos_FR_y, a1_default_foot_pos_RL_y, a1_default_foot_pos_RR_y,
                a1_default_foot_pos_FL_z, a1_default_foot_pos_FR_z, a1_default_foot_pos_RL_z, a1_default_foot_pos_RR_z;
        
        foot_pos_start.setZero();
        prev_foot_pos_rel.setZero();
        prev_foot_pos_target.setZero();

        imu_acc.setZero();
        imu_ang_vel.setZero();
        power_level = 5;

        robot_mass = 12;
        kp_linear_lock_x = 120;
        kp_linear_lock_y = 120;

        for(int i=0; i< NUM_LEG; ++i){
            contacts[i] = false;
            expected_contacts[i]=false;
            early_contacts[i] = false;
        }

        gait_count_speed << 1.5 ,1.5 ,1.5 ,1.5;
        gait_count.setZero();
        gait_count_cycle = 120*2;
        swing_leg_cycle = 120;
        joint_torques.setZero();
        foot_pos_recent_contact.setZero();

        kp_swing_foot << 200,200,200,200,
                         200,200,200,200,
                         150, 150, 150, 150; 

        double kd_swing_x, kd_swing_y, kd_swing_z;
        kd_swing_x = 10.0;
        kd_swing_y = 10.0;
        kd_swing_z = 5.0; 
        kd_swing_foot << kd_swing_x, kd_swing_x, kd_swing_x, kd_swing_x,
                         kd_swing_y, kd_swing_y, kd_swing_y, kd_swing_y,
                         kd_swing_z, kd_swing_z, kd_swing_z, kd_swing_z;
        // double kp_swing_x, kp_swing_y, kp_swing_z;
        // kp_swing_x = 120;
        // kp_swing_y = 120;
        // kp_swing_z = 80;

        // kp_swing_foot << kp_swing_x,kp_swing_x,kp_swing_x,kp_swing_x,
        //                  kp_swing_y,kp_swing_y,kp_swing_y,kp_swing_y,
        //                  kp_swing_z, kp_swing_z, kp_swing_z, kp_swing_z;

        // double kd_swing_x, kd_swing_y, kd_swing_z;
        // kd_swing_x = 6.0;
        // kd_swing_y = 6.0;
        // kd_swing_z = 5.0;
        // kd_swing_foot << kd_swing_x, kd_swing_x, kd_swing_x, kd_swing_x,
        //                  kd_swing_y, kd_swing_y, kd_swing_y, kd_swing_y,
        //                  kd_swing_z, kd_swing_z, kd_swing_z, kd_swing_z;

        Q_weights.resize(13);
        R_weights.resize(12);

        double q_weights_0, q_weights_1, q_weights_2, q_weights_3, 
                q_weights_4, q_weights_5, q_weights_6, 
                q_weights_7, q_weights_8, q_weights_9, 
                q_weights_10, q_weights_11, q_weights_12;

        q_weights_0= 20.0; // roll
        q_weights_1= 10.0;// pitch
        q_weights_2= 1.0;  // yaw

        q_weights_3= 0.0;   // pos x
        q_weights_4= 0.0;   // pos y
        q_weights_5= 420.0; // pos z

        q_weights_6= 0.05; // omega x
        q_weights_7= 0.05; // omega y
        q_weights_8= 0.05; // omega z

        q_weights_9= 30.0;  // vel x
        q_weights_10= 30.0; // vel y
        q_weights_11= 10.0; // vel z

        q_weights_12= 0.0;


        Q_weights << q_weights_0, q_weights_1, q_weights_2,
                q_weights_3, q_weights_4, q_weights_5,
                q_weights_6, q_weights_7, q_weights_8,
                q_weights_9, q_weights_10, q_weights_11,
                q_weights_12;

        double r_weights_0, r_weights_1, r_weights_2, r_weights_3, 
                r_weights_4, r_weights_5, r_weights_6, r_weights_7, 
                r_weights_8, r_weights_9, r_weights_10, r_weights_11;

        r_weights_0= 1e-7;
        r_weights_1= 1e-7;
        r_weights_2= 1e-7;

        r_weights_3= 1e-7;
        r_weights_4= 1e-7;
        r_weights_5= 1e-7;

        r_weights_6= 1e-7;
        r_weights_7= 1e-7;
        r_weights_8= 1e-7;

        r_weights_9= 1e-7;
        r_weights_10= 1e-7;
        r_weights_11= 1e-7;


        R_weights << r_weights_0, r_weights_1, r_weights_2,
                r_weights_3, r_weights_4, r_weights_5,
                r_weights_6, r_weights_7, r_weights_8,
                r_weights_9, r_weights_10, r_weights_11;

        km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

        gravity_torque << 0.80, 0, 0, -0.80, 0, 0, 0.80, 0, 0, -0.80, 0, 0;

        double a1_trunk_inertia_xx;
        double a1_trunk_inertia_xy;
        double a1_trunk_inertia_xz;
        double a1_trunk_inertia_yz;
        double a1_trunk_inertia_yy;
        double a1_trunk_inertia_zz;
        a1_trunk_inertia_xx = 0.0158533;
        a1_trunk_inertia_xy = 0.0;
        a1_trunk_inertia_xz = 0.0;
        a1_trunk_inertia_yz = 0.0;
        a1_trunk_inertia_yy = 0.0377999;
        a1_trunk_inertia_zz = 0.0456542;

        a1_trunk_inertia <<a1_trunk_inertia_xx, a1_trunk_inertia_xy, a1_trunk_inertia_xz,
                a1_trunk_inertia_xy, a1_trunk_inertia_yy, a1_trunk_inertia_yz,
                a1_trunk_inertia_xz, a1_trunk_inertia_yz, a1_trunk_inertia_zz;

        FL_color_ = {166,25,46};
        RL_color_ = {0,45,114};
        FR_color_ = {0,132,61};
        RR_color_ = {242,169,0};
        
        foot_plans.header.frame_id = "trunk";
        foot_plans.feet.resize(NUM_LEG);
        for (int i = 0; i<NUM_LEG;++i){foot_plans.feet[i].footholds.resize(0);}

        reset_gait_count();
    }
    void reset_hardware(){
        motion_mode = 0;
        counter = 0;
        joint_pos.setZero();
        joint_vel.setZero();
        j_foot.setIdentity();


        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();
        root_lin_vel_desired.setZero();

        root_ang_vel_desired.setZero();
        root_pos_desired.setZero();
        root_euler_desired.setZero();

        root_rot_matrix.setZero();
        root_rot_mat_z.setZero();

        foot_force.setZero();

        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();

        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        foot_vel_world.setZero();

        foot_pos_desired_rel.setZero();
        foot_pos_desired_rel_next.setZero();
        foot_pos_desired_abs.setZero();
        foot_pos_desired_world.setZero();

        double a1_default_foot_pos_FL_x;
        double a1_default_foot_pos_FL_y;
        double a1_default_foot_pos_FL_z;

        double a1_default_foot_pos_FR_x;
        double a1_default_foot_pos_FR_y;
        double a1_default_foot_pos_FR_z;

        double a1_default_foot_pos_RL_x;
        double a1_default_foot_pos_RL_y;
        double a1_default_foot_pos_RL_z;

        double a1_default_foot_pos_RR_x;
        double a1_default_foot_pos_RR_y;
        double a1_default_foot_pos_RR_z;

        a1_default_foot_pos_FL_x= 0.17;
        a1_default_foot_pos_FL_y= 0.15;
        a1_default_foot_pos_FL_z= -0.3;
        
        a1_default_foot_pos_FR_x= 0.17;
        a1_default_foot_pos_FR_y= -0.15;
        a1_default_foot_pos_FR_z= -0.3;
        
        a1_default_foot_pos_RL_x= -0.17;
        a1_default_foot_pos_RL_y= 0.15;
        a1_default_foot_pos_RL_z= -0.3;

        a1_default_foot_pos_RR_x= -0.17;
        a1_default_foot_pos_RR_y= -0.15;
        a1_default_foot_pos_RR_z= -0.3;

        foot_pos_default<<a1_default_foot_pos_FL_x, a1_default_foot_pos_FR_x, a1_default_foot_pos_RL_x, a1_default_foot_pos_RR_x,
                a1_default_foot_pos_FL_y, a1_default_foot_pos_FR_y, a1_default_foot_pos_RL_y, a1_default_foot_pos_RR_y,
                a1_default_foot_pos_FL_z, a1_default_foot_pos_FR_z, a1_default_foot_pos_RL_z, a1_default_foot_pos_RR_z;
        
        foot_pos_start.setZero();
        prev_foot_pos_rel.setZero();
        prev_foot_pos_target.setZero();

        imu_acc.setZero();
        imu_ang_vel.setZero();
        power_level = 5;

        robot_mass = 13.0;
        kp_linear_lock_x = 120;
        kp_linear_lock_y = 120;

        for(int i=0; i< NUM_LEG; ++i){
            contacts[i] = false;
            expected_contacts[i]=false;
            early_contacts[i] = false;
        }

        gait_count_speed << 1.4, 1.4, 1.4, 1.4;
        gait_count.setZero();
        gait_count_cycle = 120*2;
        swing_leg_cycle = 120;
        joint_torques.setZero();
        foot_pos_recent_contact.setZero();

        double kp_swing_x, kp_swing_y, kp_swing_z;
        kp_swing_x = 120; //120
        kp_swing_y = 120; //120
        kp_swing_z = 80;

        kp_swing_foot << kp_swing_x,kp_swing_x,kp_swing_x,kp_swing_x,
                         kp_swing_y,kp_swing_y,kp_swing_y,kp_swing_y,
                         kp_swing_z, kp_swing_z, kp_swing_z, kp_swing_z;

        double kd_swing_x, kd_swing_y, kd_swing_z;
        kd_swing_x = 6.5; // 7.5
        kd_swing_y = 6.5; // 7.5
        kd_swing_z = 8.0; // 8.0
        kd_swing_foot << kd_swing_x, kd_swing_x, kd_swing_x, kd_swing_x,
                         kd_swing_y, kd_swing_y, kd_swing_y, kd_swing_y,
                         kd_swing_z, kd_swing_z, kd_swing_z, kd_swing_z;

        Q_weights.resize(13);
        R_weights.resize(12);

        double q_weights_0, q_weights_1, q_weights_2, q_weights_3, 
                q_weights_4, q_weights_5, q_weights_6, 
                q_weights_7, q_weights_8, q_weights_9, 
                q_weights_10, q_weights_11, q_weights_12;

        q_weights_0= 150.0; // roll
        q_weights_1= 150.0;// pitch
        q_weights_2= 50.0;  // yaw

        q_weights_3= 0.0;   // pos x
        q_weights_4= 0.0;   // pos y
        q_weights_5= 150.0; // pos z

        q_weights_6= 0.2; // omega x
        q_weights_7= 0.2; // omega y
        q_weights_8= 0.2; // omega z

        q_weights_9= 5;  // vel x
        q_weights_10= 5; // vel y
        q_weights_11= 0.1; // vel z

        q_weights_12= 0.0;


        Q_weights << q_weights_0, q_weights_1, q_weights_2,
                q_weights_3, q_weights_4, q_weights_5,
                q_weights_6, q_weights_7, q_weights_8,
                q_weights_9, q_weights_10, q_weights_11,
                q_weights_12;

        double r_weights_0, r_weights_1, r_weights_2, r_weights_3, 
                r_weights_4, r_weights_5, r_weights_6, r_weights_7, 
                r_weights_8, r_weights_9, r_weights_10, r_weights_11;

        r_weights_0= 1e-3; //1e-3;
        r_weights_1= 1e-3; //1e-3;
        r_weights_2= 1e-4; //1e-4;

        r_weights_3= 1e-3; //1e-3;
        r_weights_4= 1e-3; //1e-3;
        r_weights_5= 1e-4; //1e-4;

        r_weights_6= 1e-3; //1e-3;
        r_weights_7= 1e-3; //1e-3;
        r_weights_8= 1e-4; //1e-4;

        r_weights_9 = 1e-3; //1e-3
        r_weights_10= 1e-3; //1e-3
        r_weights_11= 1e-4; //1e-4


        R_weights << r_weights_0, r_weights_1, r_weights_2,
                r_weights_3, r_weights_4, r_weights_5,
                r_weights_6, r_weights_7, r_weights_8,
                r_weights_9, r_weights_10, r_weights_11;
        km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

        gravity_torque << 0.80, 0, 0, 
                          -0.80, 0, 0, 
                          0.80, 0, 0, 
                          -0.80, 0, 0;

        double a1_trunk_inertia_xx;
        double a1_trunk_inertia_xy;
        double a1_trunk_inertia_xz;
        double a1_trunk_inertia_yz;
        double a1_trunk_inertia_yy;
        double a1_trunk_inertia_zz;
        a1_trunk_inertia_xx = 0.0178533;
        a1_trunk_inertia_xy = 0.0;
        a1_trunk_inertia_xz = 0.0;
        a1_trunk_inertia_yz = 0.0;
        a1_trunk_inertia_yy = 0.0377999;
        a1_trunk_inertia_zz = 0.0456542;

        a1_trunk_inertia <<a1_trunk_inertia_xx, a1_trunk_inertia_xy, a1_trunk_inertia_xz,
                a1_trunk_inertia_xy, a1_trunk_inertia_yy, a1_trunk_inertia_yz,
                a1_trunk_inertia_xz, a1_trunk_inertia_yz, a1_trunk_inertia_zz;
                
        reset_gait_count();
    }


void load_ros_yaml(ros::NodeHandle &_nh){
        // _nh.param("stance_leg_control_type",motion_mode,0);
        _nh.param("a1_robot_mass", robot_mass, 13.0);

        double a1_trunk_inertia_xx;
        double a1_trunk_inertia_xy;
        double a1_trunk_inertia_xz;
        double a1_trunk_inertia_yz;
        double a1_trunk_inertia_yy;
        double a1_trunk_inertia_zz;

        _nh.param("a1_trunk_inertia_xx", a1_trunk_inertia_xx, 0.0158533);
        _nh.param("a1_trunk_inertia_xy", a1_trunk_inertia_xy, 0.0);
        _nh.param("a1_trunk_inertia_xz", a1_trunk_inertia_xz, 0.0);
        _nh.param("a1_trunk_inertia_yz", a1_trunk_inertia_yz, 0.0);
        _nh.param("a1_trunk_inertia_yy", a1_trunk_inertia_yy, 0.0377999);
        _nh.param("a1_trunk_inertia_zz", a1_trunk_inertia_zz, 0.0456542);

        a1_trunk_inertia << a1_trunk_inertia_xx, a1_trunk_inertia_xy, a1_trunk_inertia_xz,
                                a1_trunk_inertia_xy, a1_trunk_inertia_yy, a1_trunk_inertia_yz,
                                a1_trunk_inertia_xz, a1_trunk_inertia_yz, a1_trunk_inertia_zz;

        double a1_default_foot_pos_FL_x;
        double a1_default_foot_pos_FL_y;
        double a1_default_foot_pos_FL_z;

        double a1_default_foot_pos_FR_x;
        double a1_default_foot_pos_FR_y;
        double a1_default_foot_pos_FR_z;

        double a1_default_foot_pos_RL_x;
        double a1_default_foot_pos_RL_y;
        double a1_default_foot_pos_RL_z;

        double a1_default_foot_pos_RR_x;
        double a1_default_foot_pos_RR_y;
        double a1_default_foot_pos_RR_z;

        _nh.param("a1_default_foot_pos_FL_x", a1_default_foot_pos_FL_x, 0.25);
        _nh.param("a1_default_foot_pos_FL_y", a1_default_foot_pos_FL_y, 0.15);
        _nh.param("a1_default_foot_pos_FL_z", a1_default_foot_pos_FL_z, -0.33);

        _nh.param("a1_default_foot_pos_FR_x", a1_default_foot_pos_FR_x, 0.25);
        _nh.param("a1_default_foot_pos_FR_y", a1_default_foot_pos_FR_y, -0.15);
        _nh.param("a1_default_foot_pos_FR_z", a1_default_foot_pos_FR_z, -0.33);

        _nh.param("a1_default_foot_pos_RL_x", a1_default_foot_pos_RL_x, -0.17);
        _nh.param("a1_default_foot_pos_RL_y", a1_default_foot_pos_RL_y, 0.15);
        _nh.param("a1_default_foot_pos_RL_z", a1_default_foot_pos_RL_z, -0.33);

        _nh.param("a1_default_foot_pos_RR_x", a1_default_foot_pos_RR_x, -0.17);
        _nh.param("a1_default_foot_pos_RR_y", a1_default_foot_pos_RR_y, -0.15);
        _nh.param("a1_default_foot_pos_RR_z", a1_default_foot_pos_RR_z, -0.33);

        foot_pos_default<<a1_default_foot_pos_FL_x, a1_default_foot_pos_FR_x, a1_default_foot_pos_RL_x, a1_default_foot_pos_RR_x,
                a1_default_foot_pos_FL_y, a1_default_foot_pos_FR_y, a1_default_foot_pos_RL_y, a1_default_foot_pos_RR_y,
                a1_default_foot_pos_FL_z, a1_default_foot_pos_FR_z, a1_default_foot_pos_RL_z, a1_default_foot_pos_RR_z;

        double q_weights_0, q_weights_1, q_weights_2, q_weights_3, q_weights_4, q_weights_5, q_weights_6, q_weights_7, q_weights_8, q_weights_9, q_weights_10, q_weights_11, q_weights_12;

        _nh.param("q_weights_0", q_weights_0, 80.0);
        _nh.param("q_weights_1", q_weights_1, 80.0);
        _nh.param("q_weights_2", q_weights_2, 1.0);

        _nh.param("q_weights_3", q_weights_3, 0.0);
        _nh.param("q_weights_4", q_weights_4, 0.0);
        _nh.param("q_weights_5", q_weights_5, 270.0);

        _nh.param("q_weights_6", q_weights_6, 1.0);
        _nh.param("q_weights_7", q_weights_7, 1.0);
        _nh.param("q_weights_8", q_weights_8, 20.0);

        _nh.param("q_weights_9", q_weights_9, 20.0);
        _nh.param("q_weights_10", q_weights_10, 20.0);
        _nh.param("q_weights_11", q_weights_11, 20.0);

        _nh.param("q_weights_12", q_weights_12, 0.0);

        Q_weights<< q_weights_0, q_weights_1, q_weights_2,
                        q_weights_3, q_weights_4, q_weights_5,
                        q_weights_6, q_weights_7, q_weights_8,
                        q_weights_9, q_weights_10, q_weights_11,
                        q_weights_12;

        double r_weights_0, r_weights_1, r_weights_2, r_weights_3, r_weights_4, r_weights_5, r_weights_6, r_weights_7, r_weights_8, r_weights_9, r_weights_10, r_weights_11;
        _nh.param("r_weights_0", r_weights_0, 1e-5);
        _nh.param("r_weights_1", r_weights_1, 1e-5);
        _nh.param("r_weights_2", r_weights_2, 1e-6);

        _nh.param("r_weights_3", r_weights_3, 1e-5);
        _nh.param("r_weights_4", r_weights_4, 1e-5);
        _nh.param("r_weights_5", r_weights_5, 1e-6);

        _nh.param("r_weights_6", r_weights_6, 1e-5);
        _nh.param("r_weights_7", r_weights_7, 1e-5);
        _nh.param("r_weights_8", r_weights_8, 1e-6);

        _nh.param("r_weights_9", r_weights_9, 1e-5);
        _nh.param("r_weights_10", r_weights_10, 1e-5);
        _nh.param("r_weights_11", r_weights_11, 1e-6);
        
        R_weights << r_weights_0, r_weights_1, r_weights_2,
                r_weights_3, r_weights_4, r_weights_5,
                r_weights_6, r_weights_7, r_weights_8,
                r_weights_9, r_weights_10, r_weights_11;

        
        double a1_kp_foot_x, a1_kp_foot_y, a1_kp_foot_z, a1_kd_foot_x, a1_kd_foot_y, a1_kd_foot_z, a1_km_foot_x, a1_km_foot_y, a1_km_foot_z;
        _nh.param("a1_kp_foot_x", a1_kp_foot_x, 150.0);
        _nh.param("a1_kp_foot_y", a1_kp_foot_y, 150.0);
        _nh.param("a1_kp_foot_z", a1_kp_foot_z, 200.0);

        _nh.param("a1_kd_foot_x", a1_kd_foot_x, 0.0);
        _nh.param("a1_kd_foot_y", a1_kd_foot_y, 0.0);
        _nh.param("a1_kd_foot_z", a1_kd_foot_z, 0.0);

        _nh.param("a1_km_foot_x", a1_km_foot_x, 0.1);
        _nh.param("a1_km_foot_y", a1_km_foot_y, 0.1);
        _nh.param("a1_km_foot_z", a1_km_foot_z, 0.04);

        kp_swing_foot<<
                a1_kp_foot_x, a1_kp_foot_x, a1_kp_foot_x, a1_kp_foot_x,
                a1_kp_foot_y, a1_kp_foot_y, a1_kp_foot_y, a1_kp_foot_y,
                a1_kp_foot_z, a1_kp_foot_z, a1_kp_foot_z, a1_kp_foot_z;

        kd_swing_foot<<
                a1_kd_foot_x, a1_kd_foot_x, a1_kd_foot_x, a1_kd_foot_x,
                a1_kd_foot_y, a1_kd_foot_y, a1_kd_foot_y, a1_kd_foot_y,
                a1_kd_foot_z, a1_kd_foot_z, a1_kd_foot_z, a1_kd_foot_z;

        km_foot = Eigen::Vector3d(a1_km_foot_x, a1_km_foot_y, a1_km_foot_z);

        double a1_gait_counter_speed_FL;
        double a1_gait_counter_speed_FR;
        double a1_gait_counter_speed_RL;
        double a1_gait_counter_speed_RR;

        _nh.param("a1_gait_counter_speed_FL", a1_gait_counter_speed_FL, 2.0);
        _nh.param("a1_gait_counter_speed_FR", a1_gait_counter_speed_FR, 2.0);
        _nh.param("a1_gait_counter_speed_RL", a1_gait_counter_speed_RL, 2.0);
        _nh.param("a1_gait_counter_speed_RR", a1_gait_counter_speed_RR, 2.0);

        gait_count_speed<<
                a1_gait_counter_speed_FL, a1_gait_counter_speed_FR, 
                a1_gait_counter_speed_RL, a1_gait_counter_speed_RR;

        // std::cout<<"a1_gait_counter_speed is: "<<gait_count_speed<<std::endl;
}
void reset_gait_count(){gait_count<<0,120,120,0;}

Eigen::Vector3d root_pos;

Eigen::Quaterniond root_quat;

Eigen::Vector3d root_euler;
Eigen::Vector3d root_lin_vel;
Eigen::Vector3d root_ang_vel;
Eigen::Vector3d root_acc;

Eigen::Matrix3d root_rot_matrix;
Eigen::Matrix3d root_rot_mat_z;

Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
Eigen::Matrix<double, NUM_DOF, 1> joint_vel;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // in the robot frame
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_desired_rel;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_desired_rel_next;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_desired_abs;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_desired_world;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_default;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;
Eigen::Matrix<double, 3, NUM_LEG> prev_foot_pos_rel;
Eigen::Matrix<double, 3, NUM_LEG> prev_foot_pos_target;

Eigen::Matrix<double, NUM_DOF, 1> gravity_torque;
Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
Eigen::Matrix<double, NUM_DOF, NUM_DOF> j_foot;

Eigen::Vector4d foot_force;

// IMU sensor data
Eigen::Vector3d imu_acc;
Eigen::Vector3d imu_ang_vel;

// state estimation
bool estimated_contacts[4]; 
Eigen::Vector3d estimated_root_pos;
Eigen::Vector3d estimated_root_vel;
int power_level;


//Joystick command 
Eigen::Vector3d root_pos_desired;
Eigen::Vector3d root_euler_desired;
Eigen::Vector3d root_lin_vel_desired;
Eigen::Vector3d root_lin_vel_desired_world;
Eigen::Vector3d root_ang_vel_desired;
Eigen::Vector3d root_ang_vel_desired_world;


// important kinematics constants
double robot_mass;
int motion_mode;

double kp_linear_lock_x, kp_linear_lock_y;
Eigen::Vector3d kp_linear;
Eigen::Vector3d kd_linear;
Eigen::Vector3d kp_angular;
Eigen::Vector3d kd_angular;


//mpc definition
Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_desired;
Eigen::Matrix3d a1_trunk_inertia;

// control module 
int counter;
bool contacts[NUM_LEG];
bool expected_contacts[NUM_LEG];
bool early_contacts[NUM_LEG];
Eigen::Vector4d gait_count;
Eigen::Vector4d gait_count_speed;
double gait_count_cycle;
double swing_leg_cycle;
double control_dt = MAIN_UPDATE_FREQUENCY / 1000.0;
Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
Eigen::Matrix<double, 3, NUM_LEG> kp_swing_foot;
Eigen::Matrix<double, 3, NUM_LEG> kd_swing_foot;
Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;
Eigen::Matrix<double, 3, NUM_LEG> swing_leg_forces;
Eigen::Matrix<double, 3, NUM_LEG> stance_leg_forces;
Eigen::VectorXd Q_weights;
Eigen::VectorXd R_weights;
Eigen::Matrix<double, 3, 1> km_foot;
a1_msgs::MultiFootPlanDiscrete foot_plans;

std::vector<int> FL_color_;
std::vector<int> RL_color_;
std::vector<int> FR_color_;
std::vector<int> RR_color_;

};

#endif