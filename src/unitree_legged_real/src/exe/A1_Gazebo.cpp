#include "A1_Gazebo.h"

A1_Gazebo::A1_Gazebo(ros::NodeHandle &_nh){
    nh = _nh;
    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);

    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/gazebo_a1/estimation_body_pose", 100);
    pub_euler_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/euler_desired",100);
    pub_root_lin_vel_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_lin_vel_desired",100);
    pub_root_lin_vel = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_lin_vel",100);

    pub_root_lin_vel_desired_x = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_desired_x",1);
    pub_root_lin_vel_x         = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_x",1);
    pub_root_lin_vel_desired_y = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_desired_y",1);
    pub_root_lin_vel_y         = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_y",1);
    pub_root_yaw_desired       = nh.advertise<std_msgs::Float64>("a1_debug/root_yaw_desired",1);
    pub_root_yaw       = nh.advertise<std_msgs::Float64>("a1_debug/root_yaw",1);

    pub_root_ang_vel_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_ang_vel_desired",100);
    pub_root_pos_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_pos_desired",100);
    pub_desired_height_joy = nh.advertise<std_msgs::Float64>("a1_debug/joy_height",1);
    pub_mpc_height=nh.advertise<std_msgs::Float64>("a1_debug/actual_height",1);

    rviz_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",0);
    marker_timer = nh.createTimer(ros::Duration(0.0025), &A1_Gazebo::marker_callback, this);
    joint_pub_rviz = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    foot_plan_discrete_pub = nh.advertise<visualization_msgs::Marker>("foot_plan_discrete_rviz",1);

    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    a1_control = robot_control();
    robot_state.reset();
    robot_state.load_ros_yaml(nh);

    sub_joint_msg[0] = nh.subscribe("/a1_gazebo/FL_hip_controller/state", 2, &A1_Gazebo::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/a1_gazebo/FL_thigh_controller/state", 2, &A1_Gazebo::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/a1_gazebo/FL_calf_controller/state", 2, &A1_Gazebo::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/a1_gazebo/FR_hip_controller/state", 2, &A1_Gazebo::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/a1_gazebo/FR_thigh_controller/state", 2, &A1_Gazebo::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/a1_gazebo/FR_calf_controller/state", 2, &A1_Gazebo::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/a1_gazebo/RL_hip_controller/state", 2, &A1_Gazebo::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/a1_gazebo/RL_thigh_controller/state", 2, &A1_Gazebo::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/a1_gazebo/RL_calf_controller/state", 2, &A1_Gazebo::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/a1_gazebo/RR_hip_controller/state", 2, &A1_Gazebo::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &A1_Gazebo::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/a1_gazebo/RR_calf_controller/state", 2, &A1_Gazebo::RR_calf_state_callback, this);

    // sub_gt_pose_msg = nh.subscribe("/torso_odom", 100, &A1_Gazebo::gt_pose_callback, this);
    sub_joy_msg = nh.subscribe("/joy", 1000, &A1_Gazebo::joy_callback, this);

    sub_imu_msg = nh.subscribe("/trunk_imu", 100, &A1_Gazebo::imu_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/visual/FL_foot_contact/the_force", 2, &A1_Gazebo::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/visual/FR_foot_contact/the_force", 2, &A1_Gazebo::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/visual/RL_foot_contact/the_force", 2, &A1_Gazebo::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/visual/RR_foot_contact/the_force", 2, &A1_Gazebo::RR_foot_contact_callback, this);

    acc_x = MovingWindowFilter(5);
    acc_y = MovingWindowFilter(5);
    acc_z = MovingWindowFilter(5);
    gyro_x = MovingWindowFilter(5);
    gyro_y = MovingWindowFilter(5);
    gyro_z = MovingWindowFilter(5);
    quat_w = MovingWindowFilter(5);
    quat_x = MovingWindowFilter(5);
    quat_y = MovingWindowFilter(5);
    quat_z = MovingWindowFilter(5);
    // std::cout<< "Run_2"<<std::endl;

    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.21;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    joint_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    joint_msg.position.resize(12);
    joint_msg.velocity.resize(12);
    joint_msg.effort.resize(12);
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
}

bool A1_Gazebo::get_stance_leg_forces(double dt){
    robot_state.stance_leg_forces = a1_control.stance_leg_control(robot_state, dt);
    return true;
}

bool A1_Gazebo::mujoco_data_received(){
    if (robot_state.root_rot_matrix.determinant()==0){
        return false;
    } else {
        return true;
    }
}

void A1_Gazebo::marker_callback(const ros::TimerEvent&){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(robot_state.root_pos(0),
                                    robot_state.root_pos(1),
                                    robot_state.root_pos(2)));

    tf::Quaternion q;
    q.setRPY(robot_state.root_euler(0),
            robot_state.root_euler(1),
            robot_state.root_euler(2));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "trunk"));

    // marker_array.markers.clear();
    // visualization_msgs::Marker sphere_marker;

    // sphere_marker.header.stamp = ros::Time::now();
    // sphere_marker.ns = "foot position world frame";
    // sphere_marker.type = visualization_msgs::Marker::SPHERE;
    // sphere_marker.action = visualization_msgs::Marker::ADD;

    // int marker_id = 0;
    // for (int i =0; i<NUM_LEG; ++i){
    //     sphere_marker.header.frame_id = "base";
    //     sphere_marker.id = marker_id;
    //     sphere_marker.pose.position.x = robot_state.foot_pos_world(0,i); //robot_state.foot_pos_rel(0,i);//
    //     sphere_marker.pose.position.y = robot_state.foot_pos_world(1,i); // robot_state.foot_pos_rel(1,i);//
    //     sphere_marker.pose.position.z = robot_state.foot_pos_world(2,i); // robot_state.foot_pos_rel(2,i);//
    //     sphere_marker.scale.x = 0.05;
    //     sphere_marker.scale.y = 0.05;
    //     sphere_marker.scale.z = 0.05;
    //     sphere_marker.pose.orientation.x = 0.0;
    //     sphere_marker.pose.orientation.y = 0.0;
    //     sphere_marker.pose.orientation.z = 0.0;
    //     sphere_marker.pose.orientation.w = 1.0;
    //     sphere_marker.color.a = 1.0;
    //     sphere_marker.color.r = 1.0 * robot_state.estimated_contacts[i];
    //     sphere_marker.color.g = 1.0 * (1.0 - robot_state.estimated_contacts[i]);
    //     sphere_marker.color.b = 0.0;
    //     marker_array.markers.push_back(sphere_marker);
    //     ++marker_id;    
    //     // std::cout<<"Contact"<< i <<" is :"<< robot_state.estimated_contacts[i]<<std::endl;
    // }
    joint_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < 12; ++i){
        int swap_i = swap_joint_indices(i);
        joint_msg.position[i]    =  robot_state.joint_pos[swap_i]; //FR
        joint_msg.velocity[i]    =  robot_state.joint_vel[swap_i];
    }
    // std::cout<<"Estimation X pos: "<< robot_state.root_pos(0) <<" Estimation Y pos: "<< robot_state.root_pos(1)<<std::endl;
    // rviz_marker_pub.publish(marker_array);
    joint_pub_rviz.publish(joint_msg);
    // std::cout<<"running"<<std::endl;
}


bool A1_Gazebo::main_update(double t, double dt){
    if (joy_cmd_exit) {
        return false;
    }
    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into a1_ctrl_states
    // dt = 0.0025;
    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }

    
    
    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state; //joy_cmd_ctrl_state;
// 
    if (joy_cmd_ctrl_state_change_request) {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; //TODO: how to toggle more states?
        joy_cmd_ctrl_state_change_request = false; //erase this change request;
    }

    // root_lin_vel_d is in robot frame
    robot_state.root_lin_vel_desired[0] = joy_cmd_velx;
    robot_state.root_lin_vel_desired[1] = joy_cmd_vely;
    robot_state.root_lin_vel_desired[2] = joy_cmd_velz;

    // root_ang_vel_d is in robot frame
    robot_state.root_ang_vel_desired[0] = joy_cmd_roll_rate;
    robot_state.root_ang_vel_desired[1] = joy_cmd_pitch_rate;
    robot_state.root_ang_vel_desired[2] = joy_cmd_yaw_rate;
    // std::cout<<"joy_cmd_roll_rate is "<<robot_state.root_ang_vel_desired[0]<<std::endl;
    // std::cout<<"joy_cmd_pitch_rate is "<<robot_state.root_ang_vel_desired[1]<<std::endl;
    // std::cout<<"joy_cmd_yaw_rate is "<<robot_state.root_ang_vel_desired[2]<<std::endl;

    robot_state.root_euler_desired[0] = joy_cmd_roll_rate;
    robot_state.root_euler_desired[1] = joy_cmd_pitch_rate;
    robot_state.root_euler_desired[2] += joy_cmd_yaw_rate * dt;
    // std::cout<<"yaw desired is "<<robot_state.root_euler_desired[2]<<std::endl;
    // std::cout<<"yaw actual is "<<robot_state.root_euler[2]<<std::endl;

    robot_state.root_pos_desired[2] =  joy_cmd_body_height;

    // std::cout<<"desired height: "<< joy_cmd_body_height<<std::endl;
    // std::cout<<"estimated height: "<< robot_state.root_pos[2] << std::endl;
    // determine movement mode
    
    if (joy_cmd_ctrl_state == 1) {
        // walking mode, in this mode the robot should execute gait
        robot_state.motion_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        robot_state.motion_mode = 0;
        robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
        // std::cout<<"root pos desired: "<<robot_state.root_pos_desired.segment<2>(0)<<std::endl;
        robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
        robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
    } else {
        robot_state.motion_mode = 0;
    }
    // std::cout<<"root_pos_desired x: "<<robot_state.root_pos_desired[0] <<" y: "<< robot_state.root_pos_desired[1] <<std::endl;
    // std::cout<<"root_pos_now x: "<<robot_state.root_pos[0] <<" y: "<< robot_state.root_pos[1] <<std::endl;

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (robot_state.motion_mode == 1) {
        robot_state.root_pos_desired[2] =  0.3;
        robot_state.root_euler_desired[0] = 0;
        robot_state.root_euler_desired[1] = 0;
        if (robot_state.root_lin_vel_desired.segment<2>(0).norm() > 0.05) {
            // std::cout<<"lin_velocity_non_zero!! Refresh root_desired"<< std::endl;
            // has nonzero velocity, keep refreshing position target, but just xy
            robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
            robot_state.kp_linear.segment<2>(0).setZero();
        } else {
            robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
            robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
        }
        // robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
    }

    a1_control.get_desired_foot_pos(robot_state, dt);
    a1_control.swing_leg_control(robot_state, dt);

    // state estimation
    if (!a1_estimation.is_initialized()) {
        a1_estimation.init_state(robot_state);
    } else {
        a1_estimation.update_estimation(robot_state, dt);
    }

    // std::cout<<"kalman filter dt is: "<< dt <<std::endl;

    nav_msgs::Odometry estimate_odom;
    estimate_odom.header.stamp = ros::Time::now();
    estimate_odom.pose.pose.position.x = robot_state.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = robot_state.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = robot_state.estimated_root_pos(2);

    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = robot_state.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = robot_state.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = robot_state.estimated_root_vel(2);

    pub_estimated_pose.publish(estimate_odom);

    // std::cout<< ros::Time::now()<<std::endl;
    // std::cout<< joint_msg.position.empty() << std::endl;
    // joint_msg_rviz.header.stamp = ros::Time::now();
    // for (int i = 0; i<12; ++i){
    //     joint_msg_rviz.position[i] = joint_msg.position[i];
    //     joint_msg_rviz.velocity[i] = joint_msg.velocity[i];
    //     std::cout<<"joint_state position "<<i<<" is "<<joint_msg.position[i]<<std::endl;
    //     std::cout<<"joint_state velocity "<<i<<" is "<<joint_msg.velocity[i]<<std::endl;
    //     // std::cout<< "runned for "<< i <<std::endl;
    // }

    // joint_msg.position = {0};
    // joint_msg.velocity = {0};

    // std::cout<<joint_msg<<std::endl;
    // std::cout<<"11111"<<std::endl;
    // pub_euler_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/euler_desired",100);
    // pub_root_lin_vel_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_lin_vel_desired",100);
    // pub_root_ang_vel_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_ang_vel_desired",100);
    // pub_root_pos_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_pos_desired",100);

    geometry_msgs::PointStamped root_euler_desired_debug, lin_vel_desired_debug, lin_vel_debug,ang_vel_desirted_debug;
    geometry_msgs::Pose root_pos_desired_debug;

    root_euler_desired_debug.point.x = robot_state.root_euler_desired[0];
    root_euler_desired_debug.point.y = robot_state.root_euler_desired[1];
    root_euler_desired_debug.point.z = robot_state.root_euler_desired[2];

    // lin_vel_desired_debug.point.x = robot_state.root_lin_vel_desired[0];
    // lin_vel_desired_debug.point.y = robot_state.root_lin_vel_desired[1];
    // lin_vel_desired_debug.point.z = robot_state.root_lin_vel_desired[2];

    // lin_vel_debug.point.x = robot_state.root_lin_vel[0];
    // lin_vel_debug.point.y = robot_state.root_lin_vel[1];
    // lin_vel_debug.point.z = robot_state.root_lin_vel[2];

    ang_vel_desirted_debug.point.x = robot_state.root_ang_vel_desired[0];
    ang_vel_desirted_debug.point.y = robot_state.root_ang_vel_desired[1];
    ang_vel_desirted_debug.point.z = robot_state.root_ang_vel_desired[2];

    root_pos_desired_debug.position.x = robot_state.root_pos_desired[0];
    root_pos_desired_debug.position.y = robot_state.root_pos_desired[1];
    root_pos_desired_debug.position.z = robot_state.root_pos_desired[2];

    pub_euler_desired.publish(root_euler_desired_debug);
    // pub_root_lin_vel_desired.publish(lin_vel_desired_debug);
    // pub_root_lin_vel.publish(lin_vel_debug);
    std_msgs::Float64 lin_vel_d_x, lin_vel_d_y, lin_vel_x, lin_vel_y;
    std_msgs::Float64 yaw_desired, yaw_now;
    lin_vel_d_x.data = robot_state.root_lin_vel_desired_world[0];
    lin_vel_d_y.data = robot_state.root_lin_vel_desired_world[1];

    lin_vel_x.data = robot_state.root_lin_vel[0];
    lin_vel_y.data = robot_state.root_lin_vel[1];

    yaw_desired.data = robot_state.root_euler_desired[2];
    yaw_now.data = robot_state.root_euler(2);

    pub_root_lin_vel_desired_x.publish(lin_vel_d_x);
    pub_root_lin_vel_x.publish(lin_vel_x);        
    pub_root_lin_vel_desired_y.publish(lin_vel_d_y);
    pub_root_lin_vel_y.publish(lin_vel_y);    

    pub_root_ang_vel_desired.publish(ang_vel_desirted_debug);
    pub_root_pos_desired.publish(root_pos_desired_debug); 
    pub_root_yaw.publish(yaw_now);
    pub_root_yaw_desired.publish(yaw_desired);

    std_msgs::Float64 joy_height, root_pos_z;
    joy_height.data = joy_cmd_body_height;
    root_pos_z.data = robot_state.estimated_root_pos(2);
    pub_desired_height_joy.publish(joy_height);
    pub_mpc_height.publish(root_pos_z);


    int marker_id = 0;
    visualization_msgs::Marker points;
    points.header = robot_state.foot_plans.header;
    points.header.stamp = ros::Time::now();
    points.ns = "foot position world frame";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::SPHERE_LIST;

    points.scale.x = 0.02;
    points.scale.y = 0.02;
    points.scale.z = 0.02;
    // Loop through each foot
    for (int i = 0; i<NUM_LEG; ++i){
        //Loop through each footstep in plan
        int num_steps = robot_state.foot_plans.feet[i].footholds.size();
        // std::cout<<"#################################"<<std::endl;
        // std::cout<< "num step is "<< num_steps<<std::endl;
        for (int j = 0; j<num_steps; ++j){
            geometry_msgs::Point p;
            p.x = robot_state.foot_plans.feet[i].footholds[j].position.x;
            p.y = robot_state.foot_plans.feet[i].footholds[j].position.y;
            p.z = robot_state.foot_plans.feet[i].footholds[j].position.z;

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            if (i==0){
                color.r = (float) robot_state.FL_color_[0]/255.0;
                color.g = (float) robot_state.FL_color_[1]/255.0;
                color.b = (float) robot_state.FL_color_[2]/255.0;
            } else if(i==1){
                color.r = (float) robot_state.FR_color_[0]/255.0;
                color.g = (float) robot_state.FR_color_[1]/255.0;
                color.b = (float) robot_state.FR_color_[2]/255.0;
            } else if(i==2){
                color.r = (float) robot_state.RL_color_[0]/255.0;
                color.g = (float) robot_state.RL_color_[1]/255.0;
                color.b = (float) robot_state.RL_color_[2]/255.0;
            } else if(i==3){
                color.r = (float) robot_state.RR_color_[0]/255.0;
                color.g = (float) robot_state.RR_color_[1]/255.0;
                color.b = (float) robot_state.RR_color_[2]/255.0;
            }
            points.colors.push_back(color);
            points.points.push_back(p);
        }
    }
    foot_plan_discrete_pub.publish(points);


    return true;
}

bool A1_Gazebo::send_torque_cmd(){

    a1_control.calculate_joint_torques(robot_state);

    // send control cmd to robot via ros topic
    unitree_legged_msgs::LowCmd low_cmd;

    //joint_pub_rviz.publish(joint_msg);
    for (int i = 0; i < 12; i++) {
        low_cmd.motorCmd[i].mode = 0x0A;
        low_cmd.motorCmd[i].q = 0;
        low_cmd.motorCmd[i].dq = 0;
        low_cmd.motorCmd[i].Kp = 0;
        low_cmd.motorCmd[i].Kd = 0;
        // int swap_i = swap_joint_indices(i);
        low_cmd.motorCmd[i].tau = robot_state.joint_torques(i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
        // std::cout<< "torque is: "<< low_cmd.motorCmd[i].tau<<std::endl;
    }
    // std::cout << "Torque for FL: "<< low_cmd.motorCmd[0].tau<<", "<< low_cmd.motorCmd[1].tau<<", "<< low_cmd.motorCmd[2].tau<<std::endl;
    // std::cout << "Torque for FR: "<< low_cmd.motorCmd[3].tau<<", "<< low_cmd.motorCmd[4].tau<<", "<< low_cmd.motorCmd[5].tau<<std::endl;
    // std::cout << "Torque for RL: "<< low_cmd.motorCmd[6].tau<<", "<< low_cmd.motorCmd[7].tau<<", "<< low_cmd.motorCmd[8].tau<<std::endl;
    // std::cout << "Torque for RR: "<< low_cmd.motorCmd[9].tau<<", "<< low_cmd.motorCmd[10].tau<<", "<< low_cmd.motorCmd[11].tau<<std::endl;
    return true;
}   

void A1_Gazebo::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg){
    // left stick vertical  
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL; 

        //A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }
    // right vertical 
    joy_cmd_velx = joy_msg->axes[4] * JOY_CMD_VELX_MAX;

    // right horizontal 
    joy_cmd_vely = joy_msg->axes[3] * JOY_CMD_VELY_MAX;

    // left stick horizontal  
    joy_cmd_yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // std::cout<< joy_cmd_yaw_rate <<std::endl;

    // up-down button
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;

    // left-right button
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;

    // B
    if (joy_msg->buttons[1] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}

// void A1_Gazebo::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom){
//     // ROS_INFO("ssssss");
//     robot_state.root_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
//                                                 odom->pose.pose.orientation.x,
//                                                 odom->pose.pose.orientation.y,
//                                                 odom->pose.pose.orientation.z);    
//     // std::cout << "sssssssssssssss"<<odom->pose.pose.orientation.w << odom->pose.pose.orientation.x<<odom->pose.pose.orientation.y<<odom->pose.pose.orientation.z<<std::endl;
//     robot_state.root_rot_matrix = robot_state.root_quat.toRotationMatrix();
//     robot_state.root_euler = Utils::quat_to_euler(robot_state.root_quat);
    
//     double yaw_angle = robot_state.root_euler[2];

//     robot_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

//     // FL, FR, RL, RR
//     for (int i = 0; i < NUM_LEG; ++i) {
//         // robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.analytical_leg_jacobian(
//         //         robot_state.joint_pos.segment<3>(3*i),std::pow(-1,i));
//         robot_state.foot_pos_rel.block<3, 1>(0, i) = a1_dynamics_state.fk(
//                 robot_state.joint_pos.segment<3>(3 * i),
//                 rho_opt_list[i], rho_fix_list[i]);
//         robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.jac(
//                 robot_state.joint_pos.segment<3>(3*i),rho_opt_list[i], rho_fix_list[i]);

//         Eigen::Matrix3d tmp_mtx = robot_state.j_foot.block<3, 3>(3 * i, 3 * i);
//         Eigen::Vector3d tmp_vec = robot_state.joint_vel.segment<3>(3 * i);
//         robot_state.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

//         robot_state.foot_pos_abs.block<3, 1>(0, i) = robot_state.root_rot_matrix * robot_state.foot_pos_rel.block<3, 1>(0, i);
//         robot_state.foot_vel_abs.block<3, 1>(0, i) = robot_state.root_rot_matrix * robot_state.foot_vel_rel.block<3, 1>(0, i);

//         robot_state.foot_pos_world.block<3, 1>(0, i) = robot_state.foot_pos_abs.block<3, 1>(0, i) + robot_state.root_pos;
//         robot_state.foot_vel_world.block<3, 1>(0, i) = robot_state.foot_vel_abs.block<3, 1>(0, i) + robot_state.root_lin_vel;
//     }
//     //std::cout<<"foot_pos_abs"<<robot_state.foot_pos_abs<<std::endl;
// }

void A1_Gazebo::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
    // std::cout<< "running_imu"<< std::endl;
    // std::cout << "joint pos for FL: "<< robot_state.joint_pos[0]<<", "<< robot_state.joint_pos[1]<<", "<<  robot_state.joint_pos[2]<<std::endl;
    // std::cout << "joint pos for FR: "<< robot_state.joint_pos[3]<<", "<< robot_state.joint_pos[4]<<", "<<  robot_state.joint_pos[5]<<std::endl;
    // std::cout << "joint pos for RL: "<< robot_state.joint_pos[6]<<", "<< robot_state.joint_pos[7]<<", "<<  robot_state.joint_pos[8]<<std::endl;
    // std::cout << "joint pos for RR: "<< robot_state.joint_pos[9]<<", "<< robot_state.joint_pos[10]<<", "<< robot_state.joint_pos[11]<<std::endl;
    robot_state.root_quat = Eigen::Quaterniond(quat_w.CalculateAverage(imu->orientation.w),
                                               quat_x.CalculateAverage(imu->orientation.x),
                                               quat_y.CalculateAverage(imu->orientation.y),
                                               quat_z.CalculateAverage(imu->orientation.z));    
    // std::cout << "sssssssssssssss"<<odom->pose.pose.orientation.w << odom->pose.pose.orientation.x<<odom->pose.pose.orientation.y<<odom->pose.pose.orientation.z<<std::endl;
    robot_state.root_rot_matrix = robot_state.root_quat.toRotationMatrix();

    robot_state.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(imu->linear_acceleration.x),
            acc_y.CalculateAverage(imu->linear_acceleration.y),
            acc_z.CalculateAverage(imu->linear_acceleration.z)
    );
    robot_state.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(imu->angular_velocity.x),
            gyro_y.CalculateAverage(imu->angular_velocity.y),
            gyro_z.CalculateAverage(imu->angular_velocity.z)
    );
    robot_state.root_ang_vel = robot_state.root_rot_matrix * robot_state.imu_ang_vel;

    robot_state.root_euler = Utils::quat_to_euler(robot_state.root_quat);
    
    double yaw_angle = robot_state.root_euler[2];

    robot_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // FL, FR, RL, RR
    // std::cout<< robot_state.joint_pos <<std::endl;
    a1_dynamics_state.get_foot_position_base_frame(robot_state.joint_pos);

    robot_state.foot_pos_rel.block<3,1>(0,0) = a1_dynamics_state.res_FL;
    robot_state.foot_pos_rel.block<3,1>(0,1) = a1_dynamics_state.res_FR;
    robot_state.foot_pos_rel.block<3,1>(0,2) = a1_dynamics_state.res_RL;
    robot_state.foot_pos_rel.block<3,1>(0,3) = a1_dynamics_state.res_RR;

    robot_state.j_foot.block<3,3>(3*0, 3*0) = a1_dynamics_state.analytical_leg_jacobian(
                robot_state.joint_pos.segment<3>(3*0),0);
    robot_state.j_foot.block<3,3>(3*1, 3*1) = a1_dynamics_state.analytical_leg_jacobian(
                robot_state.joint_pos.segment<3>(3*1),1);
    robot_state.j_foot.block<3,3>(3*2, 3*2) = a1_dynamics_state.analytical_leg_jacobian(
                robot_state.joint_pos.segment<3>(3*2),0);
    robot_state.j_foot.block<3,3>(3*3, 3*3) = a1_dynamics_state.analytical_leg_jacobian(
                robot_state.joint_pos.segment<3>(3*3),1);

    // std::cout<< "MY FL " << a1_dynamics_state.res_FL << 
    //             "MY FR " << a1_dynamics_state.res_FR <<
    //             "MY RL " << a1_dynamics_state.res_RL <<
    //             "MY RR " << a1_dynamics_state.res_RR <<std::endl;
    for (int i = 0; i < NUM_LEG; ++i) {
        // robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.analytical_leg_jacobian(
        //         robot_state.joint_pos.segment<3>(3*i),std::pow(-1,i));
        // std::cout<< "MY J " << robot_state.j_foot.block<3,3>(3*i, 3*i) <<std::endl;
        // robot_state.foot_pos_rel.block<3, 1>(0, i) = a1_dynamics_state.fk(
        //         robot_state.joint_pos.segment<3>(3 * i),
        //         rho_opt_list[i], rho_fix_list[i]);
        // robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.jac(
        //         robot_state.joint_pos.segment<3>(3*i),rho_opt_list[i], rho_fix_list[i]);
        
        Eigen::Matrix3d tmp_mtx = robot_state.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = robot_state.joint_vel.segment<3>(3 * i);
        robot_state.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        robot_state.foot_pos_abs.block<3, 1>(0, i) = robot_state.root_rot_matrix * robot_state.foot_pos_rel.block<3, 1>(0, i);
        robot_state.foot_vel_abs.block<3, 1>(0, i) = robot_state.root_rot_matrix * robot_state.foot_vel_rel.block<3, 1>(0, i);

        robot_state.foot_pos_world.block<3, 1>(0, i) = robot_state.foot_pos_abs.block<3, 1>(0, i) + robot_state.root_pos;
        robot_state.foot_vel_world.block<3, 1>(0, i) = robot_state.foot_vel_abs.block<3, 1>(0, i) + robot_state.root_lin_vel;
    }
    // std::cout<< "THEIR FL " << robot_state.foot_pos_rel.block<3, 1>(0, 0) << 
    //     "THEIR FR " << robot_state.foot_pos_rel.block<3, 1>(0, 1) <<
    //     "THEIR RL " << robot_state.foot_pos_rel.block<3, 1>(0, 2) <<
    //     "THEIR RR " << robot_state.foot_pos_rel.block<3, 1>(0, 3) <<std::endl;
    //std::cout<<"foot_pos_abs"<<robot_state.foot_pos_abs<<std::endl;
    // std::cout<< "MY J FL " <<  a1_dynamics_state.analytical_leg_jacobian(
    //             robot_state.joint_pos.segment<3>(3*0),0)<<std::endl;
    // std::cout<< "Their J FL " << robot_state.j_foot.block<3,3>(3*0, 3*0) <<std::endl;

    //     std::cout<< "MY J FR " <<  a1_dynamics_state.analytical_leg_jacobian(
    //             robot_state.joint_pos.segment<3>(3*1),1)<<std::endl;
    // std::cout<< "Their J FR " << robot_state.j_foot.block<3,3>(3*1, 3*1) <<std::endl;

    //     std::cout<< "MY J RL " <<  a1_dynamics_state.analytical_leg_jacobian(
    //             robot_state.joint_pos.segment<3>(3*2),0)<<std::endl;
    // std::cout<< "Their J RL " << robot_state.j_foot.block<3,3>(3*2, 3*2) <<std::endl;

    //     std::cout<< "MY J RR " <<  a1_dynamics_state.analytical_leg_jacobian(
    //             robot_state.joint_pos.segment<3>(3*3),1)<<std::endl;
    // std::cout<< "Their J RR " << robot_state.j_foot.block<3,3>(3*3, 3*3) <<std::endl;
}

void A1_Gazebo::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[0] = a1_joint_state.q;
    robot_state.joint_vel[0] = a1_joint_state.dq;
}


void A1_Gazebo::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[1] = a1_joint_state.q;
    robot_state.joint_vel[1] = a1_joint_state.dq;
    //std::cout<<joint_msg.position[4]<<std::endl;
}

void A1_Gazebo::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[2] = a1_joint_state.q;
    robot_state.joint_vel[2] = a1_joint_state.dq;
}

void A1_Gazebo::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[3] = a1_joint_state.q;
    robot_state.joint_vel[3] = a1_joint_state.dq;
}


void A1_Gazebo::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[4] = a1_joint_state.q;
    robot_state.joint_vel[4] = a1_joint_state.dq;
}

void A1_Gazebo::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[5] = a1_joint_state.q;
    robot_state.joint_vel[5] = a1_joint_state.dq;
}

void A1_Gazebo::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[6] = a1_joint_state.q;
    robot_state.joint_vel[6] = a1_joint_state.dq;
}


void A1_Gazebo::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[7] = a1_joint_state.q;
    robot_state.joint_vel[7] = a1_joint_state.dq;
}

void A1_Gazebo::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[8] = a1_joint_state.q;
    robot_state.joint_vel[8] = a1_joint_state.dq;
}

void A1_Gazebo::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[9] = a1_joint_state.q;
    robot_state.joint_vel[9] = a1_joint_state.dq;
}


void A1_Gazebo::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[10] = a1_joint_state.q;
    robot_state.joint_vel[10] = a1_joint_state.dq;
}

void A1_Gazebo::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[11] = a1_joint_state.q;
    robot_state.joint_vel[11] = a1_joint_state.dq;
}

void A1_Gazebo::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[0] = force.wrench.force.z;
}

void A1_Gazebo::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[1] = force.wrench.force.z;
}

void A1_Gazebo::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[2] = force.wrench.force.z;
}

void A1_Gazebo::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[3] = force.wrench.force.z;
}
