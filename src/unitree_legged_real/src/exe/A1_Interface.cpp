#include "A1_Interface.h"

A1_Interface::A1_Interface(ros::NodeHandle &_nh):
safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
    nh = _nh;
    joint_pub_rviz = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    joint_angle_pub = nh.advertise<sensor_msgs::JointState>("/hardware_a1/joint_foot", 100);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/hardware_a1/imu",100);
    contact_pub = nh.advertise<a1_msgs::Contact>("/hardware_a1/contacts",100);
    pose_pub = nh.advertise<nav_msgs::Odometry>("/hardware_a1/estimation_body_pose",100);
    joy_msg_sub = nh.subscribe("/joy",1000, &A1_Interface::joystick_callback, this);
    
    rviz_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",0);
    
    marker_timer = nh.createTimer(ros::Duration(0.0002), &A1_Interface::marker_callback, this);
    
    pub_root_lin_vel_desired = nh.advertise<geometry_msgs::PointStamped>("a1_debug/root_lin_vel_desired",100);
    pub_root_lin_vel = nh.advertise<geometry_msgs::PointStamped>("a1_debug/pub_root_lin_vel",100);
    pub_root_lin_vel_desired_x = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_desired_x",1);
    pub_root_lin_vel_x         = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_x",1);
    pub_root_lin_vel_desired_y = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_desired_y",1);
    pub_root_lin_vel_y         = nh.advertise<std_msgs::Float64>("a1_debug/root_lin_vel_y",1);

    // joint_sub = nh.subscribe("/joint_states",100, &A1_Interface::joint_state_callback, this);
    a1_control = robot_control();
    robot_state.reset();
    // robot_state.reset_hardware();
    robot_state.load_ros_yaml(nh);

    // std::cout<<"kp is: "<<robot_state.kp_swing_foot<<std::endl;
    // std::cout<<"kd is: "<<robot_state.kd_swing_foot<<std::endl;
    // std::cout<<"Q weights: "<<robot_state.Q_weights<<std::endl;
    // std::cout<<"R Weights: "<<robot_state.R_weights<<std::endl;
    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    // joint_msg.name = {"FL0", "FL1", "FL2",
    //                   "FR0", "FR1", "FR2",
    //                   "RL0", "RL1", "RL2",
    //                   "RR0", "RR1", "RR2"};
    joint_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    joint_msg.position.resize(12);
    joint_msg.velocity.resize(12);
    joint_msg.effort.resize(12);

    // joint_msg_rviz.name = {"FL0", "FL1", "FL2",
    //                   "FR0", "FR1", "FR2",
    //                   "RL0", "RL1", "RL2",
    //                   "RR0", "RR1", "RR2"};
    // joint_msg_rviz.position.resize(12);
    // joint_msg_rviz.velocity.resize(12);
    // joint_msg_rviz.effort.resize(12);
    // roslcm.SubscribeState();

    // leg order: 0-FL  1-FR  2-RL  3-RR
    hip_offsets << 0.183, -0.047, 0,
                   0.183, -0.047, 0,
                   0.183, -0.047, 0,
                   0.183, -0.047, 0;
        
    com_offsets << -0.012731, -0.002186, -0.000515;
    

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
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.20;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.20;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    //init swap order, very important
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;

    // robot_state.gravity_torque<< 0.80, 0, 0, -0.80,0,0, 0.80,0,0, -0.80,0,0;
    
    foot_force_filters.setZero();
    foot_force_idx.setZero();
    foot_force_filters_sum.setZero();

    udp.InitCmdData(SendLowLCM);
    lcm_init();
    thread_ = std::thread(&A1_Interface::receive_motor_state, this);

    // thread2_ = std::thread(&A1_Interface::joint_test, this);
    // thread2_ = std::thread(&A1_Interface::standing_up, this);
}


void A1_Interface::joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state){
    for(int i=0;i<12;i++){
         qMujoco[i] = joint_state->position[i];   
    }
}

void A1_Interface::marker_callback(const ros::TimerEvent&){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(-robot_state.root_pos(0),
                                    -robot_state.root_pos(1),
                                    robot_state.root_pos(2)));

    tf::Quaternion q;
    q.setRPY(robot_state.root_euler(0),
            robot_state.root_euler(1),
            robot_state.root_euler(2));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "trunk"));

    marker_array.markers.clear();
    visualization_msgs::Marker sphere_marker;

    sphere_marker.header.stamp = ros::Time::now();
    sphere_marker.ns = "foot position world frame";
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::Marker::ADD;

    int marker_id = 0;
    for (int i =0; i<NUM_LEG; ++i){
        sphere_marker.header.frame_id = "base";
        sphere_marker.id = marker_id;
        sphere_marker.pose.position.x = -robot_state.foot_pos_world(0,i); //robot_state.foot_pos_rel(0,i);//
        sphere_marker.pose.position.y = -robot_state.foot_pos_world(1,i); // robot_state.foot_pos_rel(1,i);//
        sphere_marker.pose.position.z = robot_state.foot_pos_world(2,i); // robot_state.foot_pos_rel(2,i);//
        sphere_marker.scale.x = 0.05;
        sphere_marker.scale.y = 0.05;
        sphere_marker.scale.z = 0.05;
        sphere_marker.pose.orientation.x = 0.0;
        sphere_marker.pose.orientation.y = 0.0;
        sphere_marker.pose.orientation.z = 0.0;
        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.color.a = 1.0;
        sphere_marker.color.r = 1.0 * robot_state.estimated_contacts[i];
        sphere_marker.color.g = 1.0 * (1.0 - robot_state.estimated_contacts[i]);
        sphere_marker.color.b = 0.0;
        marker_array.markers.push_back(sphere_marker);
        ++marker_id;    
        // std::cout<<"Contact"<< i <<" is :"<< robot_state.estimated_contacts[i]<<std::endl;
    }

    // std::cout<<"Estimation X pos: "<< robot_state.root_pos(0) <<" Estimation Y pos: "<< robot_state.root_pos(1)<<std::endl;
    rviz_marker_pub.publish(marker_array);
}

bool A1_Interface::main_update(double t, double dt){
    if (joy_cmd_exit) {
        return false;
    }

    // access joystick data and get desired command 
    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }
    
    // 0 is standing 1 is walking
    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;
    
    if (joy_cmd_ctrl_state_change_request){
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2;
        joy_cmd_ctrl_state_change_request = false;
        
        if (joy_cmd_ctrl_state == 0){
            std::cout<<"Robot Switched to Stance Mode!"<<std::endl;
        }else{
            std::cout<<"Robot Switched to Walking Mode!"<<std::endl;
        }
    }
    

    // This is in robot body frame not in the world frame!
    robot_state.root_lin_vel_desired[0] = joy_cmd_velx;
    robot_state.root_lin_vel_desired[1] = joy_cmd_vely;
    // robot_state.root_lin_vel_desired[2] = joy_cmd_velz;

    // Process joystick for angle velocity 
    robot_state.root_ang_vel_desired[0] = joy_cmd_roll_rate;
    robot_state.root_ang_vel_desired[1] = joy_cmd_pitch_rate;
    robot_state.root_ang_vel_desired[2] = joy_cmd_yaw_rate;

    robot_state.root_euler_desired[0] = joy_cmd_roll_rate;
    robot_state.root_euler_desired[1] = joy_cmd_pitch_rate;
    robot_state.root_euler_desired[2] += joy_cmd_yaw_rate * dt;

    robot_state.root_pos_desired[2] =  joy_cmd_body_height;
    // std::cout<<"desired height: "<< joy_cmd_body_height<<std::endl;
    // std::cout<<"estimated height: "<< robot_state.root_pos[2] << std::endl;
    // consider movement mode 
    if (joy_cmd_ctrl_state == 1){
        robot_state.motion_mode = 1;
    } else if(joy_cmd_ctrl_state ==0 && prev_joy_cmd_ctrl_state==1){
        // from walking to stance mode
        robot_state.motion_mode = 0;
        robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
        robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
        robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
    } else{
        robot_state.motion_mode = 0;
    }
    // std::cout<<"root_pos_desired x: "<<robot_state.root_pos_desired[0] <<" y: "<< robot_state.root_pos_desired[1] <<std::endl;
    // std::cout<<"root_pos_now x: "<<robot_state.root_pos[0] <<" y: "<< robot_state.root_pos[1] <<std::endl;
    if (robot_state.motion_mode ==1 ){
        robot_state.root_pos_desired[2] =  0.28;
        robot_state.root_euler_desired[0] = 0;
        robot_state.root_euler_desired[1] = 0;
            
        if (robot_state.root_lin_vel_desired.segment<2>(0).norm() >0.05){
            // std::cout<<"lin_velocity_non_zero!! Refresh root_desired"<< std::endl;
            robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
            robot_state.kp_linear.segment<2>(0).setZero();
        }else {
            robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
            robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
        }
        // robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
    }

    a1_control.get_desired_foot_pos(robot_state, dt);
    a1_control.swing_leg_control(robot_state, dt);

    nav_msgs::Odometry pose_estimate;
    pose_estimate.header.stamp = ros::Time::now();
    pose_estimate.header.frame_id = "base";
    pose_estimate.child_frame_id = "trunk";
    pose_estimate.pose.pose.position.x = robot_state.estimated_root_pos(0);
    pose_estimate.pose.pose.position.y = robot_state.estimated_root_pos(1);
    pose_estimate.pose.pose.position.z = robot_state.estimated_root_pos(2);

    pose_estimate.twist.twist.linear.x = robot_state.estimated_root_vel(0);
    pose_estimate.twist.twist.linear.y = robot_state.estimated_root_vel(1);
    pose_estimate.twist.twist.linear.z = robot_state.estimated_root_vel(2);

    pose_pub.publish(pose_estimate);

    return true;
}

void A1_Interface::lcm_init(){
    SendLowLCM.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i <NUM_DOF; i++){
        SendLowLCM.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowLCM.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;        // 禁止位置环
        SendLowLCM.motorCmd[i].Kp = 0;
        SendLowLCM.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;        // 禁止速度环
        SendLowLCM.motorCmd[i].Kd = 0;
        SendLowLCM.motorCmd[i].tau = 0;
    }
    safe.PositionLimit(SendLowLCM);
    //safe.PowerProtect(SendLowLCM, RecvLowLCM, 1);
    //roslcm.Send(SendLowLCM);
    udp.SetSend(SendLowLCM);
    udp.Send();
    // ros::spinOnce(); 
    std::cout<<"LCM INIT SUCCESS"<<std::endl;
}

void A1_Interface::joystick_callback(const sensor_msgs::Joy::ConstPtr &joy_msg){
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
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);

    // B
    if (joy_msg->buttons[1] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}


void A1_Interface::receive_motor_state(){
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    while (destruct ==false ){
        // recieve low lowel motor state 
        //roslcm.Recv();
        //roslcm.Get(RecvLowLCM);
        // RecvLowROS = ToRos(RecvLowLCM);
        
        udp.Recv();
        udp.GetRecv(RecvLowLCM);
        // joy_msg_sub = nh.subscribe("/joy",1000, &A1_Interface::joystick_callback, this);
        //std::cout<<"1"<<std::endl;

        // imu data
        robot_state.root_quat = Eigen::Quaterniond(RecvLowLCM.imu.quaternion[0],
                                                   RecvLowLCM.imu.quaternion[1],
                                                   RecvLowLCM.imu.quaternion[2],
                                                   RecvLowLCM.imu.quaternion[3]);
        robot_state.root_rot_matrix = robot_state.root_quat.toRotationMatrix();
        // std::cout<<"here we have : "<< robot_state.root_rot_matrix<<std::endl;
        robot_state.root_euler = Utils::quat_to_euler(robot_state.root_quat);//robot_state.root_rot_matrix.eulerAngles(2,1,0);
        
        double yaw_angle = robot_state.root_euler[2];
        robot_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

        robot_state.imu_acc = Eigen::Vector3d(RecvLowLCM.imu.accelerometer[0],
                                              RecvLowLCM.imu.accelerometer[1],
                                              RecvLowLCM.imu.accelerometer[2]);
        // std::cout<<"IMU_X: "<< RecvLowLCM.imu.accelerometer[0] << "IMU_Y: "<< RecvLowLCM.imu.accelerometer[1]<<std::endl;

        robot_state.imu_ang_vel = Eigen::Vector3d(RecvLowLCM.imu.gyroscope[0],
                                                  RecvLowLCM.imu.gyroscope[1],
                                                  RecvLowLCM.imu.gyroscope[2]);

        robot_state.root_ang_vel = robot_state.root_rot_matrix * robot_state.imu_ang_vel;

        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        double dt_pub = dt.toSec();


        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.angular_velocity.x = RecvLowLCM.imu.gyroscope[0];
        imu_msg.angular_velocity.y = RecvLowLCM.imu.gyroscope[1];
        imu_msg.angular_velocity.z = RecvLowLCM.imu.gyroscope[2];


        imu_msg.linear_acceleration.x = RecvLowLCM.imu.accelerometer[0];
        imu_msg.linear_acceleration.y = RecvLowLCM.imu.accelerometer[1];
        imu_msg.linear_acceleration.z = RecvLowLCM.imu.accelerometer[2];
        imu_pub.publish(imu_msg);

        // Joint State 
        // FL, FR, RL, RR
        for (int i = 0; i < 12; ++i){
            int swap_i = swap_joint_indices(i);
            //   /
            //swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
            robot_state.joint_vel[i] = RecvLowLCM.motorState[swap_i].dq;
            robot_state.joint_pos[i] = RecvLowLCM.motorState[swap_i].q;  // standing_pose[i]; 
            
            joint_msg.position[i]    =  RecvLowLCM.motorState[i].q; // standing_pose[i];
            joint_msg.velocity[i] = RecvLowLCM.motorState[i].dq;
        }
        // std::cout << "joint pos for FL: "<< robot_state.joint_pos[0]<<", "<< robot_state.joint_pos[1]<<", "<<  robot_state.joint_pos[2]<<std::endl;
        // std::cout << "joint pos for FR: "<< robot_state.joint_pos[3]<<", "<< robot_state.joint_pos[4]<<", "<<  robot_state.joint_pos[5]<<std::endl;
        // std::cout << "joint pos for RL: "<< robot_state.joint_pos[6]<<", "<< robot_state.joint_pos[7]<<", "<<  robot_state.joint_pos[8]<<std::endl;
        // std::cout << "joint pos for RR: "<< robot_state.joint_pos[9]<<", "<< robot_state.joint_pos[10]<<", "<< robot_state.joint_pos[11]<<std::endl;

        // 0.389943  1.02229  -2.61051
        // -0.355934 1.04382 -2.60338
        // 0.34995   1.03965   -2.6167
        // -0.36057  1.03312  -2.61139
        // std::cout<< "joint_pos: " << robot_state.joint_pos <<std::endl;
        //std::cout<<"2"<<std::endl;
        joint_msg.header.stamp = ros::Time::now();
        joint_angle_pub.publish(joint_msg);

        joint_pub_rviz.publish(joint_msg);

        //foot force 
        for (int i = 0; i < 4; ++i){
            // swap_foot_indices << 1, 0, 3, 2;
            int swap_i = swap_foot_indices(i);

            double value = static_cast<double>(RecvLowLCM.footForce[swap_i]);

            foot_force_filters_sum[i] -= foot_force_filters(i, foot_force_idx[i]);
            foot_force_filters(i, foot_force_idx[i]) = value;
            foot_force_filters_sum[i] += value;

            foot_force_idx[i]++;
            foot_force_idx[i] %= FOOT_FILTER_WINDOW_SIZE;
            
            robot_state.foot_force[i] = foot_force_filters_sum[i]/static_cast<double>(FOOT_FILTER_WINDOW_SIZE);
            // robot_state.foot_force[i] = static_cast<double>(RecvLowLCM.footForce[i]);
            // contact_msg.contacts[i] = robot_state.foot_force[i];
        }
        
        // std::cout<<"The contact force in FL" <<": " <<robot_state.foot_force[0]<<std::endl;
        // std::cout<<"This is FR" <<": " <<robot_state.foot_force[1]<<std::endl;
        // std::cout<<"The contact force in RL is" <<": " <<robot_state.foot_force[2]<<std::endl;
        // std::cout<<"This is RR" <<": " <<robot_state.foot_force[3]<<std::endl;

        //contact_pub.publish(contact_msg);
        //get foot pos and vel in the world frame
        // FL, FR, RL, RR
        // Eigen::Matrix<double, 3, 4> Temp_angle = robot_state.joint_pos.resize(3,4)
        // a1_dynamics_state.get_foot_position_base_frame(robot_state.joint_pos);
        

        // for (int i = 0; i < 4; i++){
        //     robot_state.foot_pos_rel.block<3,1>(0,i) = 
        // }

        
        auto t1 = ros::Time::now();
        if (!a1_estimation.is_initialized()){
            a1_estimation.init_state(robot_state);
            std::cout<<"Filter Initialized!"<<std::endl;
        } else {
            a1_estimation.update_estimation(robot_state, dt_pub);
        }
        auto t2 = ros::Time::now();
        ros::Duration update_time = t2-t1;
        
        lin_vel_d_x.data = robot_state.root_lin_vel_desired_world[0];
        lin_vel_d_y.data = robot_state.root_lin_vel_desired_world[1];

        lin_vel_x.data = robot_state.root_lin_vel[0];
        lin_vel_y.data = robot_state.root_lin_vel[1];

        pub_root_lin_vel_desired_x.publish(lin_vel_d_x);
        pub_root_lin_vel_x.publish(lin_vel_x);        
        pub_root_lin_vel_desired_y.publish(lin_vel_d_y);
        pub_root_lin_vel_y.publish(lin_vel_y); 
        // lin_vel_desired_debug.point.x = robot_state.root_lin_vel_desired[0];
        // lin_vel_desired_debug.point.y = robot_state.root_lin_vel_desired[1];

        // lin_vel_debug.point.x = robot_state.root_lin_vel[0];
        // lin_vel_debug.point.y = robot_state.root_lin_vel[1];

        // pub_root_lin_vel_desired.publish(lin_vel_desired_debug);
        // pub_root_lin_vel.publish(lin_vel_debug);



        // std::cout<<"root_lin_vel "<< "x: "<<robot_state.root_lin_vel[0]<< " y: "<<robot_state.root_lin_vel[1]<<std::endl;
        // std::cout<<"root_lin_vel_desired_world "<<"x: "<< robot_state.root_lin_vel_desired_world[0]
        //                                         <<" y: "<< robot_state.root_lin_vel_desired_world[1]<<std::endl;

        // std::cout<<"dt_pub is: "<<dt_pub<<std::endl;
        // std::cout<<"update time is: "<< update_time.toSec()<<std::endl;
        // start to update the robot control state based on the estimation 
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

        for (int i = 0; i < NUM_LEG; ++i){
            // robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.analytical_leg_jacobian(
            //     robot_state.joint_pos.segment<3>(3*i),std::pow(-1,i)
            // );

            Eigen::Matrix3d tmp_mtx = robot_state.j_foot.block<3,3>(3*i,3*i);
            Eigen::Vector3d tmp_vec = robot_state.joint_vel.segment<3>(3*i);

            // get robot joint velocity 
            robot_state.foot_vel_rel.block<3,1>(0,i) = tmp_mtx * tmp_vec;

            robot_state.foot_pos_abs.block<3,1>(0,i) = robot_state.root_rot_matrix * robot_state.foot_pos_rel.block<3,1>(0,i);
            robot_state.foot_vel_abs.block<3,1>(0,i) = robot_state.root_rot_matrix * robot_state.foot_vel_rel.block<3,1>(0,i); 

            robot_state.foot_pos_world.block<3,1>(0,i) = robot_state.foot_pos_abs.block<3,1>(0,i) + robot_state.root_pos;
            robot_state.foot_vel_world.block<3,1>(0,i) = robot_state.foot_vel_abs.block<3,1>(0,i) + robot_state.root_lin_vel;

        }
        double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
        // sleep for interval_ms
        double interval_time = interval_ms / 1000.0;
        if (interval_time > update_time.toSec()) {
            ros::Duration(interval_time - update_time.toSec()).sleep();
        }
    };
}

double A1_Interface::jointLinearInterpolation(double initPos, double targetPos, double rate){
        double p;
        rate = std::min(std::max(rate, 0.0), 1.0);
        p = initPos*(1-rate) + targetPos*rate;
        return p;    
}

bool A1_Interface::get_stance_leg_forces(double dt){
    robot_state.stance_leg_forces = a1_control.stance_leg_control(robot_state,dt);
    return true;
}

bool A1_Interface::standing_up(){
    
    // std::cout << "WARNING: Robot will stand up." << std::endl
    //           << "Prepare to stop anytime." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();
    //Waiting for a short time to recieve curr State
    count++;
    if(count > 10){
        count = 10;
        initiated_flag = true;
    }
    // starting standing sequence

    if(initiated_flag==true){
        motiontime++;
        // std::cout<<"motiontime is: "<<motiontime<<std::endl;
        if(motiontime>=0 && motiontime < 10){
            // get Initial Position
            //qInit[idx] = RecvLowLCM.motorState[idx].q;    
            // std::cout<< "qInit_0: "<< qInit[0]<< "qInit_1: "<< qInit[1]<< "qInit_2: "<< qInit[2]<<std::endl;
            for (int i=0; i<12;i++){ 
                qInit[i] = RecvLowLCM.motorState[i].q;
            }    
            std::cout<< "qInit_0: "<< qInit[0]<< "qInit_1: "<< qInit[1]<< "qInit_2: "<< qInit[2]<<std::endl;
        }
        if( motiontime >= 10){
                rate_count++;
                double rate = rate_count/600.0;                       // needs count to 200
                for (int i=0; i<12;i++){                       
                    Kp[i] = 5.0;
                    Kd[i] = 1.0;          
                    qDes[i] = jointLinearInterpolation(qInit[i], standing_pose[i], rate);
                }
            }
        // std::cout<<"The qDes[2] will be: "<<qDes[2]<<std::endl;
        }
    return true;
}

bool A1_Interface::joint_test(){
    
    // std::cout << "WARNING: Robot will stand up." << std::endl
    //           << "Prepare to stop anytime." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();
    //Waiting for a short time to recieve curr State
    count++;
    if(count > 10){
        count = 10;
        initiated_flag = true;
    }
    // starting standing sequence

    if(initiated_flag==true){
        motiontime++;
        // std::cout<<"motiontime is: "<<motiontime<<std::endl;
        if(motiontime>=0 && motiontime < 10){
            // get Initial Position
            //qInit[idx] = RecvLowLCM.motorState[idx].q;    
            // std::cout<< "qInit_0: "<< qInit[0]<< "qInit_1: "<< qInit[1]<< "qInit_2: "<< qInit[2]<<std::endl;
            qInit[0] = RecvLowLCM.motorState[0].q;
            qInit[1] = RecvLowLCM.motorState[1].q;
            qInit[2] = RecvLowLCM.motorState[2].q;
            std::cout<< "qInit_0: "<< qInit[0]<< "qInit_1: "<< qInit[1]<< "qInit_2: "<< qInit[2]<<std::endl;
        }
        if( motiontime >= 10 && motiontime < 1000){
                // printf("%f %f %f\n", );
                rate_count++;
                double rate = rate_count/400.0;                       // needs count to 200
                Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
                Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
                
                qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);


            }
        std::cout<<"The qDes[2] will be: "<<qDes[2]<<std::endl;
        double sin_joint0, sin_joint1, sin_joint2;
        // sin move 
        if(motiontime >=1000){
            sin_count++;
            sin_joint0 = 0.6* sin(3*M_PI*sin_count/1000.0);
            sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0] + sin_joint0;
            qDes[1] = sin_mid_q[1] + sin_joint1;
            qDes[2] = sin_mid_q[2] + sin_joint2;     
        }
        // std::cout<<"The sin_joint2 will be: "<<sin_joint2<<std::endl;
        // std::cout<<"The angle on the third joint will be: "<<qDes[2]<<std::endl;
        }
        // usleep(1000);
    return true;
}

bool A1_Interface::send_stop(){
    for (int i=0; i<12; i++){
        SendLowLCM.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowLCM.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
        SendLowLCM.motorCmd[i].Kp = 0;
        SendLowLCM.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
        SendLowLCM.motorCmd[i].Kd = 0;
        //int swap_i = swap_joint_indices(i);
        SendLowLCM.motorCmd[i].tau = 0;
    }
    safe.PositionLimit(SendLowLCM);
    safe.PowerProtect(SendLowLCM, RecvLowLCM, power_level);
    //roslcm.Send(SendLowLCM);
    // udp.SetSend(SendLowLCM);
    // udp.Send();
    return true;
}

bool A1_Interface::send_position_cmd(){
    // cmd uses order FR, FL, RR, RL
    float joint_torque[12] = {-0.65f, 0.0f,0.0f,
                              +0.65f, 0.0f,0.0f,
                              -0.65f, 0.0f,0.0f,
                              +0.65f, 0.0f,0.0f};
    for (int i=0; i<12; i++){
        // joint_torque[i] = (standing_pose[i] - RecvLowLCM.motorState[i].q) * Kp[i] - RecvLowLCM.motorState[i].dq * Kd[i]; // + robot_state.gravity_torque[i];
        // joint_torque[i] = (standing_pose[i] - RecvLowLCM.motorState[i].q) * Kp[i] - RecvLowLCM.motorState[i].dq * Kd[i] + robot_state.gravity_torque[i];
        // if(joint_torque[i] > 5.0f) joint_torque[i] = 5.0f;
        // if(joint_torque[i] < -5.0f) joint_torque[i] = -5.0f;
        SendLowLCM.motorCmd[i].mode = 0x0A;
        SendLowLCM.motorCmd[i].q =  qDes[i]; //; // shut down position control //UNITREE_LEGGED_SDK::PosStopF
        SendLowLCM.motorCmd[i].dq = 0; //UNITREE_LEGGED_SDK::VelStopF;
        SendLowLCM.motorCmd[i].Kp = Kp[i];
        SendLowLCM.motorCmd[i].Kd = Kd[i];
        //int swap_i = swap_joint_indices(i);
        SendLowLCM.motorCmd[i].tau = joint_torque[i];//torque[i];
    }
    safe.PositionLimit(SendLowLCM);
    safe.PowerProtect(SendLowLCM, RecvLowLCM, 10);
    //roslcm.Send(SendLowLCM);
    udp.SetSend(SendLowLCM);
    udp.Send();
    // std::cout<<"q 0 "<< RecvLowLCM.motorState[0].q <<std::endl;
    // std::cout<<"q 1 "<< RecvLowLCM.motorState[1].q <<std::endl;
    // std::cout<<"q 2 "<< RecvLowLCM.motorState[2].q <<std::endl;

    return true;
}

bool A1_Interface::send_torque_cmd(){
    // cmd uses order FR, FL, RR, RL
    a1_control.calculate_joint_torques(robot_state);
    SendLowLCM.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i=0; i<NUM_DOF; i++){
        // joint_torque[i] = (standing_pose[i] - RecvLowLCM.motorState[i].q) * Kp[i] - RecvLowLCM.motorState[i].dq * Kd[i]; // + robot_state.gravity_torque[i];
        // joint_torque[i] = (standing_pose[i] - RecvLowLCM.motorState[i].q) * Kp[i] - RecvLowLCM.motorState[i].dq * Kd[i] + robot_state.gravity_torque[i];
        // if(joint_torque[i] > 5.0f) joint_torque[i] = 5.0f;
        // if(joint_torque[i] < -5.0f) joint_torque[i] = -5.0f;
        SendLowLCM.motorCmd[i].mode = 0x0A;
        SendLowLCM.motorCmd[i].q =  UNITREE_LEGGED_SDK::PosStopF; //; // shut down position control //UNITREE_LEGGED_SDK::PosStopF
        SendLowLCM.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; //UNITREE_LEGGED_SDK::VelStopF;
        SendLowLCM.motorCmd[i].Kp = 0;
        SendLowLCM.motorCmd[i].Kd = 0;
        // swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
        int swap_i = swap_joint_indices(i);
        SendLowLCM.motorCmd[i].tau = robot_state.joint_torques(swap_i);//torque[i];
    }
    safe.PositionLimit(SendLowLCM);
    safe.PowerProtect(SendLowLCM, RecvLowLCM, 10);
    // std::cout << "Torque for FL: "<< SendLowLCM.motorCmd[0].tau<<", "<< SendLowLCM.motorCmd[1].tau<<", "<< SendLowLCM.motorCmd[2].tau<<std::endl;
    // std::cout << "Torque for FR: "<< SendLowLCM.motorCmd[3].tau<<", "<< SendLowLCM.motorCmd[4].tau<<", "<< SendLowLCM.motorCmd[5].tau<<std::endl;
    // std::cout << "Torque for RL: "<< SendLowLCM.motorCmd[6].tau<<", "<< SendLowLCM.motorCmd[7].tau<<", "<< SendLowLCM.motorCmd[8].tau<<std::endl;
    // std::cout << "Torque for RR: "<< SendLowLCM.motorCmd[9].tau<<", "<< SendLowLCM.motorCmd[10].tau<<", "<< SendLowLCM.motorCmd[11].tau<<std::endl;
    // roslcm.Send(SendLowLCM);
    udp.SetSend(SendLowLCM);
    udp.Send();
    // std::cout<<"q 0 "<< RecvLowLCM.motorState[0].q <<std::endl;
    // std::cout<<"q 1 "<< RecvLowLCM.motorState[1].q <<std::endl;
    // std::cout<<"q 2 "<< RecvLowLCM.motorState[2].q <<std::endl;

    return true;
}

bool A1_Interface::warm_start(){
    if(robot_state.root_rot_matrix.determinant()==1){
        return false;
    } else {
        return true;
    }
}



