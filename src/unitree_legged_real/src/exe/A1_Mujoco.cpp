#include "A1_Mujoco.h"

A1_Mujoco::A1_Mujoco(ros::NodeHandle &_nh){
    nh = _nh;
    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/FL_hip_controller/command", 1);
    pub_joint_cmd[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/FL_thigh_controller/command", 1);
    pub_joint_cmd[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/FL_calf_controller/command", 1);

    pub_joint_cmd[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/FR_hip_controller/command", 1);
    pub_joint_cmd[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/FR_thigh_controller/command", 1);
    pub_joint_cmd[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/FR_calf_controller/command", 1);

    pub_joint_cmd[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/RL_hip_controller/command", 1);
    pub_joint_cmd[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/RL_thigh_controller/command", 1);
    pub_joint_cmd[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/RL_calf_controller/command", 1);

    pub_joint_cmd[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/RR_hip_controller/command", 1);
    pub_joint_cmd[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/RR_thigh_controller/command", 1);
    pub_joint_cmd[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/a1_mujoco/RR_calf_controller/command", 1);

    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/a1_mujoco/estimation_body_pose", 100);
    
    joint_pub_rviz = nh.advertise<sensor_msgs::JointState>("joint_states",100); 
    
    joint_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
                      
    joint_msg.position.resize(12);
    joint_msg.velocity.resize(12);
    joint_msg.effort.resize(12);

    // joint_msg.header.stamp = ros::Time::now();

    // joint_msg.position = {0};
    // joint_msg.velocity = {0};

    joint_msg_rviz.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    joint_msg_rviz.position.resize(12);
    joint_msg_rviz.velocity.resize(12);
    joint_msg_rviz.effort.resize(12);

    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    a1_control = robot_control() ;
    robot_state.reset();

    sub_joint_msg[0] = nh.subscribe("/a1_mujoco/FL_hip_controller/state", 2, &A1_Mujoco::FL_hip_state_callback, this);
    sub_joint_msg[1] = nh.subscribe("/a1_mujoco/FL_thigh_controller/state", 2, &A1_Mujoco::FL_thigh_state_callback, this);
    sub_joint_msg[2] = nh.subscribe("/a1_mujoco/FL_calf_controller/state", 2, &A1_Mujoco::FL_calf_state_callback, this);

    sub_joint_msg[3] = nh.subscribe("/a1_mujoco/FR_hip_controller/state", 2, &A1_Mujoco::FR_hip_state_callback, this);
    sub_joint_msg[4] = nh.subscribe("/a1_mujoco/FR_thigh_controller/state", 2, &A1_Mujoco::FR_thigh_state_callback, this);
    sub_joint_msg[5] = nh.subscribe("/a1_mujoco/FR_calf_controller/state", 2, &A1_Mujoco::FR_calf_state_callback, this);

    sub_joint_msg[6] = nh.subscribe("/a1_mujoco/RL_hip_controller/state", 2, &A1_Mujoco::RL_hip_state_callback, this);
    sub_joint_msg[7] = nh.subscribe("/a1_mujoco/RL_thigh_controller/state", 2, &A1_Mujoco::RL_thigh_state_callback, this);
    sub_joint_msg[8] = nh.subscribe("/a1_mujoco/RL_calf_controller/state", 2, &A1_Mujoco::RL_calf_state_callback, this);

    sub_joint_msg[9] = nh.subscribe("/a1_mujoco/RR_hip_controller/state", 2, &A1_Mujoco::RR_hip_state_callback, this);
    sub_joint_msg[10] = nh.subscribe("/a1_mujoco/RR_thigh_controller/state", 2, &A1_Mujoco::RR_thigh_state_callback, this);
    sub_joint_msg[11] = nh.subscribe("/a1_mujoco/RR_calf_controller/state", 2, &A1_Mujoco::RR_calf_state_callback, this);

    sub_gt_pose_msg = nh.subscribe("/torso_odom", 100, &A1_Mujoco::gt_pose_callback, this);
    sub_joy_msg = nh.subscribe("/joy", 1000, &A1_Mujoco::joy_callback, this);
    sub_imu_msg = nh.subscribe("/trunk_imu", 100, &A1_Mujoco::imu_callback, this);

    sub_foot_contact_msg[0] = nh.subscribe("/a1_mujoco/FL_foot_contact/the_force", 2, &A1_Mujoco::FL_foot_contact_callback, this);
    sub_foot_contact_msg[1] = nh.subscribe("/a1_mujoco/FR_foot_contact/the_force", 2, &A1_Mujoco::FR_foot_contact_callback, this);
    sub_foot_contact_msg[2] = nh.subscribe("/a1_mujoco/RL_foot_contact/the_force", 2, &A1_Mujoco::RL_foot_contact_callback, this);
    sub_foot_contact_msg[3] = nh.subscribe("/a1_mujoco/RR_foot_contact/the_force", 2, &A1_Mujoco::RR_foot_contact_callback, this);

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
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;
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
}

bool A1_Mujoco::get_stance_leg_forces(double dt){
    robot_state.stance_leg_forces = a1_control.stance_leg_control(robot_state, dt);
    return true;
}

bool A1_Mujoco::mujoco_data_received(){
    if (robot_state.root_rot_matrix.determinant()==0){
        return false;
    } else {
        return true;
    }
}

bool A1_Mujoco::main_update(double t, double dt){
    if (joy_cmd_exit) {
        return false;
    }
    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into a1_ctrl_states
    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }
     prev_joy_cmd_ctrl_state = 0; //joy_cmd_ctrl_state;

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
    robot_state.root_euler_desired[0] += joy_cmd_roll_rate * dt;
    robot_state.root_euler_desired[1] += joy_cmd_pitch_rate * dt;
    robot_state.root_euler_desired[2] += joy_cmd_yaw_rate * dt;
    // std::cout<<"joy_cmd_body_height is "<<joy_cmd_body_height<<std::endl;
    robot_state.root_pos_desired[2] =  joy_cmd_body_height;

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        // walking mode, in this mode the robot should execute gait
        robot_state.motion_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        robot_state.motion_mode = 0;
        robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
        robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
        robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
    } else {
        robot_state.motion_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (robot_state.motion_mode == 1) {
        if (robot_state.root_lin_vel_desired.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy
            robot_state.root_pos_desired.segment<2>(0) = robot_state.root_pos.segment<2>(0);
            robot_state.kp_linear.segment<2>(0).setZero();
        } else {
            robot_state.kp_linear(0) = robot_state.kp_linear_lock_x;
            robot_state.kp_linear(1) = robot_state.kp_linear_lock_y;
        }
    }

    a1_control.get_desired_foot_pos(robot_state, dt);
    a1_control.swing_leg_control(robot_state, dt);

    // state estimation
    if (!a1_estimation.is_initialized()) {
        a1_estimation.init_state(robot_state);
    } else {
        a1_estimation.update_estimation(robot_state, dt);
    }
    // nav_msgs::Odometry estimate_odom;
    // estimate_odom.header.stamp = ros::Time::now();
    // estimate_odom.pose.pose.position.x = robot_state.estimated_root_pos(0);
    // estimate_odom.pose.pose.position.y = robot_state.estimated_root_pos(1);
    // estimate_odom.pose.pose.position.z = robot_state.estimated_root_pos(2);

    // // make sure root_lin_vel is in world frame
    // estimate_odom.twist.twist.linear.x = robot_state.estimated_root_vel(0);
    // estimate_odom.twist.twist.linear.y = robot_state.estimated_root_vel(1);
    // estimate_odom.twist.twist.linear.z = robot_state.estimated_root_vel(2);

    // pub_estimated_pose.publish(estimate_odom);

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
    
    joint_msg.header.stamp = ros::Time::now();
    joint_pub_rviz.publish(joint_msg);
    // std::cout<<joint_msg<<std::endl;
    // std::cout<<"11111"<<std::endl;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // std::cout<<"root_pos"<< robot_state.root_pos << std::endl;
    transform.setOrigin(tf::Vector3(robot_state.root_pos(0),
                                    robot_state.root_pos(1),
                                    robot_state.root_pos(2)));
    tf::Quaternion q;
    q.setRPY(robot_state.root_euler(0),
            robot_state.root_euler(1),
            robot_state.root_euler(2));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "trunk"));
    return true;
}

bool A1_Mujoco::send_torque_cmd(){
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
        int swap_i = swap_joint_indices(i);
        low_cmd.motorCmd[i].tau = robot_state.joint_torques(swap_i, 0);
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
        // std::cout<< "torque is: "<< low_cmd.motorCmd[i].tau<<std::endl;
    }
    return true;
}   

void A1_Mujoco::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg){
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }

    // right updown
    joy_cmd_velx = joy_msg->axes[1] * JOY_CMD_VELX_MAX;

    // right horiz
    joy_cmd_vely = joy_msg->axes[0] * JOY_CMD_VELY_MAX;

    // left right BR stick controller
    joy_cmd_yaw_rate = joy_msg->axes[3] * JOY_CMD_YAW_MAX;

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

void A1_Mujoco::gt_pose_callback(const nav_msgs::Odometry::ConstPtr &odom){
    robot_state.root_quat = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                odom->pose.pose.orientation.x,
                                                odom->pose.pose.orientation.y,
                                                odom->pose.pose.orientation.z);    
    // std::cout << "sssssssssssssss"<<odom->pose.pose.orientation.w << odom->pose.pose.orientation.x<<odom->pose.pose.orientation.y<<odom->pose.pose.orientation.z<<std::endl;
    robot_state.root_rot_matrix = robot_state.root_quat.toRotationMatrix();
    robot_state.root_euler = Utils::quat_to_euler(robot_state.root_quat);
    
    double yaw_angle = robot_state.root_euler[2];

    robot_state.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // FL, FR, RL, RR
    // a1_dynamics_state.get_foot_position_base_frame(robot_state.joint_pos);

    // robot_state.foot_pos_rel.block<3,1>(0,0) = a1_dynamics_state.res_FL;
    // robot_state.foot_pos_rel.block<3,1>(0,1) = a1_dynamics_state.res_FR;
    // robot_state.foot_pos_rel.block<3,1>(0,2) = a1_dynamics_state.res_RL;
    // robot_state.foot_pos_rel.block<3,1>(0,3) = a1_dynamics_state.res_RR;
    for (int i = 0; i < NUM_LEG; ++i) {
        // robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.analytical_leg_jacobian(
        //         robot_state.joint_pos.segment<3>(3*i),std::pow(-1,i));
        robot_state.foot_pos_rel.block<3, 1>(0, i) = a1_dynamics_state.fk(
                robot_state.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        robot_state.j_foot.block<3,3>(3*i, 3*i) = a1_dynamics_state.jac(
                robot_state.joint_pos.segment<3>(3*i),rho_opt_list[i], rho_fix_list[i]);

        Eigen::Matrix3d tmp_mtx = robot_state.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = robot_state.joint_vel.segment<3>(3 * i);
        robot_state.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        robot_state.foot_pos_abs.block<3, 1>(0, i) = robot_state.root_rot_matrix * robot_state.foot_pos_rel.block<3, 1>(0, i);
        robot_state.foot_vel_abs.block<3, 1>(0, i) = robot_state.root_rot_matrix * robot_state.foot_vel_rel.block<3, 1>(0, i);

        robot_state.foot_pos_world.block<3, 1>(0, i) = robot_state.foot_pos_abs.block<3, 1>(0, i) + robot_state.root_pos;
        robot_state.foot_vel_world.block<3, 1>(0, i) = robot_state.foot_vel_abs.block<3, 1>(0, i) + robot_state.root_lin_vel;
    }
    //std::cout<<"foot_pos_abs"<<robot_state.foot_pos_abs<<std::endl;
}

void A1_Mujoco::imu_callback(const sensor_msgs::Imu::ConstPtr &imu){
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
}

void A1_Mujoco::FL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[0] = a1_joint_state.q;
    robot_state.joint_vel[0] = a1_joint_state.dq;
    joint_msg.position[0] = a1_joint_state.q;
    joint_msg.velocity[0] = a1_joint_state.dq;
}


void A1_Mujoco::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[1] = a1_joint_state.q;
    robot_state.joint_vel[1] = a1_joint_state.dq;
    joint_msg.position[1] = a1_joint_state.q;
    joint_msg.velocity[1] = a1_joint_state.dq;
    //std::cout<<joint_msg.position[4]<<std::endl;
}

void A1_Mujoco::FL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[2] = a1_joint_state.q;
    robot_state.joint_vel[2] = a1_joint_state.dq;
    joint_msg.position[2] = a1_joint_state.q;
    joint_msg.velocity[2] = a1_joint_state.dq;
}

void A1_Mujoco::FR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[3] = a1_joint_state.q;
    robot_state.joint_vel[3] = a1_joint_state.dq;
    joint_msg.position[3] = a1_joint_state.q;
    joint_msg.velocity[3] = a1_joint_state.dq;
}


void A1_Mujoco::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[4] = a1_joint_state.q;
    robot_state.joint_vel[4] = a1_joint_state.dq;
    joint_msg.position[4] = a1_joint_state.q;
    joint_msg.velocity[4] = a1_joint_state.dq;
}

void A1_Mujoco::FR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[5] = a1_joint_state.q;
    robot_state.joint_vel[5] = a1_joint_state.dq;
    joint_msg.position[5] = a1_joint_state.q;
    joint_msg.velocity[5] = a1_joint_state.dq;
}

void A1_Mujoco::RL_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[6] = a1_joint_state.q;
    robot_state.joint_vel[6] = a1_joint_state.dq;
    joint_msg.position[6] = a1_joint_state.q;
    joint_msg.velocity[6] = a1_joint_state.dq;
}


void A1_Mujoco::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[7] = a1_joint_state.q;
    robot_state.joint_vel[7] = a1_joint_state.dq;
    joint_msg.position[7] = a1_joint_state.q;
    joint_msg.velocity[7] = a1_joint_state.dq;
}

void A1_Mujoco::RL_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[8] = a1_joint_state.q;
    robot_state.joint_vel[8] = a1_joint_state.dq;
    joint_msg.position[8] = a1_joint_state.q;
    joint_msg.velocity[8] = a1_joint_state.dq;
}

void A1_Mujoco::RR_hip_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[9] = a1_joint_state.q;
    robot_state.joint_vel[9] = a1_joint_state.dq;
    joint_msg.position[9] = a1_joint_state.q;
    joint_msg.velocity[9] = a1_joint_state.dq;
}


void A1_Mujoco::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[10] = a1_joint_state.q;
    robot_state.joint_vel[10] = a1_joint_state.dq;
    joint_msg.position[10] = a1_joint_state.q;
    joint_msg.velocity[10] = a1_joint_state.dq;
}

void A1_Mujoco::RR_calf_state_callback(const unitree_legged_msgs::MotorState &a1_joint_state){
    robot_state.joint_pos[11] = a1_joint_state.q;
    robot_state.joint_vel[11] = a1_joint_state.dq;
    joint_msg.position[11] = a1_joint_state.q;
    joint_msg.velocity[11] = a1_joint_state.dq;
}

void A1_Mujoco::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[0] = force.wrench.force.z;
}

void A1_Mujoco::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[1] = force.wrench.force.z;
}

void A1_Mujoco::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[2] = force.wrench.force.z;
}

void A1_Mujoco::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force){
    robot_state.foot_force[3] = force.wrench.force.z;
}
