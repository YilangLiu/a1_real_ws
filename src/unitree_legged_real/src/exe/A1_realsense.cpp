#include "A1_realsense.h"



using namespace UNITREE_LEGGED_SDK;

A1_realsense::A1_realsense(ros::NodeHandle &_nh):
safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
    nh = _nh;
    joint_pub_rviz = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    robot_state.reset();
    robot_state.load_ros_yaml(nh);

    foot_force_filters.setZero();
    foot_force_idx.setZero();
    foot_force_filters_sum.setZero();

    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;

    udp.InitCmdData(SendLowLCM);
    lcm_init();

    tag_x = MovingWindowFilter(10);
    tag_y = MovingWindowFilter(10);
    tag_z = MovingWindowFilter(10);
    a1_x = MovingWindowFilter(10);
    a1_y = MovingWindowFilter(10);
    a1_z = MovingWindowFilter(10);
    quat_w = MovingWindowFilter(10);
    quat_x = MovingWindowFilter(10);
    quat_y = MovingWindowFilter(10);
    quat_z = MovingWindowFilter(10);

    joint_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    joint_msg.position.resize(12);
    joint_msg.velocity.resize(12);
    joint_msg.effort.resize(12);

    Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
    Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
    qDes << 0, 0.9, -1.8, 
            0, 0.9, -1.8, 
            0, 0.9, -1.8, 
            0, 0.9, -1.8;

    SendLowLCM.levelFlag = LOWLEVEL;
    for (int i=0; i<12; i++){
        // joint_torque[i] = (standing_pose[i] - RecvLowLCM.motorState[i].q) * Kp[i] - RecvLowLCM.motorState[i].dq * Kd[i]; // + robot_state.gravity_torque[i];
        // joint_torque[i] = (standing_pose[i] - RecvLowLCM.motorState[i].q) * Kp[i] - RecvLowLCM.motorState[i].dq * Kd[i] + robot_state.gravity_torque[i];
        // if(joint_torque[i] > 5.0f) joint_torque[i] = 5.0f;
        // if(joint_torque[i] < -5.0f) joint_torque[i] = -5.0f;
        SendLowLCM.motorCmd[i].mode = 0x0A;
        SendLowLCM.motorCmd[i].q =  PosStopF; //; // shut down position control //UNITREE_LEGGED_SDK::PosStopF
        SendLowLCM.motorCmd[i].dq = VelStopF; //UNITREE_LEGGED_SDK::VelStopF;
        SendLowLCM.motorCmd[i].Kp = 0;
        SendLowLCM.motorCmd[i].Kd = 0;
        //int swap_i = swap_joint_indices(i);
        SendLowLCM.motorCmd[i].tau = 0;
}

    
    thread_ = std::thread(&A1_realsense::receive_motor_state, this);
}

void A1_realsense::camera_pos_reset(){
    static tf::TransformBroadcaster br_camera;
    tf::Transform transform_camera;
    transform_camera.setOrigin(tf::Vector3(default_camera_pos[0],
                                    default_camera_pos[1],
                                    default_camera_pos[2]));
    tf::Quaternion q_camera;
    q_camera.setRPY(0,M_PI/2,0);
    transform_camera.setRotation(q_camera);
    try{
        br_camera.sendTransform(tf::StampedTransform(transform_camera, 
                        ros::Time::now(), 
                        "base", 
                        "camera_link"));
    } 
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
    }
}

void A1_realsense::pos_update(){
    static tf::TransformBroadcaster br;
    try{
        // listener.lookupTransform("base", "tag_2", ros::Time(0), tag2_transform);
        listener.lookupTransform("base", "tag_6", ros::Time(0), tag6_transform);
        listener.lookupTransform("base", "tag_link", ros::Time(0),quad_transform);

        // robot_state.root_pos = Eigen::Vector3d(a1_x.CalculateAverage(tag2_transform.getOrigin().x()),
        //                                 a1_y.CalculateAverage(tag2_transform.getOrigin().y()),
        //                                 a1_z.CalculateAverage(tag2_transform.getOrigin().z()));

        //0.2 0 0.06
        robot_state.root_pos = Eigen::Vector3d(a1_x.CalculateAverage(tag6_transform.getOrigin().x()+0.2),
                                        a1_y.CalculateAverage(tag6_transform.getOrigin().y()),
                                        a1_z.CalculateAverage(tag6_transform.getOrigin().z()-0.06));
        
        tag2_trans.setOrigin(tf::Vector3(robot_state.root_pos(0),
                                         robot_state.root_pos(1),
                                         robot_state.root_pos(2)));

        // temporarily fix pitch and roll 
        tf::Matrix3x3 m(tag6_transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw_quater.setRPY(0,0,-M_PI);
        // tag2_trans.setRotation(tag2_transform.getRotation());
        tag2_trans.setRotation(yaw_quater);
        
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "CUBE";
        sphere_marker.type = visualization_msgs::Marker::CUBE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.header.frame_id = "tag_3";
        sphere_marker.pose.position.x = 0;
        sphere_marker.pose.position.y = 0;
        sphere_marker.pose.position.z = 0;
        sphere_marker.pose.orientation.x = 0.0;
        sphere_marker.pose.orientation.y = 0.0;
        sphere_marker.pose.orientation.z = 0.0;
        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.color.r = 0.0f;
        sphere_marker.color.g = 1.0f;
        sphere_marker.color.b = 0.0f;
        sphere_marker.color.a = 1.0;
        sphere_marker.scale.x = 0.05;
        sphere_marker.scale.y = 0.05;
        sphere_marker.scale.z = 0.05;
        
        marker_pub.publish(sphere_marker);

        br.sendTransform(tf::StampedTransform(tag2_trans, ros::Time::now(), "base", "trunk"));
        
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
        }
    
}


void A1_realsense::receive_motor_state(){
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);


    while (destruct ==false){
        udp.Recv();
        udp.GetRecv(RecvLowLCM);
        camera_pos_reset();
        pos_update();
        start_moving();
        // joy_msg_sub = nh.subscribe("/joy",1000, &A1_Interface::joystick_callback, this);
        // std::cout<<"1"<<std::endl;
        
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

        // std::cout<<"2"<<std::endl;
        

        // Joint State 
        // FL, FR, RL, RR
        for (int i = 0; i < 12; ++i){
            int swap_i = swap_joint_indices(i);
            //   /
            //swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
            robot_state.joint_vel[i] = RecvLowLCM.motorState[swap_i].dq;
            robot_state.joint_pos[i] = RecvLowLCM.motorState[swap_i].q; // standing_pose[i]; 
            // std::cout<<"3"<<std::endl;
            // joint_msg.position[i]    =  standing_pose[i]; // RecvLowLCM.motorState[i].q; 
            // joint_msg.velocity[i] = RecvLowLCM.motorState[i].dq;
            qDes[i] = RecvLowLCM.motorState[i].q;
        }
        // std::cout<<"joint position is "<<robot_state.joint_pos<<std::endl;
        //foot force 
        
        // joint_msg.header.stamp = ros::Time::now();
        // joint_pub_rviz.publish(joint_msg);

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

        // std::cout<<"4"<<std::endl;
        // camera_pos_reset();
        // pos_update();        
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

    }
    now = ros::Time::now();
    ros::Duration update_time = now - prev;
    prev = now;
    double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
    // sleep for interval_ms
    double interval_time = interval_ms / 1000.0;
    if (interval_time > update_time.toSec()) {
        ros::Duration(interval_time - update_time.toSec()).sleep();
    }
}

void A1_realsense::lcm_init(){
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

void A1_realsense::start_moving(){
    // start getting relative position from the foot to object 
    try{
        listener.lookupTransform("trunk", "tag_3", ros::Time(0), foot_sphere_rel);
        target_pos = Eigen::Vector3d(foot_sphere_rel.getOrigin().x(),
                                     foot_sphere_rel.getOrigin().y(),
                                     foot_sphere_rel.getOrigin().z());
        

        visualization_msgs::Marker line_marker;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "ARROW";
        line_marker.type = visualization_msgs::Marker::ARROW;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.header.frame_id = "trunk";
        
        line_marker.pose.position.x = 0;
        line_marker.pose.position.y = 0;
        line_marker.pose.position.z = 0;
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;

        geometry_msgs::Point start_p, end_p;
        start_p.x = 0;//robot_state.root_pos(0); 
        start_p.y = 0;//robot_state.root_pos(1); 
        start_p.z = 0;//robot_state.root_pos(2);  

        end_p.x = target_pos(0); 
        end_p.y = target_pos(1); 
        end_p.z = target_pos(2);     // relative to trunk 

        line_marker.points.push_back(start_p);
        line_marker.points.push_back(end_p);
        // line_marker.points.resize(2)
        // line_marker.points[0].x = target_pos(0);

        line_marker.color.r = 0.0f;
        line_marker.color.g = 1.0f;
        line_marker.color.b = 0.0f;
        line_marker.color.a = 1.0;
        line_marker.scale.x = 0.03;
        line_marker.scale.y = 0.03;
        line_marker.scale.z = 0.03;

        marker_pub.publish(line_marker);
        
        float bezier_t = rate_count/control_period;
        Eigen::Vector3d curr_FL_foot_pos = robot_state.foot_pos_rel.block<3,1>(0,0);
        desired_FL_foot_pos = FL_Leg_curve.get_foot_pos_curve_xyz(bezier_t,
                                        curr_FL_foot_pos,
                                        target_pos,
                                        0.0);
        // std::cout<<bezier_t <<std:endl;
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "SPHERE";
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.header.frame_id = "trunk";
        sphere_marker.pose.position.x = desired_FL_foot_pos(0);
        sphere_marker.pose.position.y = desired_FL_foot_pos(1);
        sphere_marker.pose.position.z = desired_FL_foot_pos(2);
        sphere_marker.pose.orientation.x = 0.0;
        sphere_marker.pose.orientation.y = 0.0;
        sphere_marker.pose.orientation.z = 0.0;
        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.color.r = 1.0f;
        sphere_marker.color.g = 0.0f;
        sphere_marker.color.b = 0.0f;
        sphere_marker.color.a = 1.0;
        sphere_marker.scale.x = 0.05;
        sphere_marker.scale.y = 0.05;
        sphere_marker.scale.z = 0.05;
        
        marker_pub.publish(sphere_marker);
        // after getting desired_FL we need to back calculate the angle

        joint_angle_FL = a1_dynamics_state.get_joint_angle_FL(desired_FL_foot_pos);

    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
    }
} 

void A1_realsense::start_counting(){
    if (rate_count<= control_period){
        rate_count +=1;

    } else{
        counting_finished = true;
        // rate_count = control_period;
    }
}

void A1_realsense::target_pos_update(){
        initial_counting += 1;
        if (initial_counting >3000){
            Initialized_flag = true;
        }
        if (!isnan(joint_angle_FL.norm())){
            // robot_state.joint_pos
            qDes.segment<3>(3) = joint_angle_FL;
            joint_msg.position[3]  =  qDes[3]; // joint_angle_FL(0); // robot_state.joint_pos[0]; 
            joint_msg.position[4]  =  qDes[4]; // joint_angle_FL(1); // robot_state.joint_pos[1];
            joint_msg.position[5]  =  qDes[5]; // joint_angle_FL(2); // robot_state.joint_pos[2];

            joint_msg.position[0]  =  qDes[0]; // robot_state.joint_pos[3]; // standing_pose[0];
            joint_msg.position[1]  =  qDes[1]; // robot_state.joint_pos[4]; // standing_pose[1];
            joint_msg.position[2]  =  qDes[2]; // robot_state.joint_pos[5]; // standing_pose[2];

            joint_msg.position[6]  =  qDes [6]; // robot_state.joint_pos[9]; // standing_pose[0];
            joint_msg.position[7]  =  qDes [7]; // robot_state.joint_pos[10]; // standing_pose[1];
            joint_msg.position[8]  =  qDes [8]; // robot_state.joint_pos[11]; // standing_pose[2];

            joint_msg.position[9]  =  qDes[9]; // robot_state.joint_pos[6]; // standing_pose[0];
            joint_msg.position[10] =  qDes[10];// robot_state.joint_pos[7]; // standing_pose[1];
            joint_msg.position[11] =  qDes[11];// robot_state.joint_pos[8]; // standing_pose[2];
            
            joint_msg.header.stamp = ros::Time::now();
            joint_pub_rviz.publish(joint_msg);
        } 
}

bool A1_realsense::send_position_cmd(){
    // cmd uses order FR, FL, RR, RL
    SendLowLCM.motorCmd[FR_0].tau = -0.65f;
    SendLowLCM.motorCmd[FL_0].tau = +0.65f;
    SendLowLCM.motorCmd[RR_0].tau = -0.65f;
    SendLowLCM.motorCmd[RL_0].tau = +0.65f;

    if (!isnan(joint_angle_FL.norm())){
        // qDes.segment<3>(3) = joint_angle_FL;

        SendLowLCM.motorCmd[FL_0].q = joint_angle_FL(0);
        SendLowLCM.motorCmd[FL_0].dq = 0;
        SendLowLCM.motorCmd[FL_0].Kp = Kp[0];
        SendLowLCM.motorCmd[FL_0].Kd = Kd[0];
        SendLowLCM.motorCmd[FL_0].tau = +0.65f;

        SendLowLCM.motorCmd[FL_1].q = joint_angle_FL(1);
        SendLowLCM.motorCmd[FL_1].dq = 0;
        SendLowLCM.motorCmd[FL_1].Kp = Kp[1];
        SendLowLCM.motorCmd[FL_1].Kd = Kd[1];
        SendLowLCM.motorCmd[FL_1].tau = 0.0f;

        SendLowLCM.motorCmd[FL_2].q =  joint_angle_FL(2);
        SendLowLCM.motorCmd[FL_2].dq = 0;
        SendLowLCM.motorCmd[FL_2].Kp = Kp[2];
        SendLowLCM.motorCmd[FL_2].Kd = Kd[2];
        SendLowLCM.motorCmd[FL_2].tau = 0.0f;


        safe.PositionLimit(SendLowLCM);
        safe.PowerProtect(SendLowLCM, RecvLowLCM, 10);
        //roslcm.Send(SendLowLCM);
        udp.SetSend(SendLowLCM);
        udp.Send();
        // std::cout<<"q 0 "<< RecvLowLCM.motorState[0].q <<std::endl;
        // std::cout<<"q 1 "<< RecvLowLCM.motorState[1].q <<std::endl;
        // std::cout<<"q 2 "<< RecvLowLCM.motorState[2].q <<std::endl;
    }
    return true;
}

