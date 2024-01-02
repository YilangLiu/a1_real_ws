#include "A1_realsense.h"

A1_realsense::A1_realsense(ros::NodeHandle &_nh):
safe(UNITREE_LEGGED_SDK::LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
    nh = _nh;
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

    tag_x = MovingWindowFilter(5);
    tag_y = MovingWindowFilter(5);
    tag_z = MovingWindowFilter(5);
    a1_x = MovingWindowFilter(5);
    a1_y = MovingWindowFilter(5);
    a1_z = MovingWindowFilter(5);
    quat_w = MovingWindowFilter(5);
    quat_x = MovingWindowFilter(5);
    quat_y = MovingWindowFilter(5);
    quat_z = MovingWindowFilter(5);
    
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

    br_camera.sendTransform(tf::StampedTransform(transform_camera, 
                    ros::Time::now(), 
                    "base", 
                    "camera_link"));
}

void A1_realsense::pos_update(){
    static tf::TransformBroadcaster br;
    try{
        listener.lookupTransform("base", "tag_2", ros::Time(0), tag2_transform);
        listener.lookupTransform("base", "tag_link", ros::Time(0),quad_transform);

        robot_state.root_pos = Eigen::Vector3d(a1_x.CalculateAverage(tag2_transform.getOrigin().x()),
                                        a1_y.CalculateAverage(tag2_transform.getOrigin().y()),
                                        a1_z.CalculateAverage(tag2_transform.getOrigin().z()));
        
        tag2_trans.setOrigin(tf::Vector3(robot_state.root_pos(0),
                                         robot_state.root_pos(1),
                                         robot_state.root_pos(2)));

        // temporarily fix pitch and roll 
        tf::Matrix3x3 m(tag2_transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        yaw_quater.setRPY(0,0,yaw);
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

    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
        }
    br.sendTransform(tf::StampedTransform(tag2_trans, ros::Time::now(), "base", "trunk"));
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

        // Joint State 
        // FL, FR, RL, RR
        for (int i = 0; i < 12; ++i){
            int swap_i = swap_joint_indices(i);
            //   /
            //swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
            robot_state.joint_vel[i] = RecvLowLCM.motorState[swap_i].dq;
            robot_state.joint_pos[i] = RecvLowLCM.motorState[swap_i].q;  // standing_pose[i]; 
        }
        // std::cout<<"joint position is "<<robot_state.joint_pos<<std::endl;
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
        end_p.z = target_pos(2);     

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

        
    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
    }
    
    
} 