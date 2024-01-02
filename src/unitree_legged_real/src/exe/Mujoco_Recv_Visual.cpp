// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include "A1_Mujoco.h"
#include <thread>
using namespace std::chrono;


Eigen::Rand::P8_mt19937_64 urng{ 42 };

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
// mjvCamera cam;                      // abstract camera
// mjvOption opt;                      // visualization options
// mjvScene scn;                       // abstract scene
// mjrContext con;                     // custom GPU context
char* model_xml;
int num_state;
int num_joint; 

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

//ROS node init 
Eigen::VectorXd q_pos;
Eigen::VectorXd q_vel;
mjtNum target_pos[12];
mjtNum curr_joint_pos[19];
mjtNum curr_joint_vel[18];
mjtNum torque[12];
unitree_legged_msgs::MotorState joint_state_pub[12];

ros::Subscriber sub_joint_torques[12];
ros::Publisher pub_joint_pos[12];
ros::Publisher pub_imu;
ros::Publisher pub_odm;
ros::Publisher pub_contat_forces[4];

int FL_Calf_body_id = 7;
int FR_Calf_body_id = 4;
int RL_Calf_body_id = 13;
int RR_Calf_body_id = 10;
// keyboard callback
// void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
//     // backspace: reset simulation
//     if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
//         mj_resetData(m, d);
//         mj_forward(m, d);
//     }
// }

// // mouse button callback
// void mouse_button(GLFWwindow* window, int button, int act, int mods) {
//     // update button state
//     button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
//     button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
//     button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

//     // update mouse position
//     glfwGetCursorPos(window, &lastx, &lasty);
// }

// // mouse move callback
// void mouse_move(GLFWwindow* window, double xpos, double ypos) {
//     // no buttons down: nothing to do
//     if (!button_left && !button_middle && !button_right) {
//         return;
//     }

//     // compute mouse displacement, save
//     double dx = xpos - lastx;
//     double dy = ypos - lasty;
//     lastx = xpos;
//     lasty = ypos;

//     // get current window size
//     int width, height;
//     glfwGetWindowSize(window, &width, &height);

//     // get shift key state
//     bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
//                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

//     // determine action based on mouse button
//     mjtMouse action;
//     if (button_right) {
//     action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
//     } else if (button_left) {
//     action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
//     } else {
//     action = mjMOUSE_ZOOM;
//     }

//     // move camera
//     mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
// }


// // scroll callback
// void scroll(GLFWwindow* window, double xoffset, double yoffset) {
//     // emulate vertical mouse motion = 5% of window height
//     mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
// }


void FL_hip_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    // std::cout<<"running "<<std::endl;
    torque[0] = state->tau;
}   

void FL_thigh_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[1] = state->tau;
}  

void FL_calf_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[2] = state->tau;
}  

void FR_hip_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[3] = state->tau;
}   

void FR_thigh_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[4] = state->tau;
}  

void FR_calf_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[5] = state->tau;
}  

void RL_hip_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[6] = state->tau;
}   

void RL_thigh_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[7] = state->tau;
}  

void RL_calf_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[8] = state->tau;
}  

void RR_hip_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[9] = state->tau;
}   

void RR_thigh_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[10] = state->tau;
}  

void RR_calf_torque_callback(const unitree_legged_msgs::MotorCmd::ConstPtr &state){
    torque[11] = state->tau;
}  

double* SensorByName(const mjModel* m, const mjData* d,
                     const std::string& name) {
  int id = mj_name2id(m, mjOBJ_SENSOR, name.c_str());
  if (id == -1) {
    std::cerr << "sensor \"" << name << "\" not found.\n";
    return nullptr;
  } else {
    return d->sensordata + m->sensor_adr[id];
  }
}

// main function
int main(int argc, char **argv) {
    model_xml = "src//assets/unitree_mujoco/data/a1/xml/a1.xml";
    char error[1000] = "Could not load binary model";

    m = mj_loadXML(model_xml, NULL, error, 1000);
    d = mj_makeData(m);

    num_state = m->nq + m->nv;
    num_joint = m->nu;

    ros::init(argc, argv,"Mujoco_Sub");

    ros::NodeHandle nh;
    

    
    // ros::Subscriber sub = nh.subscribe("/hardware_a1/joint_foot",100,robot_joint_callback);
    sub_joint_torques[0] = nh.subscribe("/a1_mujoco/FR_hip_controller/command",2,FR_hip_torque_callback);
    sub_joint_torques[1] = nh.subscribe("/a1_mujoco/FR_thigh_controller/command",2,FR_thigh_torque_callback);
    sub_joint_torques[2] = nh.subscribe("/a1_mujoco/FR_calf_controller/command",2,FR_calf_torque_callback);
    
    sub_joint_torques[3] = nh.subscribe("/a1_mujoco/FL_hip_controller/command",2,FL_hip_torque_callback);
    sub_joint_torques[4] = nh.subscribe("/a1_mujoco/FL_thigh_controller/command",2,FL_thigh_torque_callback);
    sub_joint_torques[5] = nh.subscribe("/a1_mujoco/FL_calf_controller/command",2,FL_calf_torque_callback);
    
    sub_joint_torques[6] = nh.subscribe("/a1_mujoco/RR_hip_controller/command",2,RR_hip_torque_callback);
    sub_joint_torques[7] = nh.subscribe("/a1_mujoco/RR_thigh_controller/command",2,RR_thigh_torque_callback);
    sub_joint_torques[8] = nh.subscribe("/a1_mujoco/RR_calf_controller/command",2,RR_calf_torque_callback);
    
    sub_joint_torques[9] = nh.subscribe("/a1_mujoco/RL_hip_controller/command",2,RL_hip_torque_callback);
    sub_joint_torques[10] = nh.subscribe("/a1_mujoco/RL_thigh_controller/command",2,RL_thigh_torque_callback);
    sub_joint_torques[11] = nh.subscribe("/a1_mujoco/RL_calf_controller/command",2,RL_calf_torque_callback);
    
    pub_joint_pos[0] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/FR_hip_controller/state",1);
    pub_joint_pos[1] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/FR_thigh_controller/state",1);
    pub_joint_pos[2] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/FR_calf_controller/state",1);

    pub_joint_pos[3] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/FL_hip_controller/state",1);
    pub_joint_pos[4] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/FL_thigh_controller/state",1);
    pub_joint_pos[5] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/FL_calf_controller/state",1);

    pub_joint_pos[6] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/RR_hip_controller/state",1);
    pub_joint_pos[7] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/RR_thigh_controller/state",1);
    pub_joint_pos[8] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/RR_calf_controller/state",1);

    pub_joint_pos[9] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/RL_hip_controller/state",1);
    pub_joint_pos[10] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/RL_thigh_controller/state",1);
    pub_joint_pos[11] = nh.advertise<unitree_legged_msgs::MotorState>("/a1_mujoco/RL_calf_controller/state",1);

    pub_odm = nh.advertise<nav_msgs::Odometry>("/torso_odom",100);
    pub_imu = nh.advertise<sensor_msgs::Imu>("/trunk_imu",100);

    pub_contat_forces[0] = nh.advertise<geometry_msgs::WrenchStamped>("/a1_mujoco/FL_foot_contact/the_force",1);
    pub_contat_forces[1] = nh.advertise<geometry_msgs::WrenchStamped>("/a1_mujoco/FR_foot_contact/the_force",1);
    pub_contat_forces[2] = nh.advertise<geometry_msgs::WrenchStamped>("/a1_mujoco/RL_foot_contact/the_force",1);
    pub_contat_forces[3] = nh.advertise<geometry_msgs::WrenchStamped>("/a1_mujoco/RR_foot_contact/the_force",1);


    // // init GLFW
    // if (!glfwInit()) {
    //     mju_error("Could not initialize GLFW");
    // }

    // // create window, make OpenGL context current, request v-sync
    // GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    // glfwMakeContextCurrent(window);
    // glfwSwapInterval(1);

    // // initialize visualization data structures
    // mjv_defaultCamera(&cam);
    // mjv_defaultOption(&opt);
    // mjv_defaultScene(&scn);q_pos = Eigen::VectorXd::Zero(12);
    // mjr_defaultContext(&con);

    // // create scene and context
    // mjv_makeScene(m, &scn, 2000);
    // mjr_makeContext(m, &con, mjFONTSCALE_150);

    // // install GLFW mouse and keyboard callbacks
    // glfwSetKeyCallback(window, keyboard);
    // glfwSetCursorPosCallback(window, mouse_move);
    // glfwSetMouseButtonCallback(window, mouse_button);
    // glfwSetScrollCallback(window, scroll);

    // run main loop, target real-time simulation and 60 fps rendering
    q_pos = Eigen::VectorXd::Zero(12);
    // std::thread main_thread([&]() {
    // while (!glfwWindowShouldClose(window)) {
    //     // advance interactive simulation for 1/60 sec
    //     //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //     //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //     //  Otherwise add a cpu timer and exit this loop when it is time to render.
    //     mjtNum simstart = d->time;
    //     while (d->time - simstart < 1.0/60.0) {
    //         auto start = high_resolution_clock::now(); // get the starting time
    //         // std::cout<<q_pos.data()<<std::endl;
    //         // PID_Controller(m, d);

            // mju_copy(d->ctrl, torque, m->nu);

            // std::cout<<"run 4"<<std::endl;
            // std::cout<< "contact_force_FL z is: " << contact_force_FL[0] <<std::endl;
            // std::cout<< "contact_force_FR z is: " << contact_force_FR[0] <<std::endl;
            // std::cout<< "contact_force_RL z is: " << contact_force_RL[0] <<std::endl;
            // std::cout<< "contact_force_RR z is: " << contact_force_RR[0] <<std::endl;

            // std::cout<< "acc 1 is: " << acc[0] <<std::endl;
            // std::cout<< "acc 2  is: " << acc[1] <<std::endl;
            // std::cout<< "acc 3 is: " << acc[2] <<std::endl;

            // std::cout<< "contact_force_FR x is: " << contact_force_FR[0] <<std::endl;
            // std::cout<< "contact_force_FR y  is: " << contact_force_FR[1] <<std::endl;
            // std::cout<< "contact_force_FR z is: " << contact_force_FR[2] <<std::endl;
            // std::cout<<"Target Position: "<<target_pos[0]<<std::endl;
            // mj_step(m, d);

            
    //         auto stop = high_resolution_clock::now(); // get the stopping time
    //         auto duration = duration_cast<microseconds>(stop - start); // calculate the elapsed time
    //         // std::cout << "Elapsed time: " << duration.count() << " microseconds" << std::endl;
    //     }

    //     // get framebuffer viewport
    //     mjrRect viewport = {0, 0, 0, 0};
    //     glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    //     // update scene and render
    //     mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    //     mjr_render(viewport, &scn, &con);

    //     // swap OpenGL buffers (blocking call due to v-sync)
    //     glfwSwapBuffers(window);

    //     // process pending GUI events, call GLFW callbacks
    //     glfwPollEvents();
    // }
    // });

    std::thread main_thread([&]() {
        while (ros::ok){
          mju_copy(d->ctrl, torque, m->nu);
          mj_step(m, d);
        }
    });

    std::thread pub_thred([&]() {
    while(ros::ok){
        // mjtNum simstart = d->time;
        // while (d->time - simstart < 1.0/60.0) {
        mju_copy(curr_joint_pos, d->qpos, m->nq);
        mju_copy(curr_joint_vel, d->qvel, m->nv);

        joint_state_pub[0].q = curr_joint_pos[7];  //FR
        joint_state_pub[0].dq = curr_joint_vel[6]; //FR
        
        joint_state_pub[1].q = curr_joint_pos[8];  //FR
        joint_state_pub[1].dq = curr_joint_vel[7]; //FR

        joint_state_pub[2].q = curr_joint_pos[9];  //FR
        joint_state_pub[2].dq = curr_joint_vel[8]; //FR

        joint_state_pub[3].q = curr_joint_pos[10];  //FL
        joint_state_pub[3].dq = curr_joint_vel[9]; //FL

        joint_state_pub[4].q = curr_joint_pos[11];  //FL
        joint_state_pub[4].dq = curr_joint_vel[10]; //FL

        joint_state_pub[5].q = curr_joint_pos[12];  //FL
        joint_state_pub[5].dq = curr_joint_vel[11]; //FL

        joint_state_pub[6].q = curr_joint_pos[13];  //RR
        joint_state_pub[6].dq = curr_joint_vel[12]; //RR

        joint_state_pub[7].q = curr_joint_pos[14];  //RR
        joint_state_pub[7].dq = curr_joint_vel[13]; //RR

        joint_state_pub[8].q = curr_joint_pos[15];  //RR
        joint_state_pub[8].dq = curr_joint_vel[14]; //RR

        joint_state_pub[9].q = curr_joint_pos[16];  //RL
        joint_state_pub[9].dq = curr_joint_vel[15]; //RL

        joint_state_pub[10].q = curr_joint_pos[17];  //RL
        joint_state_pub[10].dq = curr_joint_vel[16]; //RL

        joint_state_pub[11].q = curr_joint_pos[18];  //RL
        joint_state_pub[11].dq = curr_joint_vel[17]; //RL

        pub_joint_pos[0].publish(joint_state_pub[0]);
        pub_joint_pos[1].publish(joint_state_pub[1]);
        pub_joint_pos[2].publish(joint_state_pub[2]);

        pub_joint_pos[3].publish(joint_state_pub[3]);
        pub_joint_pos[4].publish(joint_state_pub[4]);
        pub_joint_pos[5].publish(joint_state_pub[5]);

        pub_joint_pos[6].publish(joint_state_pub[6]);
        pub_joint_pos[7].publish(joint_state_pub[7]);
        pub_joint_pos[8].publish(joint_state_pub[8]);

        pub_joint_pos[9].publish(joint_state_pub[9]);
        pub_joint_pos[10].publish(joint_state_pub[10]);
        pub_joint_pos[11].publish(joint_state_pub[11]);

        double* gyro_data = SensorByName(m,d,"Body_Gyro");
        double* acc_data = SensorByName(m,d,"Body_Acc");
        double* quat_data = SensorByName(m,d,"Body_Quat");

        mjtNum acc[3];
        mjtNum gyro[3];
        mjtNum quat[4];
        mjtNum contact_force_FL[6];
        mjtNum contact_force_FR[6];
        mjtNum contact_force_RL[6];
        mjtNum contact_force_RR[6];
        mju_copy3(gyro, gyro_data);
        mju_copy3(acc, acc_data);
        mju_copy4(quat,quat_data);

        geometry_msgs::WrenchStamped pub_contact_FL;
        geometry_msgs::WrenchStamped pub_contact_FR;
        geometry_msgs::WrenchStamped pub_contact_RL;
        geometry_msgs::WrenchStamped pub_contact_RR;

        ros::Time curr_time = ros::Time::now();
        sensor_msgs::Imu imu_data;
        nav_msgs::Odometry odm_data;
        imu_data.header.stamp = curr_time;
        // imu_data.header.frame_id = 'imu_link';
        imu_data.linear_acceleration.x = acc[0];
        imu_data.linear_acceleration.y = acc[1];
        imu_data.linear_acceleration.z = acc[2];

        imu_data.angular_velocity.x = gyro[0];
        imu_data.angular_velocity.y = gyro[1];
        imu_data.angular_velocity.z = gyro[2];

        // std::cout<<"run 1"<<std::endl;
        pub_imu.publish(imu_data);

        odm_data.header.stamp = curr_time;
        // odm_data.header.frame_id = "base";
        // odm_data.child_frame_id = "trunk";
        odm_data.pose.pose.orientation.w = quat[0];
        odm_data.pose.pose.orientation.x = quat[1];
        odm_data.pose.pose.orientation.y = quat[2];
        odm_data.pose.pose.orientation.z = quat[3];
        // std::cout<<"run 2"<<std::endl;
        pub_odm.publish(odm_data);

        // the x-axis belongs to the contact frame (by definition the contact normal always points along the x-axis
        // https://github.com/google-deepmind/mujoco/issues/679
        // for (int i=0; i<d->ncon; ++i){
        //     const char* name1;
        //     const char* name2;
        //     int geom1_id = d->contact[i].geom1;
        //     int geom2_id = d->contact[i].geom2;
        //     int body1_id = m->geom_bodyid[geom1_id];
        //     int body2_id = m->geom_bodyid[geom2_id];
        //     name1 = mj_id2name(m, mjOBJ_BODY, body1_id);
        //     name2 = mj_id2name(m, mjOBJ_BODY, body2_id);
        //     mj_contactForce(m,d,i,contact_force_FR);
        //     std::cout<<"             "<<std::endl;
        //     std::cout<<"contact number is "<<i<<std::endl;
        //     std::cout << "name_1: "<< std::string(name1)<<std::endl;
        //     std::cout << "name_2: "<< std::string(name2)<<std::endl;
        //     std::cout<< "contact_force x is: " << contact_force_FR[0] <<std::endl;
        //     std::cout<< "contact_force y  is: " << contact_force_FR[1] <<std::endl;
        //     std::cout<< "contact_force z is: " << contact_force_FR[2] <<std::endl;
        // }
        // FL_Calf body id = 7
        // FR_Calf body id = 4
        // RL_Calf body id = 13
        // RR_Calf body id = 10
        // int body1_id = m->geom_bodyid[12];
        // int body2_id = m->geom_bodyid[20];
        // int body3_id = m->geom_bodyid[28];
        // int body4_id = m->geom_bodyid[36];
        // std::cout<< "FR: "<<body1_id<<std::endl;
        // std::cout<< "FL: "<<body2_id<<std::endl;
        // std::cout<< "RR: "<<body3_id<<std::endl;
        // std::cout<< "RL: "<<body4_id<<std::endl;


        mj_contactForce(m,d,FL_Calf_body_id,contact_force_FL);
        mj_contactForce(m,d,FR_Calf_body_id,contact_force_FR);
        mj_contactForce(m,d,RL_Calf_body_id,contact_force_RL);
        mj_contactForce(m,d,RR_Calf_body_id,contact_force_RR);
        
        pub_contact_FL.wrench.force.z = contact_force_FL[0];
        pub_contact_FR.wrench.force.z = contact_force_FR[0];
        pub_contact_RL.wrench.force.z = contact_force_RL[0];
        pub_contact_RR.wrench.force.z = contact_force_RR[0];

        //std::cout<<"run 3"<<std::endl;
        pub_contat_forces[0].publish(pub_contact_FL);
        pub_contat_forces[1].publish(pub_contact_FR);
        pub_contat_forces[2].publish(pub_contact_RL);
        pub_contat_forces[3].publish(pub_contact_RR);


        // std::cout<<"curr joint pos "<< curr_joint_pos[18]<<std::endl;
        // std::cout<<"torque 0 is "<< torque[0] <<std::endl;
        // }
    }

    });
    ros::AsyncSpinner spinner(12);
    spinner.start();
    // //free visualization storage
    // mjv_freeScene(&scn);
    // mjr_freeContext(&con);

    // // free MuJoCo model and data
    // mj_deleteData(d);
    // mj_deleteModel(m);

    main_thread.join();
    pub_thred.join();
    // // terminate GLFW (crashes with Linux NVidia drivers)
    // #if defined(__APPLE__) || defined(_WIN32)
    //     glfwTerminate();
    // #endif

    return 0;
}
