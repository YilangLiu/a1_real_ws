#include "A1_control.h"

robot_control::robot_control(){
    init_control();
}

void robot_control::init_control(){
    mpc_wait = 0;
    terrain_angle_filter = MovingWindowFilter(100);
    // std::cout<<"################################"<<std::endl;
    // std::cout<<"Control Contact Initiated!"<<std::endl;
    // std::cout<<"control initialized"<<std::endl;
    for (int i = 0; i < NUM_LEG; ++i) {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}

void robot_control::get_desired_foot_pos(Robot_State &state, double dt){
    state.counter +=1;
    if(state.motion_mode==0){
        // if the motion mode is 0, meaning the robot is in the stance mode
         //in stance mode, all legs have contacts 
        for (int i=0; i<NUM_LEG; ++i){state.expected_contacts[i]=true;}
    } else{
        // if the robot is in walking mode, then we will start counting 
        for (int i=0;i<NUM_LEG; ++i){
            state.gait_count(i) += state.gait_count_speed(i); 
            state.gait_count(i) = std::fmod(state.gait_count(i),state.gait_count_cycle);
            if (state.gait_count(i) <= state.swing_leg_cycle){
                state.expected_contacts[i] = true;
            } else{
                state.expected_contacts[i] = false;
            }
        }
    }

    Eigen::Vector3d lin_vel_world = state.root_lin_vel; //linear velocity in the world frame
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose()*lin_vel_world; 

    state.foot_pos_desired_rel = state.foot_pos_default;
    state.foot_pos_desired_rel_next = state.foot_pos_default;
    for(int i=0;i<NUM_LEG; ++i){
        double dx, Railbert_x, Capture_Point_x;
        double dy, Railbert_y, Capture_Point_y;

        Railbert_x = (state.swing_leg_cycle/state.gait_count_speed(i))* state.control_dt/2.0 * state.root_lin_vel_desired(0);
        Capture_Point_x = std::sqrt(std::abs(state.foot_pos_default(2))/9.8)*(lin_vel_rel(0)-state.root_lin_vel_desired(0));

        Railbert_y = (state.swing_leg_cycle/state.gait_count_speed(i))* state.control_dt/2.0 * state.root_lin_vel_desired(1);
        Capture_Point_y = std::sqrt(std::abs(state.foot_pos_default(2))/9.8)*(lin_vel_rel(1)-state.root_lin_vel_desired(1));

        dx = Railbert_x + Capture_Point_x;
        dy = Railbert_y + Capture_Point_y;

        if(dx<-FOOT_DELTA_X_LIMIT){dx = -FOOT_DELTA_X_LIMIT;}
        if(dx> FOOT_DELTA_X_LIMIT){dx = FOOT_DELTA_X_LIMIT;}
        if(dy<-FOOT_DELTA_Y_LIMIT){dy = -FOOT_DELTA_X_LIMIT;}
        if(dy> FOOT_DELTA_Y_LIMIT){dy = FOOT_DELTA_X_LIMIT;}

        state.foot_pos_desired_rel(0,i) += dx;
        state.foot_pos_desired_rel(1,i) += dy;
        
        state.foot_pos_desired_rel_next(0,i) = state.foot_pos_desired_rel(0,i) + 3*dx;
        state.foot_pos_desired_rel_next(1,i) = state.foot_pos_desired_rel(1,i) + 3*dy;

        state.foot_pos_desired_abs.block<3,1>(0,i) = state.root_rot_matrix * state.foot_pos_desired_rel.block<3,1>(0,i);
        state.foot_pos_desired_world.block<3,1>(0,i) = state.root_pos + state.foot_pos_desired_abs.block<3,1>(0,i);
    }

}



void robot_control::swing_leg_control(Robot_State &state, double dt){
    state.joint_torques.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> current_foot_pos;
    Eigen::Matrix<double, 3, NUM_LEG> current_foot_vel;
    Eigen::Matrix<double, 1, NUM_LEG> bezier_t;
    bezier_t.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> target_foot_pos;
    Eigen::Matrix<double, 3, NUM_LEG> target_foot_vel;
    target_foot_pos.setZero();
    target_foot_vel.setZero();

    Eigen::Matrix<double, 3, NUM_LEG> diff_pos;
    Eigen::Matrix<double, 3, NUM_LEG> diff_vel;

    Eigen::Matrix<double, 3, NUM_LEG> swing_foot_forces; // in robot frame!!!!!

    for (int i=0; i < NUM_LEG; ++i){
        current_foot_pos.block<3,1>(0,i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3,1>(0,i);
        if (state.gait_count(i) <= state.swing_leg_cycle){
            // For the stance leg, the robot leg should be 
            bezier_t(i) = 0.0;
            state.foot_pos_start.block<3,1>(0,i) = current_foot_pos.block<3,1>(0,i);
        } else {
            bezier_t(i) = float(state.gait_count(i)-state.swing_leg_cycle)/float(state.swing_leg_cycle);
        }
        target_foot_pos.block<3,1>(0,i) = leg_bezier_curve[i].get_foot_pos_curve(bezier_t(i),
                                                                                state.foot_pos_start.block<3,1>(0,i),
                                                                                state.foot_pos_desired_rel.block<3,1>(0,i),
                                                                                0.0);
        current_foot_vel.block<3,1>(0,i) = (current_foot_pos.block<3,1>(0,i)-state.prev_foot_pos_rel.block<3,1>(0,i)) / dt;
        state.prev_foot_pos_rel.block<3,1>(0,i) = current_foot_pos.block<3,1>(0,i);

        target_foot_vel.block<3,1>(0,i) = (target_foot_pos.block<3,1>(0,i) - state.prev_foot_pos_target.block<3,1>(0,i))/dt;
        state.prev_foot_pos_target.block<3,1>(0,i) = target_foot_pos.block<3,1>(0,i);

        diff_pos.block<3,1>(0,i) = target_foot_pos.block<3,1>(0,i) - current_foot_pos.block<3,1>(0,i);
        diff_vel.block<3,1>(0,i) = target_foot_vel.block<3,1>(0,i) - current_foot_vel.block<3,1>(0,i);

        swing_foot_forces.block<3,1>(0,i) = diff_pos.block<3,1>(0,i).cwiseProduct(state.kp_swing_foot.block<3,1>(0,i))+
                                            diff_vel.block<3,1>(0,i).cwiseProduct(state.kd_swing_foot.block<3,1>(0,i));
        // std::cout<<"kp is: "<<state.kp_swing_foot <<std::endl;
        // std::cout<<"kd is: "<<state.kd_swing_foot <<std::endl;

    }
    // state.foot
    Eigen::Vector3d target_foot_position_rviz;
    target_foot_position_rviz.setZero();

    bool last_contacts[NUM_LEG];
    for (int i=0; i<NUM_LEG; ++i){
        if (state.gait_count(i) <= state.swing_leg_cycle*1.5){
            state.early_contacts[i] = false;
        }
        if (!state.expected_contacts[i] && 
            (state.gait_count(i) > state.swing_leg_cycle *1.5) && (state.foot_force(i)>FOOT_FORCE_LOW)){
                state.early_contacts[i] = true;
            }
        last_contacts[i]  = state.contacts[i];
        state.contacts[i] = state.expected_contacts[i] || state.early_contacts[i];

        if (state.contacts[i]==true){
            state.foot_pos_recent_contact.block<3,1>(0,i)<< recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0,i)),
                                                            recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1,i)),
                                                            recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2,i));
        }           
    }
    state.swing_leg_forces = swing_foot_forces;
    // start generating the foot plan
    // std::cout<<"######################################"<<std::endl;
    // std::cout<<"Here"<<std::endl;
    state.foot_plans.header.frame_id = "trunk";
    state.foot_plans.header.stamp = ros::Time::now();
    state.foot_plans.feet.clear();

    for (int i=0; i<NUM_LEG; ++i){
        // std::cout<<"#################################"<<std::endl;
        // std::cout<< "num leg is "<< i <<std::endl;
        if (state.gait_count(i) <= state.swing_leg_cycle){
            int foot_holds_num = 1;
            state.foot_plans.feet[i].footholds.clear();
            state.foot_plans.feet[i].footholds.resize(foot_holds_num);
            // std::cout<<"#################################"<<std::endl;
            // std::cout<< "At this point in contact : "<< i <<std::endl;
            state.foot_plans.feet[i].footholds[0].position.x =  target_foot_pos.coeff(0,i);
            state.foot_plans.feet[i].footholds[0].position.y =  target_foot_pos.coeff(1,i);
            state.foot_plans.feet[i].footholds[0].position.z =  target_foot_pos.coeff(2,i);
        } else{
            // std::cout<<"#################################"<<std::endl;
            // std::cout<< "At this point not in contact : "<< i <<std::endl;
            int foot_holds_num = (2* state.swing_leg_cycle - state.gait_count(i)) + state.swing_leg_cycle;
            state.foot_plans.feet[i].footholds.clear();
            state.foot_plans.feet[i].footholds.resize(foot_holds_num);
            // std::cout<<"foot_holds num "<<i<<" is: "<< foot_holds_num<<std::endl;
            for (int j=0; j<foot_holds_num; ++j){

                if (j < (2* state.swing_leg_cycle - state.gait_count(i))){
                    // std::cout<< "In : "<< j << " not in contact" <<std::endl;
                    float curr_bezier_t = (state.gait_count(i) - state.swing_leg_cycle + j)/state.swing_leg_cycle;
                    // std::cout<< "start get foot pos curve " <<std::endl;
                    target_foot_position_rviz = leg_bezier_curve[i].get_foot_pos_curve(curr_bezier_t,
                                                                                    state.foot_pos_start.block<3,1>(0,i),
                                                                                    state.foot_pos_desired_rel.block<3,1>(0,i),
                                                                                    0.0);
                    // std::cout<< "get foot pos curve " <<std::endl;
                    // std::cout<< "foot pos curve is: "<<target_foot_position_rviz<<std::endl;
                    // std::cout<<"position x "<< target_foot_position_rviz(0)<<std::endl;
                    // std::cout<<"position y "<< target_foot_position_rviz(1)<<std::endl;
                    // std::cout<<"position z "<< target_foot_position_rviz(2)<<std::endl;

                    state.foot_plans.feet[i].footholds[j].position.x =  target_foot_position_rviz(0);
                    state.foot_plans.feet[i].footholds[j].position.y =  target_foot_position_rviz(1);
                    state.foot_plans.feet[i].footholds[j].position.z =  target_foot_position_rviz(2);  
                } else{
                    float curr_bezier_t = (state.gait_count(i) - 2*state.swing_leg_cycle + j) / state.swing_leg_cycle;
                    // std::cout<<"curr bezier_t is: "<<curr_bezier_t<<std::endl;
                    target_foot_position_rviz = leg_bezier_curve[i].get_foot_pos_curve(curr_bezier_t,
                                                                                    state.foot_pos_desired_rel.block<3,1>(0,i),
                                                                                    state.foot_pos_desired_rel_next.block<3,1>(0,i),
                                                                                    0.0);
                    state.foot_plans.feet[i].footholds[j].position.x =  target_foot_position_rviz(0);
                    state.foot_plans.feet[i].footholds[j].position.y =  target_foot_position_rviz(1);
                    state.foot_plans.feet[i].footholds[j].position.z =  target_foot_position_rviz(2);
                }                         
            }
        }
    }
}

Eigen::Matrix<double, 3, NUM_LEG> robot_control::stance_leg_control(Robot_State &state, double dt){
    Eigen::Matrix<double, 3, NUM_LEG> stance_leg_forces;
    Eigen::Vector3d coeff = terrain_estimation(state);
    Eigen::Vector3d flat_ground_coeff;
    flat_ground_coeff<< 0,0,1;
    double terrain_angle=0;

    if (state.root_pos[2] >0.1){
        terrain_angle = 
            terrain_angle_filter.CalculateAverage(Utils::get_terrain_angle(flat_ground_coeff, coeff));
        } else{
            terrain_angle = 0;
        }
    if (terrain_angle > 0.5){ terrain_angle = 0.5;}
    if (terrain_angle <-0.5) {terrain_angle = -0.5;}

    double z_diff = state.foot_pos_recent_contact(2,0) + state.foot_pos_recent_contact(2,1) -
                    state.foot_pos_recent_contact(2,2) - state.foot_pos_recent_contact(2,3);
    
    // if (z_diff >0.05){
    //     state.root_euler_desired[1] = -terrain_angle;
    // } else {
    //     state.root_euler_desired[1] = terrain_angle;
    // }
    // Eigen::Vector3d root_pos_mpc = state.root_pos;
    // state.root_pos[2] = -state.root_pos[2];
    // std::cout<<" Q_weights: "<< state.Q_weights<<std::endl;
    A1_MPC a1_mpc = A1_MPC(state.Q_weights, state.R_weights);

    // std::cout<<"Q_weights are: "<< state.Q_weights<<std::endl;
    // std::cout<<"R_weights are: "<< state.R_weights<<std::endl;
    a1_mpc.init_mpc();
    state.mpc_states << state.root_euler[0], state.root_euler[1],state.root_euler[2],
                        // root_pos_mpc[0], root_pos_mpc[1], root_pos_mpc[2],
                        state.root_pos[0], state.root_pos[1], state.root_pos[2],
                        state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
                        state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
                        -9.8;
    // std::cout<<"mpc_states root pos "<<state.root_pos<<std::endl;
    // std::cout<<"our contacts: "<<"FR: "<<state.contacts[0]<<
    //                             " FL: "<<state.contacts[1]<<
    //                             " RL: "<<state.contacts[2]<<
    //                             " RR: "<<state.contacts[3]<<std::endl;
    // std::cout<< "mpc states: "<<"root_ang_vel_x: "<< state.root_ang_vel[0] <<
    //                             "root_ang_vel_y: "<< state.root_ang_vel[1] <<
    //                             "root_ang_vel_z: "<< state.root_ang_vel[2] <<std::endl;

    // std::cout<< "mpc states: "<<"root_lin_vel_x: "<< state.root_lin_vel[0] <<
    //                             "root_lin_vel_y: "<< state.root_lin_vel[1] <<
    //                             "root_lin_vel_z: "<< state.root_lin_vel[2] <<std::endl;
    // double mpc_dt = 0.0025; 0.05
    double mpc_dt = 0.025;
    state.root_lin_vel_desired_world = state.root_rot_matrix * state.root_lin_vel_desired;

    // std::cout<<"root_lin_vel "<< "x: "<<state.root_lin_vel[0]<< " y: "<<state.root_lin_vel[1]<<std::endl;
    // std::cout<<"root_lin_vel_desired_world "<<"x: "<< state.root_lin_vel_desired_world[0]<<" y: "<< state.root_lin_vel_desired_world[1]<<std::endl;
    // std::cout<<"euler : "<< state.root_euler<<std::endl;
    // std::cout<<"angular velocity desired: "<<state.root_ang_vel_desired[2]<<std::endl;
    // std::cout<<"euler desired: "<< state.root_euler_desired<<std::endl;
    // std::cout<<"root_pos : "<< state.root_pos<<std::endl;
    for (int i=0; i<PLAN_HORIZON; ++i){
        state.mpc_states_desired.segment(i*MPC_STATE_DIM, MPC_STATE_DIM) <<
            state.root_euler_desired[0], state.root_euler_desired[1], state.root_euler[2] + state.root_ang_vel_desired[2] * mpc_dt * (i+1), //  state.root_euler_desired[2]
            state.root_pos[0]+ state.root_lin_vel_desired_world[0] * mpc_dt * (i+1), // + state.root_lin_vel_desired_world[0] * mpc_dt * (i+1)
            state.root_pos[1]+ state.root_lin_vel_desired_world[1] * mpc_dt * (i+1), // + state.root_lin_vel_desired_world[1] * mpc_dt * (i+1)
            // root_pos_mpc[0] + state.root_lin_vel_desired_world[0] * mpc_dt * (i+1),
            // root_pos_mpc[1] + state.root_lin_vel_desired_world[1] * mpc_dt * (i+1),
            state.root_pos_desired[2],// state.root_pos_desired[2] 
            state.root_ang_vel_desired[0], state.root_ang_vel_desired[1], state.root_ang_vel_desired[2],
            state.root_lin_vel_desired_world[0], state.root_lin_vel_desired_world[1], 0,
            -9.8;
            // std::cout<<"pos desired: "<< state.root_pos[0]<<" "<<state.root_pos[1]<<" "<<state.root_pos_desired[2]<<std::endl;
            // std::cout<<"Plan horizon "<<i <<" has root pos x: "<< state.root_pos[0] + state.root_lin_vel_desired_world[0] * mpc_dt * (i+1)<<
            //                        " has root pos y: "<< state.root_pos[1] + state.root_lin_vel_desired_world[1] * mpc_dt * (i+1)<<
            //                         " has root pos z: "  <<  state.root_pos_desired[2]<<std::endl;
            // std::cout<<"Plan horizon "<<i<<" root_ang_vel_x: "<< state.root_ang_vel_desired[0] <<
            //                     "root_ang_vel_y: "<< state.root_ang_vel_desired[1] <<
            //                     "root_ang_vel_z: "<< state.root_ang_vel_desired[2] <<std::endl;
            // std::cout<< " root_lin_vel_x: "<< state.root_lin_vel_desired_world[0] <<
            //             "root_lin_vel_y: "<< state.root_lin_vel_desired_world[1] <<
            //             "root_lin_vel_z: "<< 0 <<std::endl;
    }
    // std::cout<< "root_rot_matrix_robot_control: "<<state.root_rot_matrix <<std::endl;
    // std::cout<< "root pos desired 2: "<<state.root_pos_desired[2]<<std::endl;
    // std::cout<<"mpc_states_desired is "<<state.mpc_states_desired<<std::endl;
    a1_mpc.get_matrix_A(state.root_euler);
    // std::cout<<"robot foot pos rel :" << state.foot_pos_abs<<std::endl;
    // std::cout<<"robot rot matrix: "<<state.root_rot_matrix<<std::endl;
    for (int i =0; i<PLAN_HORIZON; ++i){
        // std::cout<< "root_rot_matrix: "<<state.root_rot_matrix<<std::endl;
        // std::cout<< "foot_pos: "<<state.foot_pos_abs<<std::endl;
        a1_mpc.get_matrix_B(state.robot_mass, state.a1_trunk_inertia,
                            state.root_rot_matrix,state.foot_pos_abs); // should I use foot_pos_rel?
        a1_mpc.forward_euler_discretization(mpc_dt);
        a1_mpc.QP_B_d_list.block<MPC_STATE_DIM, NUM_DOF>(i*MPC_STATE_DIM,0) = a1_mpc.QP_B_d;
    }
    // std::cout<< "QP_B_d: "<<a1_mpc.QP_B_d_list<<std::endl;
    a1_mpc.qp_formulation(state);

    if(!solver.isInitialized()){
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
        solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
        solver.data()->setLinearConstraintsMatrix(a1_mpc.linear_constraints);
        solver.data()->setHessianMatrix(a1_mpc.hessian_sparse);
        solver.data()->setGradient(a1_mpc.gradient);
        solver.data()->setLowerBound(a1_mpc.lower_bound);
        solver.data()->setUpperBound(a1_mpc.upper_bound);
        solver.initSolver();
    } else{
        solver.updateHessianMatrix(a1_mpc.hessian_sparse);
        solver.updateGradient(a1_mpc.gradient);
        solver.updateLowerBound(a1_mpc.lower_bound);
        solver.updateUpperBound(a1_mpc.upper_bound);
    }
    solver.solve();
    Eigen::VectorXd solution = solver.getSolution();
    for (int i =0; i<NUM_LEG; ++i){
        if(!isnan(solution.segment<3>(i*3).norm())){
            stance_leg_forces.block<3,1>(0,i) = state.root_rot_matrix.transpose() * solution.segment<3>(i*3);
            // std::cout<<"stance leg forces for  "<< i <<" is: "<< stance_leg_forces.block<3,1>(0,i)<< std::endl;

        }
    }
    // std::cout<<"root euler is: "<< state.root_euler<<std::endl;
    // std::cout<<"root position z is: "<< state.root_pos[2]<<std::endl;
    // std::cout<<"root position desited z is: "<<state.root_pos_desired[2]<<std::endl;
    // std::cout<< "linear velocity desired is "<< state.root_lin_vel_desired_world<<std::endl;
    return stance_leg_forces;
}

Eigen::Vector3d robot_control::terrain_estimation(Robot_State &state){
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d coeff;

    W.block<NUM_LEG,1>(0,0).setOnes();
    W.block<NUM_LEG,2>(0,1) = state.foot_pos_recent_contact.block<2,NUM_LEG>(0,0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1,NUM_LEG>(2,0).transpose();

    a = Utils::Moore_Penrose_inverse(W.transpose()*W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    coeff << a[1], a[2], -1;
    return coeff;
}

void robot_control::calculate_joint_torques(Robot_State &state){
    Eigen::Matrix<double, NUM_DOF,1> joint_torques;
    joint_torques.setZero();
    mpc_wait++;
    if (mpc_wait<10){state.joint_torques = joint_torques;}
    else{
        for (int i=0; i<NUM_LEG; ++i){
            Eigen::Matrix3d jacobian = state.j_foot.block<3,3>(3*i, 3*i);
            if(state.contacts[i]==true){
                joint_torques.segment<3>(i*3) = jacobian.transpose()*-state.stance_leg_forces.block<3,1>(0,i);
                // std::cout<< "for leg "<< i << " contact force from MPC is: "<< state.stance_leg_forces.block<3,1>(0,i)<<std::endl;
            } else{
                Eigen::Vector3d swing_leg_forces = state.km_foot.cwiseProduct(state.swing_leg_forces.block<3,1>(0,i));
                joint_torques.segment<3>(i*3) = jacobian.lu().solve(swing_leg_forces);
            }
        }
        joint_torques += state.gravity_torque;

        for (int i=0; i<12; ++i){
            if(!isnan(joint_torques[i])){
                state.joint_torques[i] = joint_torques[i];
            }
        }
    }
}