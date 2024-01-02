#include "A1_QP_MPC.h"

A1_MPC::A1_MPC(Eigen::VectorXd &q_weigths_mpc_, Eigen::VectorXd &r_weights_mpc_){
    // give weights for each control horizon
    
    // Eigen::MatrixXd QP_B(MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);
    QP_B.resize(MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);
    // std::cout<<"Passed QP_B Initial"<<std::endl;
    q_weigths_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
    for (int i=0; i<PLAN_HORIZON; ++i){
        q_weigths_mpc.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) = q_weigths_mpc_;
    }

    Q.diagonal() = 2 * q_weigths_mpc; 

    r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);
    for (int i=0; i<PLAN_HORIZON; ++i){
        r_weights_mpc.segment(i*NUM_DOF, NUM_DOF) = r_weights_mpc_; // ?
    }    
    R.diagonal() = 2*r_weights_mpc;

    mu = 0.3;
    fz_min = 0.0;
    fz_max = 0.0;
        linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);
    for (int i = 0; i< NUM_LEG * PLAN_HORIZON; ++i){
        linear_constraints.insert(5*i, 3*i) = 1;
        linear_constraints.insert(5*i, 3*i+2) = mu; // -mu * f_z << f_x

        linear_constraints.insert(5*i+1, 3*i) = 1;
        linear_constraints.insert(5*i+1, 3*i+2) = -mu; // f_x <= mu * f_z

        linear_constraints.insert(5*i+2, 3*i+1) = 1;
        linear_constraints.insert(5*i+2, 3*i+2) = mu; // -mu * f_z <= f_y

        linear_constraints.insert(5*i+3, 3*i+1) = 1;
        linear_constraints.insert(5*i+3, 3*i+2) = -mu;  // f_y <= mu * f_z

        linear_constraints.insert(5*i+4, 3*i+2) = 1;    // f_min <= f_z <= f_max
    }
}

void A1_MPC::init_mpc(){

    A.setZero();
    B.setZero();

    QP_A.setZero();
    QP_A_d.setZero();
    QP_B.setZero();
    QP_B_d.setZero();
    QP_B_d_list.setZero();
    
    gradient.setZero();
}


void A1_MPC::get_matrix_A(Eigen::Vector3d root_euler){
    Eigen::Matrix3d root_rot_vel;
    root_rot_vel << cos(root_euler[2]), sin(root_euler[2]),0,
                   -sin(root_euler[2]), cos(root_euler[2]),0,
                   0,0,1;

    A.block<3,3>(0,6) = root_rot_vel;
    A.block<3,3>(3,9) = Eigen::Matrix3d::Identity();
    A(11,NUM_DOF) = 1;
}

void A1_MPC::get_matrix_B(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia,
                      Eigen::Matrix3d root_rot_matrix, Eigen::Matrix<double, 3, NUM_LEG> foot_pos){
    Eigen::Matrix3d trunk_inertia_world;
    trunk_inertia_world = root_rot_matrix * a1_trunk_inertia * root_rot_matrix.transpose();
    for (int i=0;i<NUM_LEG;++i){
        B.block<3,3>(6, 3*i) = trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3,1>(0,i));
        B.block<3,3>(9, 3*i) = (1/robot_mass) * Eigen::Matrix3d::Identity();
    }
}

void A1_MPC::forward_euler_discretization(double dt){
    QP_A_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A *dt;
    QP_B_d = B * dt;

    // for (int i =0 ; i< PLAN_HORIZON; ++i){
    //     QP_B_d_list.block<MPC_STATE_DIM, NUM_DOF>(i * 13, 0) = QP_B_d;
    // }
}

void A1_MPC::qp_formulation(Robot_State &state){
    // std::cout<<"start QP Formulation"<<std::endl;
    for (int i =0; i <PLAN_HORIZON; ++i){
        if (i==0){
            QP_A.block<MPC_STATE_DIM, MPC_STATE_DIM>(0,0) = QP_A_d;
        } else{
            QP_A.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM*i,0) = QP_A.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM*(i-1),0) * QP_A_d;
        }

        for (int j = 0; j <i+1;++j){
            if (i - j == 0){
                // diagonal is B 
                QP_B.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM*i,NUM_DOF*j) = QP_B_d_list.block<MPC_STATE_DIM, NUM_DOF>(j*MPC_STATE_DIM,0);
            } else{
                // Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> temp_A = QP_A.block<MPC_STATE_DIM, MPC_STATE_DIM>((i-j-1)*MPC_STATE_DIM,0);
                // Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> temp_B = QP_B_d_list.block<MPC_STATE_DIM, NUM_DOF>(j*MPC_STATE_DIM,0);

                // std::cout<<"i = "<<i<<" and j = "<<j<<std::endl;
                // std::cout<<"shape of QP_A"<< temp_A.size()<<std::endl;
                // std::cout<<"shape of QP_B"<< temp_B.size()<<std::endl;

                QP_B.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM*i, NUM_DOF*j) = 
                        QP_A.block<MPC_STATE_DIM, MPC_STATE_DIM>((i-j-1)*MPC_STATE_DIM,0) * QP_B_d_list.block<MPC_STATE_DIM, NUM_DOF>(j*MPC_STATE_DIM,0);
            }
        }
    }

    
    Eigen::MatrixXd hessian(NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);
    // std::cout<<"Passed hessian Initial"<<std::endl;
    hessian = (QP_B.transpose()*Q*QP_B);
    hessian += R;
    hessian_sparse = hessian.sparseView(); // here

    // std::cout<<"Passed hessian_sparse"<<std::endl;
    
    Eigen::Matrix<double, MPC_STATE_DIM*PLAN_HORIZON, 1> g_matrix;
    g_matrix = QP_A * state.mpc_states;
    g_matrix -= state.mpc_states_desired;
    gradient = QP_B.transpose() * Q * g_matrix;

    // std::cout<<"Passed gradient"<<std::endl;

    fz_min = 0;
    fz_max = 180;

    Eigen::VectorXd lower_bound_i(MPC_CONSTRAINT_DIM);
    Eigen::VectorXd upper_bound_i(MPC_CONSTRAINT_DIM);

    for (int i =0;i<NUM_LEG; ++i){
        lower_bound_i.segment<5>(i*5) << 0, -OsqpEigen::INFTY, 0, -OsqpEigen::INFTY, fz_min * state.contacts[i];
        upper_bound_i.segment<5>(i*5) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, fz_max * state.contacts[i];
    }
    for (int i = 0; i<PLAN_HORIZON; ++i){
        lower_bound.segment<MPC_CONSTRAINT_DIM>(i*MPC_CONSTRAINT_DIM) = lower_bound_i;
        upper_bound.segment<MPC_CONSTRAINT_DIM>(i*MPC_CONSTRAINT_DIM) = upper_bound_i;
    }
}