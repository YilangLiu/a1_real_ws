#ifndef A1_CPP_QP_MPC 
#define A1_CPP_QP_MPC

#include <vector>
#include <chrono>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>


#include "A1_dynamics.h"
#include "Robot_State.h"
#include "A1Params.h"
#include "Utils.h"

class A1_MPC{
    public:
    A1_MPC(Eigen::VectorXd &q_weigths_mpc, Eigen::VectorXd &r_weights_mpc);

    void init_mpc();

    void get_matrix_A(Eigen::Vector3d root_euler);

    void get_matrix_B(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia,
                      Eigen::Matrix3d root_rot_matrix, Eigen::Matrix<double, 3, NUM_LEG> foot_pos);

    void qp_formulation(Robot_State &state);

    void forward_euler_discretization(double dt);

    double mu;
    double fz_min;
    double fz_max;

    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weigths_mpc;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;
    
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;

    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B;

    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> QP_A;
    // Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> QP_B;
    Eigen::MatrixXd QP_B;

    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> QP_A_d;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> QP_B_d;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> QP_B_d_list;
    
    Eigen::SparseMatrix<double> hessian_sparse;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient;
    
    Eigen::SparseMatrix<double> linear_constraints; 

    Eigen::Matrix<double, MPC_CONSTRAINT_DIM*PLAN_HORIZON,1> lower_bound;
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM*PLAN_HORIZON,1> upper_bound;
};

#endif