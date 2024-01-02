#ifndef A1_CPP_DYNAMICS
#define A1_CPP_DYNAMICS

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

class A1_dynamics{
    public:
    A1_dynamics();
    ~A1_dynamics() = default;
    const int RHO_OPT_SIZE = 3;
    const int RHO_FIX_SIZE = 5;

    //Eigen::Matrix3d jacobian(Eigen::Vector3d q);
    void get_foot_position_base_frame(Eigen::Matrix<double, 12, 1> motor_angle);
    void foot_position_in_hip_frame(Eigen::Matrix<double, 12, 1> motor_angle);
    Eigen::Matrix3d analytical_leg_jacobian(Eigen::Vector3d q, int leg_id);
    Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
    Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);
    void autoFunc_fk_derive(const double in1[3], const double in2[3], const double in3[5], double p_bf[3]);
    void autoFunc_d_fk_dq(const double in1[3], const double in2[3], const double in3[5], double jacobian[9]);

    double l_up = 0.2;
    double l_low = 0.2;
    double leg_distance, eff_swing, off_x_hip, off_z_hip, off_y_hip;
    Eigen::Vector3d res_FL, res_FR, res_RL, res_RR;
    Eigen::Vector3d hip_offset_FL, hip_offset_FR, hip_offset_RL,hip_offset_RR;
    Eigen::Vector3d offset_x_y_z;
    std::vector<Eigen::Vector3d> foot_position_rel;
    std::vector<Eigen::Vector3d> foot_position_base;
};  
#endif