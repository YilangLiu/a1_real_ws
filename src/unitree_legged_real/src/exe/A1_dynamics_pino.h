#include "pinocchio/fwd.hpp"
#include <eigen3/Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include <iostream>
class A1_dynamics_pino{
public:
    // nq 12 
    // nv 12
    A1_dynamics_pino();
    ~A1_dynamics_pino() = default;
    //Eigen::Matrix3d jacobian(Eigen::Vector3d q);
    const std::string urdf_filename = std::string("./src/assets/a1/a1.urdf"); //src//unitree_ros/robots/a1_description/urdf/a1.urdf
    
    Eigen::VectorXd forward_kinematics(Eigen::VectorXd q);
    void get_joint_jacobian(Eigen::VectorXd q);
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::Data::Matrix6x jacobian;

};  