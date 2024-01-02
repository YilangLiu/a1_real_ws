#include "A1_dynamics_pino.h"

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif

int main(int argc, char **argv){
A1_dynamics_pino a1;
// const std::string robot_name = std::string("src//unitree_ros/robots/a1_description/urdf/a1.urdf");
//A1_dynamics_pino a1(robot_name);
//Eigen::VectorXd c = pinocchio::randomConfiguration(a1.model);
// a1.forward_kinematics(a1.model,a1.data, c);
Eigen::Matrix<double, 4, 3> hip_offsets;
hip_offsets << 0.183, -0.047, 0,
                   0.183, -0.047, 0,
                   0.183, -0.047, 0,
                   0.183, -0.047, 0;
// std::cout<<hip_offsets <<std::endl;
// a1.get_joint_jacobian();
Eigen::Matrix<double, 600, 1> g_matrix;
// Eigen::VectorXd q = pinocchio::randomConfiguration(a1.model);
// a1.get_joint_jacobian(q);

// std::cout<<"random config"<< q<<std::endl;
// std::cout<<"jacobian"<<a1.jacobian.size()<<std::endl;
// Eigen::Matrix<double, 28, 18> C;
// Eigen::Matrix<double,3, 3> eye3;

// Eigen::Matrix<double, 18,1> x;
// x<<1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18;

// std::cout<<x.segment<3>(3)<<std::endl;
// eye3.setIdentity();

// C.setZero();

    
// for (int i=0; i<4; ++i){
//     C.block<3,3>(i*3,0) = -eye3;
//     C.block<3,3>(i*3,6+i*3) = eye3;
//     C.block<3,3>(4*3+i*3,3) = eye3;
//     C(4*6+i,6+i*3+2) = 1;
// }

// std::cout<< C <<std::endl;


return 0;
}