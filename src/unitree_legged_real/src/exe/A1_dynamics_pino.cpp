#include "A1_dynamics_pino.h"

A1_dynamics_pino::A1_dynamics_pino(){
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "Robot name: " <<model.name <<std::endl;
    pinocchio::Data robot_data(model);
    data = robot_data;
    std::cout << "Robot model nq: " <<model.nq <<std::endl;
    std::cout << "Robot model nv: " <<model.nv <<std::endl;
    // pinocchio::Data::Matrix6x jacobian(6, model.nv);
    pinocchio::Data::Matrix6x jacobian_init(6, model.nv);
    jacobian_init.setZero();
    jacobian = jacobian_init;
}

Eigen::VectorXd A1_dynamics_pino::forward_kinematics(Eigen::VectorXd q){
    pinocchio::forwardKinematics(model, data, q);
// Print out the placement of each joint of the kinematic tree
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;
    return q;
}

void A1_dynamics_pino::get_joint_jacobian(Eigen::VectorXd q){
    // FR_hip_joint FR_thigh_joint FR_calf_joint (4,5,6)
    // FL_hip_joint FL_thigh_joint FL_calf_joint (1,2,3)
    // RR_hip_joint RR_thigh_joint RR_calf_joint (10,11,12)
    // RL_hip_joint RL_thigh_joint RL_calf_joint (7,8,9)

    // pinocchio::Model::Index idx = model.existJointName("floating_base")?model.getJointId("FR_hip_joint"):(pinocchio::Model::Index)(model.njoints-1);
    // std::cout<<"Joint found: "<<model.getJointId("FR_hip_joint")<<std::endl; 

    // std::cout<<"The Index for Floating Base is: "<<idx<<std::endl;
    // pinocchio::forwardKinematics(model, data, q);
    // pinocchio::computeJointJacobians(model,data,q);
    Eigen::Matrix<double, 6, 12> J1;
    pinocchio::computeJointJacobians(model,data, q);
    jacobian = data.J;
    for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id){
        std::cout<< std::setw(24)<<std::left
                 << model.names[joint_id]<<": "<<std::endl
                 <<std::fixed <<std::setprecision(6)
                 <<std::endl;


        // pinocchio::computeJointJacobian(model,data,q,joint_id,J1);

    }
    // std::cout<<"rows of jacobian is: "<<jacobian.rows()<<std::endl;
    // std::cout<<"cols of jacobian is: "<<jacobian.cols()<<std::endl;
    // std::cout<<"The size of model nv is : "<<model.nv<<std::endl;
    // std::cout<<"The size of model nq is : "<<model.nq<<std::endl;
    // pinocchio::getJointJacobian(model,data, idx, pinocchio::WORLD, jacobian);
    // std::cout<<"Success: "<<std::endl;
    // std::cout<<jacobian<<std::endl;
}