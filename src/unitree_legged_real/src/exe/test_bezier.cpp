#include <iostream>

#include <cstdio>
#include <vector>

#include <Eigen/Dense>
#include "Bezier_curve.h"

int main(int, char**) {

    Eigen::Vector3d foot_pos_start(0.2,0.15,-0.33);
    Eigen::Vector3d foot_pos_final(0.25,0.16,-0.33);

    Bezier_Curve bs_utils;
    Eigen::MatrixXd interp_pos_rst(3,11);
    int i = 0;
    for (double t=0.0; t<=1.0; t += 0.1) {
        interp_pos_rst.col(i) = bs_utils.get_foot_pos_curve(t,foot_pos_start, 
        foot_pos_final,0);
        i++;
    }
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_default;
    // foot_pos_default<<0.25, 0.25, -0.17, -0.17, 0.15, -0.15, 0.15, -0.15, -0.33, -0.33, -0.33, -0.33;
    foot_pos_default<<1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
    std::cout<<foot_pos_default(2)<<std::endl;
    // std::cout<<"position interpolation"<<std::endl;
    // std::cout<<interp_pos_rst<<std::endl;
}