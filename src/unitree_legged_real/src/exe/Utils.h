#ifndef A1_CPP_UTILS
#define A1_CPP_UTILS

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "A1Params.h"

class Utils{
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    //static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    static Eigen::Matrix3d Moore_Penrose_inverse(const Eigen::Matrix3d &mat);
    static double get_terrain_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
};
#endif