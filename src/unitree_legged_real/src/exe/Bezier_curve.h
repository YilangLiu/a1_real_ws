#include <vector>
#include <Eigen/Dense>
#include "A1Params.h"

class Bezier_Curve{
public:
    Bezier_Curve(){
        curve_constructed = false;
        bezier_degree = 4;
        coefficients = {1,4,6,4,1};
    }

    double bezier_curve(double t, const std::vector<double> &Points);
    Eigen::Vector3d get_foot_pos_curve(float t, 
                                       Eigen::Vector3d start, 
                                       Eigen::Vector3d end,
                                       double pitch_angle);

    Eigen::Vector3d get_foot_pos_curve_xyz(float t, 
                                       Eigen::Vector3d start, 
                                       Eigen::Vector3d end,
                                       double pitch_angle);

    bool reset_foot_pos_curve(){curve_constructed=false;}

private:
    bool curve_constructed;
    float bezier_degree;
    std::vector<double> coefficients;
};