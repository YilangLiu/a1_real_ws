#include "Bezier_curve.h"

double Bezier_Curve::bezier_curve(double t, const std::vector<double> &Points){
    double res = 0;
    for (int i=0; i<=bezier_degree; i++){
        res += coefficients[i] * std::pow(t,i) * std::pow(1-t, bezier_degree-i)*Points[i];
    }
    return res;
}

Eigen::Vector3d Bezier_Curve::get_foot_pos_curve(float t, Eigen::Vector3d start, 
                                                Eigen::Vector3d end, double pitch_angle){
Eigen::Vector3d foot_pos_desired;

std::vector<double> X_Axis{start(0),start(0), end(0), end(0), end(0)};
std::vector<double> Y_Axis{start(1),start(1), end(1), end(1), end(1)};
std::vector<double> Z_Axis{start(2),start(2), end(2), end(2), end(2)};

foot_pos_desired(0) = bezier_curve(t, X_Axis);
foot_pos_desired(1) = bezier_curve(t, Y_Axis);
Z_Axis[1] += FOOT_SWING_CLEARANCE1;
Z_Axis[2] += FOOT_SWING_CLEARANCE2 + 0.5*sin(pitch_angle);

foot_pos_desired(2) = bezier_curve(t, Z_Axis);

return foot_pos_desired;
                                       }

Eigen::Vector3d Bezier_Curve::get_foot_pos_curve_xyz(float t, Eigen::Vector3d start, 
                                                Eigen::Vector3d end, double pitch_angle){
Eigen::Vector3d foot_pos_desired;

Eigen::Vector3d intermediate_points;

intermediate_points(0) = end(0);
intermediate_points(1) = start(1);
intermediate_points(2) = end(2);

std::vector<double> X_Axis{start(0), intermediate_points(0), intermediate_points(0), intermediate_points(0), end(0)};
std::vector<double> Y_Axis{start(1), intermediate_points(1), intermediate_points(1), intermediate_points(1), end(1)};
std::vector<double> Z_Axis{start(2), intermediate_points(2), intermediate_points(2), intermediate_points(2), end(2)};

// std::vector<double> X_Axis{start(0),start(0), end(0), end(0), end(0)};
// std::vector<double> Y_Axis{start(1),start(1), end(1), end(1), end(1)};
// std::vector<double> Z_Axis{start(2),start(2), end(2), end(2), end(2)};

foot_pos_desired(0) = bezier_curve(t, X_Axis);
foot_pos_desired(1) = bezier_curve(t, Y_Axis);
// Z_Axis[1] += FOOT_SWING_CLEARANCE1;
// Z_Axis[2] += FOOT_SWING_CLEARANCE2 + 0.5*sin(pitch_angle);

foot_pos_desired(2) = bezier_curve(t, Z_Axis);

return foot_pos_desired;
                                       }