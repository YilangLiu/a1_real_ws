#include "A1Params.h"
#include "Robot_State.h"
#include "Utils.h"

#define STATE_SIZE 18
#define MEAS_SIZE 28
#define PROCESS_NOISE_P_IMU 0.01 // covariance matrix for IMU position estimation 
#define PROCESS_NOISE_V_IMU 0.01 // covariance matrix for IMU velcity estimation 
#define PROCESS_NOISE_P_FOOT 0.01 // covariance matrix for foot position prediction 
#define SENSOR_NOISE_P_IMU_REL_FOOT 0.001 //covariance matrix for the foot position measurement 
#define SENSOR_NOISE_V_IMU_REL_FOOT 0.1 // covariance matrix for the foot velocity measurement 
#define SENSOR_NOISE_ZFOOT 0.001 // covariance matrix for the foot z axis measurement 


class A1_state_estimation{
public:
    A1_state_estimation();
    A1_state_estimation(bool assume_flat_ground_);
    void init_state(Robot_State& state);
    void update_estimation(Robot_State& state, double dt);
    bool is_initialized() {return filter_initialized;}
private:

    bool filter_initialized=false;

    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
    Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, 3> B; // control input transition
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

    // observation
    // 0 1 2   FL pos residual in the world frame
    // 3 4 5   FR pos residual in the world frame
    // 6 7 8   RL pos residual in the world frame
    // 9 10 11 RR pos residual in the world frame
    // 12 13 14 vel from FL
    // 15 16 17 vel from FR
    // 18 19 20 vel from RL
    // 21 22 23 vel from RR
    // 24 25 26 27 foot height
    Eigen::Matrix<double, MEAS_SIZE, 1> y; //  observation
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_y
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // estimation state observation noise
    

    Eigen::Matrix<double, 3, 3> eye3; // Identity
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; // Innovation (or pre-fit residual) covariance
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> L; // kalman gain

    double smooth_foot_force[4];
    double estimated_contacts[4];

    bool assume_flat_ground = false;
};
