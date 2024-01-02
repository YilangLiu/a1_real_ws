#include "A1_state_estimation.h"

A1_state_estimation::A1_state_estimation(){
    eye3.setIdentity();
    C.setZero();
    
    for (int i=0; i<NUM_LEG; ++i){
        C.block<3,3>(i*3,0) = -eye3;
        C.block<3,3>(i*3,6+i*3) = eye3;
        C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;
        C(NUM_LEG*6+i,6+i*3+2) = 1;
    }

    //set Q and R matrix 

    // setup noise covariance matrix Q 
    Q.setIdentity();
    Q.block<3,3>(0,0) = PROCESS_NOISE_P_IMU * eye3; // base position noise covariance  
    Q.block<3,3>(3,3) = PROCESS_NOISE_V_IMU * eye3; // base velocity noise covariance
    for (int i=0; i<NUM_LEG;++i){
        Q.block<3,3>(6+3*i, 6+3*i) = PROCESS_NOISE_P_FOOT * eye3; //foot position noise covariance 
    }

    R.setIdentity();
    for (int i=0; i<NUM_LEG; ++i){
        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_P_IMU_REL_FOOT * eye3;
        R.block<3,3>(NUM_LEG*3 +i*3, NUM_LEG*3+i*3) = SENSOR_NOISE_V_IMU_REL_FOOT * eye3;
        R(NUM_LEG*6 + i, NUM_LEG*6+ i) = SENSOR_NOISE_ZFOOT; 
    }

    // linearized dynamics matrix xdot = Ax + Bu
    A.setIdentity();
    B.setZero();
    assume_flat_ground = true;
}

A1_state_estimation::A1_state_estimation(bool assume_flat_ground_):A1_state_estimation(){
    assume_flat_ground = assume_flat_ground_;
    if (assume_flat_ground ==false){
        for(int i=0;i<NUM_LEG;++i){
            R(NUM_LEG*6+i, NUM_LEG*6+i) = 1e5;
        }
    }
}

void A1_state_estimation::init_state(Robot_State& state){
    P.setIdentity(); // state covariance matrix 
    P = P * 3;

    x.setZero();
    x.segment<3>(0) = Eigen::Vector3d(0,0,0.09); // set robot initial base position relative to the world frame I 
    //  set forward kinematic leg position relative to the world frame
    for (int i =0; i<NUM_LEG;++i){
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3,1>(0,i);
        x.segment<3>(3*i+6) = state.root_rot_matrix * fk_pos + x.segment<3>(0);
    }   

    filter_initialized = true;
}


void A1_state_estimation::update_estimation(Robot_State& state, double dt){
    A.block<3,3>(0,3) = dt*eye3;
    B.block<3,3>(3,0) = dt*eye3;

    // get acceleration for the estimated state (gravity and IMU Acc) 
    Eigen::Vector3d u = state.root_rot_matrix * state.imu_acc + Eigen::Vector3d(0,0,-9.81);
    // std::cout<<state.root_rot_matrix<< std::endl;
    
    //  std::cout<<"IMU_X: "<< state.imu_acc[0] << "IMU_Y: "<< state.imu_acc[1]<<std::endl;

    if (state.motion_mode==0){
        for (int i=0;i <NUM_LEG;++i) estimated_contacts[i] = 1.0;
    }
    else{
        for (int i=0;i<NUM_LEG;++i){
            estimated_contacts[i] = std::min(std::max(state.foot_force(i) / (100.0 - 0.0), 0.0), 1.0);
            // std::cout<< "contacts at "<<i <<" is "<< estimated_contacts[i]<<std::endl;
        }
    }

    // Q construciton 
    Q.block<3,3>(0,0) = PROCESS_NOISE_P_IMU * dt /20.0 * eye3;
    Q.block<3,3>(3,3) = PROCESS_NOISE_V_IMU * dt *9.8/20.0*eye3;

    // update Q R for the legs are ont in contact
    for (int i=0;i<NUM_LEG;++i){
        // for legs not in contact, we have high covariance meaning we dont update 
        Q.block<3,3>(6 + i*3, 6+i*3) = (1 + (1-estimated_contacts[i])*1e3) *dt * PROCESS_NOISE_P_FOOT *eye3;
        // for legs not in contact, we have high covariance for the position of foot rel (forward kinematics estimation)
        R.block<3,3>(i*3,i*3) = (1 + (1-estimated_contacts[i])* 1e3) * SENSOR_NOISE_P_IMU_REL_FOOT * eye3;
        R.block<3,3>(NUM_LEG*3+ i*3, NUM_LEG*3+ i*3) = (1 + (1-estimated_contacts[i])*1e3)*SENSOR_NOISE_V_IMU_REL_FOOT * eye3;

        if (assume_flat_ground){
            // if the foot was not in contact on the flat ground, the z estimation should have maximum noise
            R(NUM_LEG*6 +i, NUM_LEG*6 +i) = (1 + (1-estimated_contacts[i])* 1e3) * SENSOR_NOISE_ZFOOT;
        }
    }

    // state transition 
    xbar = A*x +B*u;
    Pbar = A * P * A.transpose() + Q;

    // construct measurement function 
    yhat = C * xbar;

    // get measurement to calculate the posteriior 

    for (int i = 0; i< NUM_LEG; ++i){
        Eigen::Vector3d fk_pos = state.foot_pos_rel.block<3,1>(0,i);
        y.block<3,1>(i*3,0) = state.root_rot_matrix*fk_pos; // forward kinematics foot position relative to the world 
        
        // leg velocity in world frame = foot velocity in body frame + body movement in the world frame
        Eigen::Vector3d leg_v = -state.foot_vel_rel.block<3,1>(0,i) - Utils::skew(state.imu_ang_vel) * fk_pos; 
        
        // use leg velocity while in the contact, use body velocity while not in contact 
        y.block<3,1>(NUM_LEG*3+i*3,0) = (1.0- estimated_contacts[i])*x.segment<3>(3) + estimated_contacts[i]*state.root_rot_matrix*leg_v;


        // if leg is in contact z=0 not in contact use forward kinematics
        y(NUM_LEG*6 + i) = (1.0 - estimated_contacts[i]) * (x(2)*fk_pos(2)) + estimated_contacts[i] * 0;
    }

    // S observation covariance matrix 28*28
    S = C * Pbar * C.transpose() + R;
    S = 0.5 * (S + S.transpose());

    error_y = y-yhat;
    Serror_y = S.fullPivHouseholderQr().solve(error_y);

    x = xbar +Pbar*C.transpose() * Serror_y;

    SC = S.fullPivHouseholderQr().solve(C);
    P = Pbar - Pbar * C.transpose()*SC *Pbar;
    P = 0.5 * (P +P.transpose());
    
    // reduce position drift
    if (P.block<2,2>(0,0).determinant()>1e-6){
        P.block<2,16>(0,2).setZero();
        P.block<16,2>(2,0).setZero();
        P.block<2,2>(0,0) /= 10.0;
    }

    // >50N is considered contact
    for (int i = 0; i<NUM_LEG; ++i){
        if (estimated_contacts[i] <0.5){
            state.estimated_contacts[i] = false;
        } else{
            state.estimated_contacts[i] = true;
        }
    }

    state.estimated_root_pos = x.segment<3>(0);
    state.estimated_root_vel = x.segment<3>(3);
    
    state.root_pos = x.segment<3>(0);
    state.root_lin_vel = x.segment<3>(3);
    // std::cout<<"root linear velocity: "<<state.root_lin_vel<<std::endl;
    // std::cout<<"root pos: "<<state.root_pos <<std::endl;
    // std::cout<<"        "<<std::endl;
}