
#pragma once

#include<Eigen/Dense>
#include "quaternions/quaternions.h"

// Tuning of the ESKF
struct ESKFParams{
    float accm_std;
    float accm_bias_std;
    float accm_bias_p; 

    float gyro_std; 
    float gyro_bias_std; 
    float gyro_bias_p; 

    float gnss_std_ne;
    float gnss_std_d; 

    Eigen::Matrix3d accm_correction;
    Eigen::Matrix3d gyro_correction;
    Eigen::Vector3d gnss_lever;
};

// Nominal state with quaternion parametrization of the orientation
struct NominalState{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    RotationQuaternion ori;
    Eigen::Vector3d accm_bias;
    Eigen::Vector3d gyro_bias;
};

// Error state of the ESKF
struct ErrorStateGauss{
    Eigen::VectorXd mean;
    Eigen::MatrixXd cov;
};

// Struct for the IMU measurement with timestamp
struct IMUMeasurement{
    Eigen::Vector3d acc;
    Eigen::Vector3d avel;
    float ts;
};

// Struct for the GNSS measurement with timestamp
struct GNSSMeasurement{
    Eigen::Vector3d pos;
    float ts;
};

class ESKF {
    public:
        ESKF();
        ESKF(ESKFParams p);

        NominalState getNomState();
        ErrorStateGauss getErrState();

        void predictFromIMU(IMUMeasurement z_imu);
        void updateFromGNSS(GNSSMeasurement z_gnss);

    private:
        ESKFParams p;
        
        Eigen::MatrixXd Q_err;
        Eigen::Vector3d g;

        NominalState nomState;
        ErrorStateGauss errorState;
};