
#pragma once

#include<Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
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

    Eigen::Matrix3d accm_correction_matrix;
    Eigen::Matrix3d gyro_correction_matrix;
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

struct ErrorStateModel{
    Eigen::MatrixXd A;
    Eigen::MatrixXd GQGT;
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

        NominalState nomState_aided;
        ErrorStateGauss errorState_aided;

        Eigen::Matrix3d skew_symmetric(Eigen::Vector3d vec);

        IMUMeasurement correctIMUMeasurement(NominalState x_nom_prev, IMUMeasurement z_imu);
        NominalState predictNominalState(NominalState x_nom_prev, IMUMeasurement z_imu_corr);
        ErrorStateGauss predictErrorState(NominalState x_nom_prev, ErrorStateGauss x_err_gauss, IMUMeasurement z_imu_corr);



    };