
#pragma once

#include<Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions> // for that exponential matrix

#include <cmath>
#include "quaternions/quaternions.h"

// Tuning and parameters of the ESKF
struct ESKFParams{
    float accm_std;
    float accm_bias_std;
    float accm_bias_p; 

    float gyro_std; 
    float gyro_bias_std; 
    float gyro_bias_p; 

    float gnss_std_n;
    float gnss_std_e;
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
    float ts;
};

struct ESKFState{
    NominalState nomState;
    ErrorStateGauss errorState;
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

struct GNSSMeasurementGauss{
    Eigen::Vector3d mean;
    Eigen::Matrix3d cov;
    float ts;
};

class ESKF {
    public:
        ESKF();
        ESKF(ESKFParams p);

        NominalState getNomState();
        ErrorStateGauss getErrState();
        ESKFState getFullState();

        void predictFromIMU(IMUMeasurement z_imu);
        void updateFromGNSS(GNSSMeasurement zGNSS);

    private:
        ESKFParams p;
        
        Eigen::MatrixXd Q_err;
        Eigen::Vector3d g;
        Eigen::Matrix3d gnssCov;

        NominalState nomState;
        ErrorStateGauss errorState;

        NominalState nomState_aided;
        ErrorStateGauss errorState_aided;

        // Math methods
        Eigen::Matrix3d skew_symmetric(Eigen::Vector3d vec);
        Eigen::MatrixXd gnssMeasurementJacobian(NominalState xNom);


        // IMU prediction methods
        IMUMeasurement correctIMUMeasurement(NominalState x_nom_prev, IMUMeasurement z_imu);
        NominalState predictNominalState(NominalState x_nom_prev, IMUMeasurement z_imu_corr);
        ErrorStateGauss predictErrorState(NominalState x_nom_prev, ErrorStateGauss x_err_gauss, IMUMeasurement z_imu_corr);

        // GNSS update methods
        GNSSMeasurementGauss predictGNSSMeasurement(NominalState xNom, ErrorStateGauss xErr, GNSSMeasurement zGNSS, Eigen::MatrixXd H);
        ErrorStateGauss updateErrorState(ErrorStateGauss xErrPrev, GNSSMeasurementGauss zGNSSPred, GNSSMeasurement zGNSS, Eigen::MatrixXd H);
        ESKFState inject(NominalState xNomPrev, ErrorStateGauss xErrUpd);


    };