#include "eskf/eskf.h"

#include<Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions> // for that exponential matrix

#include<cmath>
#include "quaternions/quaternions.h"

ESKF::ESKF(ESKFParams p)
    : p {p}
{
    Q_err = Eigen::MatrixXd::Zero(12,12);

    // Accelerometer noise
    Eigen::Matrix3d Q1 = std::pow(p.accm_std, 2) * p.accm_correction_matrix * p.accm_correction_matrix.transpose();
    
    // Gyro noise
    Eigen::Matrix3d Q2 = std::pow(p.gyro_std, 2) * p.gyro_correction_matrix * p.gyro_correction_matrix.transpose();

    // Accelerometer bias process noise
    Eigen::Matrix3d Q3 = std::pow(p.accm_bias_std, 2) * Eigen::MatrixXd::Identity(3, 3);

    // Gyro bias process noise
    Eigen::Matrix3d Q4 = std::pow(p.gyro_bias_std, 2) * Eigen::MatrixXd::Identity(3, 3);
    
    // Total noise matrix
    Q_err.block(0, 0, 3, 3) = Q1;
    Q_err.block(3, 3, 3, 3) = Q2;
    Q_err.block(6, 6, 3, 3) = Q3;
    Q_err.block(9, 9, 3, 3) = Q4;

    // Gravity vector in the world frame
    g << 0.0, 0.0, 9.81;

    // GNSS measurement covariance
    gnssCov = Eigen::Matrix3d::Zero();
    gnssCov(0, 0) = p.gnss_std_n;
    gnssCov(1, 1) = p.gnss_std_e;
    gnssCov(2, 2) = p.gnss_std_d;

    // Initial guesses
    nomState = p.x0;
    errorState = p.xErr0;

}

// Getter methods for the filter state
NominalState ESKF::getNomState(){
    return nomState;
}
ErrorStateGauss ESKF::getErrState(){
    return errorState;
}
ESKFState ESKF::getFullState(){
    ESKFState state{nomState, errorState};
    return state;
}

Eigen::Matrix3d ESKF::skew_symmetric(Eigen::Vector3d vec) {

    Eigen::Matrix3d S;

    S << 0,  -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;
    
    return S;
};

IMUMeasurement ESKF::correctIMUMeasurement(NominalState x_nom_prev, IMUMeasurement z_imu) {

    IMUMeasurement imuCorr;
    
    imuCorr.acc = p.accm_correction_matrix * (z_imu.acc - x_nom_prev.accm_bias);
    imuCorr.avel = p.gyro_correction_matrix * (z_imu.avel - x_nom_prev.gyro_bias);
    imuCorr.ts = z_imu.ts;

    return imuCorr;
}

NominalState ESKF::predictNominalState(NominalState x_nom_prev, IMUMeasurement z_imu_corr) {

    float Ts = z_imu_corr.ts;

    Eigen::Vector3d acc = x_nom_prev.ori.as_rotmat() * z_imu_corr.acc + g;

    // Incremental quaternion from the angular velocity
    Eigen::Vector3d omegaIncrement = Ts * z_imu_corr.avel;
    RotationQuaternion quatIncrement = RotationQuaternion(
        std::cos(omegaIncrement.norm()/2), 
        std::sin(omegaIncrement.norm()/2) * omegaIncrement / omegaIncrement.norm()
    );

    NominalState x_nom_pred; 
    
    // Numerical integration of the dynamics
    x_nom_pred.pos = x_nom_prev.pos + Ts * x_nom_prev.vel + (std::pow(Ts, 2) / 2) * acc;
    x_nom_pred.vel = x_nom_prev.vel + Ts * acc;
    x_nom_pred.ori = x_nom_prev.ori.multiply(quatIncrement);
    x_nom_pred.accm_bias = x_nom_prev.accm_bias - Ts*(p.accm_bias_p * x_nom_prev.accm_bias);
    x_nom_pred.gyro_bias = x_nom_prev.gyro_bias - Ts*(p.gyro_bias_p * x_nom_prev.gyro_bias);

    return x_nom_pred;

}

ErrorStateGauss ESKF::predictErrorState(NominalState x_nom_prev, ErrorStateGauss x_err_gauss, IMUMeasurement z_imu_corr) {

    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(15,15);
    Eigen::Matrix3d Rq = x_nom_prev.ori.as_rotmat();

    // Error state dynamics matrix A(x) in continious time

    Ac.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
    Ac.block(3, 6, 3, 3) = -Rq * this->skew_symmetric(z_imu_corr.acc);
    Ac.block(3, 9, 3, 3) = -Rq * p.accm_correction_matrix;
    Ac.block(6, 6, 3, 3) = -this->skew_symmetric(z_imu_corr.avel);
    Ac.block(6, 12, 3, 3) = - p.gyro_correction_matrix;
    Ac.block(9, 9, 3, 3) = - p.accm_bias_p * Eigen::Matrix3d::Identity();
    Ac.block(12, 12, 3, 3) = -p.gyro_bias_p * Eigen::Matrix3d::Identity();


    // Transformation of the process noise in continuous time GQGTc

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(15, 12);

    G.block(3, 0, 3, 3) = -Rq;
    G.block(6, 3, 3, 3) = - Eigen::Matrix3d::Identity();
    G.block(9, 6, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(12, 9, 3, 3) = Eigen::Matrix3d::Identity();
    
    Eigen::MatrixXd GQGTc = G * Q_err * G.transpose();

    // Discretization of system matrix and process noise
    float Ts = z_imu_corr.ts;
    Eigen::MatrixXd VL = Eigen::MatrixXd::Zero(30, 30); // VanLoan matrix

    //VL.block(0, 0, Ac.rows(), Ac.cols()) = -Ac;
    //VL.block(Ac.)

    VL.topLeftCorner(Ac.rows(), Ac.cols()) = -Ac;
    VL.topRightCorner(GQGTc.rows(), GQGTc.cols()) = GQGTc;
    VL.bottomRightCorner(Ac.rows(), Ac.cols()) = Ac.transpose();

    VL *= Ts;
    VL = VL.exp();

    Eigen::MatrixXd VL1 = VL.bottomRightCorner(Ac.rows(), Ac.cols());
    Eigen::MatrixXd VL2 = VL.topRightCorner(Ac.rows(), Ac.cols());

    // Finally Ad and GQGTd
    Eigen::MatrixXd Ad = VL1.transpose();
    Eigen::MatrixXd GQGTd = VL1.transpose() * VL2;

    ErrorStateGauss x_err_gauss_pred{
        Ad * x_err_gauss.mean,
        Ad * x_err_gauss.cov * Ad.transpose() + GQGTd,
        z_imu_corr.ts
    };

    return x_err_gauss_pred;
    
}

void ESKF::predictFromIMU(IMUMeasurement z_imu) {

    NominalState x_nom_prev = nomState;
    ErrorStateGauss x_err_gauss = errorState;

    // Correct the IMU measurements for mounting errors
    IMUMeasurement z_imu_corr = this->correctIMUMeasurement(x_nom_prev, z_imu);

    // Integrate the nominal system dynamics with the imu as input
    NominalState x_nom_pred = this->predictNominalState(x_nom_prev, z_imu_corr);

    // Update the error state covariance
    ErrorStateGauss x_err_pred = this->predictErrorState(x_nom_prev, x_err_gauss, z_imu_corr);

    // Update internal state with the predictions
    nomState = x_nom_pred;
    errorState = x_err_gauss;
}

Eigen::MatrixXd ESKF::gnssMeasurementJacobian(NominalState xNom) {

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,15);
    
    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, 6, 3, 3) = - xNom.ori.as_rotmat() * this->skew_symmetric(p.gnss_lever);
    
    return H;
}

GNSSMeasurementGauss ESKF::predictGNSSMeasurement(NominalState xNom, ErrorStateGauss xErr, GNSSMeasurement zGNSS, Eigen::MatrixXd H) {

    // Compensate for the GNSS placement
    Eigen::Vector3d predMean = xNom.pos + xNom.ori.as_rotmat() * p.gnss_lever;

    // Predicting the covariance
    Eigen::Matrix3d predCov = H * xErr.cov * H.transpose() + gnssCov;
    
    GNSSMeasurementGauss zGNSSPred{predMean, predCov, zGNSS.ts};

    return zGNSSPred;
}

ErrorStateGauss ESKF::updateErrorState(ErrorStateGauss xErrPrev, GNSSMeasurementGauss zGNSSPred, GNSSMeasurement zGNSS, Eigen::MatrixXd H) {

    Eigen::MatrixXd P = xErrPrev.cov;
    Eigen::Matrix3d R = gnssCov;

    // Kalman Gain
    Eigen::MatrixXd W = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    Eigen::MatrixXd IWH = Eigen::MatrixXd::Identity(P.rows(), P.cols()) - W * H;

    Eigen::VectorXd errMean = W * (zGNSS.pos - zGNSSPred.mean);
    Eigen::MatrixXd errCov = IWH * P * IWH.transpose() + W * R * W.transpose();

    ErrorStateGauss xErrUpd{errMean, errCov, zGNSS.ts};

    return xErrUpd;
}

ESKFState ESKF::inject(NominalState xNomPrev, ErrorStateGauss xErrUpd) {

    // Orientation error variable and quaternion increment
    Eigen::Vector3d errTheta = xErrUpd.mean(Eigen::seq(6,8));
    RotationQuaternion errQuat = RotationQuaternion(1, 1/2 * errTheta);

    ESKFState updatedState;
    
    // Inject information into the nominal state
    updatedState.nomState.pos = xNomPrev.pos + xErrUpd.mean(Eigen::seq(0,2));
    updatedState.nomState.vel = xNomPrev.vel + xErrUpd.mean(Eigen::seq(3,5));
    updatedState.nomState.ori = xNomPrev.ori.multiply(errQuat);
    updatedState.nomState.accm_bias = xNomPrev.accm_bias + xErrUpd.mean(Eigen::seq(9,11));
    updatedState.nomState.accm_bias = xNomPrev.accm_bias + xErrUpd.mean(Eigen::seq(12,14));

    // Reset the error state mean
    updatedState.errorState.mean = Eigen::VectorXd::Zero(xErrUpd.mean.size());
    
    // Final update to the error state covariance
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(15, 15);
    G.block(6, 6, 3, 3) -= this->skew_symmetric(errTheta); // this will need thorough debugging!
    updatedState.errorState.cov = G * xErrUpd.cov * G.transpose();

    return updatedState;

}

void ESKF::updateFromGNSS(GNSSMeasurement zGNSS) {

    NominalState xNomPrev = nomState;
    ErrorStateGauss xErrPrev = errorState;

    // GNSS measurement Jacobian
    Eigen::MatrixXd H = this->gnssMeasurementJacobian(xNomPrev);

    GNSSMeasurementGauss zGNSSPred = this->predictGNSSMeasurement(xNomPrev, xErrPrev, zGNSS, H);
    ErrorStateGauss xErrUpd = this->updateErrorState(xErrPrev, zGNSSPred, zGNSS, H);
    ESKFState updatedState = this->inject(xNomPrev, xErrUpd);

    // Set the internal state as the injected state
    nomState = updatedState.nomState;
    errorState = updatedState.errorState;

}