#include "eskf/eskf.h"

#include<Eigen/Dense>
#include "quaternions/quaternions.h"

ESKF::ESKF(ESKFParams p)
    : p {p}
{
    Q_err = Eigen::MatrixXd::Zero(12,12);

    // Accelerometer noise
    Eigen::Matrix3d Q1 = std::pow(p.accm_std, 2) * p.accm_correction * p.accm_correction.transpose();
    
    // Gyro noise
    Eigen::Matrix3d Q2 = std::pow(p.gyro_std, 2) * p.gyro_correction * p.gyro_correction.transpose();

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

}
