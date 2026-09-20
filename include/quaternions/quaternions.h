
#pragma once

#include<Eigen/Dense>


class RotationQuaternion {
    public:
        RotationQuaternion();
        RotationQuaternion(float real_part, Eigen::Vector3d vec_part);
    
        float real_part;
        Eigen::Vector3d vec_part;

        Eigen::Matrix3d as_rotmat();
        Eigen::Vector3d as_euler();
        Eigen::Vector3d as_avec();

        RotationQuaternion multiply(RotationQuaternion other);
        RotationQuaternion conjugate();
        void normalize();

    private:
        Eigen::Matrix3d skew_symmetric(Eigen::Vector3d vec);

    };