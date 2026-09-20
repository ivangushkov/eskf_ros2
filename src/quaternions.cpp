#include "quaternions/quaternions.h"

RotationQuaternion::RotationQuaternion(float real_part, Eigen::Vector3d vec_part)
    : real_part{real_part}, vec_part{vec_part}    
{

    // Make sure we initialize as a unit quaternion
    this->normalize();

    // Get shortest rotation
    if (real_part < 0) {
        real_part *= -1;
        vec_part *= -1;
    }

};

Eigen::Matrix3d RotationQuaternion::skew_symmetric(Eigen::Vector3d vec) {

    Eigen::Matrix3d S;

    S << 0,  -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;
    
    return S;
};

RotationQuaternion RotationQuaternion::multiply(RotationQuaternion other) {
    
    float product_real = this->real_part * other.real_part - this->vec_part.dot(other.vec_part);
    Eigen::Vector3d product_vec = other.real_part*this->vec_part + this->real_part*other.vec_part + this->skew_symmetric(this->vec_part) * other.vec_part;

    return RotationQuaternion(product_real, product_vec);
};

RotationQuaternion RotationQuaternion::conjugate() {
    return RotationQuaternion(this->real_part, -this->vec_part);
};

Eigen::Matrix3d RotationQuaternion::as_rotmat() {

    Eigen::Matrix3d S = this->skew_symmetric(this->vec_part);
    Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3) + 2*this->real_part*S*S;
    
    return R;

};

Eigen::Vector3d RotationQuaternion::as_avec(){

    const Eigen::Vector3d vector_part = this->vec_part;
    const double vector_norm = vector_part.norm();
    
    constexpr double epsilon = 1e-10;
    
    if (vector_norm < epsilon) {
        return 2.0 * vector_part;
    }
    
    const double angle =
    2.0 * std::atan2(vector_norm, this->real_part);
    
    return (angle / vector_norm) * vector_part;

};

void RotationQuaternion::normalize(){
    
    float norm = std::sqrt(std::pow(this->real_part, 2) + std::pow(this->vec_part.norm(), 2));
    this->real_part *= 1/norm;
    this->vec_part *= 1/norm;

};