#include <cstdio>
#include <iostream>

#include "quaternions/quaternions.h"
#include "eskf/eskf.h"
#include<Eigen/Dense>


int main(int argc, char ** argv)
{
  
  (void) argc;
  (void) argv;
  

  Eigen::Vector3d gnss_lever;
  gnss_lever << 1.0, 0.0, 2.0;
  
  
  ESKFParams p{
    2.0,                              // accm_std
    4.0,                              // accm_bias_std
    6.0,                              // accm_bias_p
    8.0,                              // gyro_std
    10.0,                             // gyro_bias_std
    12.0,                             // gyro_bias_p
    20.0,                             // gnss_std_ne;
    20.0,                             // gnss_std_d;
    Eigen::MatrixXd::Identity(3, 3),  // accm_correction
    Eigen::MatrixXd::Identity(3, 3),  // gyro_correction 
    gnss_lever                        // gnss lever arm
  };
  

  ESKF eskf(p);
  return 0;


}
