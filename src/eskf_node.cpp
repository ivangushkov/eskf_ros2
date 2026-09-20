#include <cstdio>
#include <iostream>

#include "quaternions/quaternions.h"
#include<Eigen/Dense>


int main(int argc, char ** argv)
{
  
  (void) argc;
  (void) argv;
  Eigen::Vector3d test_vec;
  float test_real = 6.7;

  test_vec << 4, 2, 0;

  RotationQuaternion test_quat(test_real, test_vec);
  RotationQuaternion test_quat2(test_real, test_vec);

  test_quat.normalize();
  test_quat2.normalize();

  std::cout << test_quat.vec_part << std::endl;
  std::cout << test_quat.as_rotmat() << std::endl;
  std::cout << test_quat.as_avec() << std::endl;

  std::cout << test_quat.multiply(test_quat2).vec_part << std::endl;

  printf("hello world eskf package\n");
  return 0;


}
