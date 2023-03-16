#ifndef EXAM_2022_EIGEN_H
#define EXAM_2022_EIGEN_H

//#include <Eigen/Core>
//#include <lab_tricolor/ik_client.h>
//#include <geometry_msgs/msg/twist.hpp>
// math to meaningfull stuff
//using JacobianInverseCoefs = std::array<double, 42>;

inline std::vector<double> computeCommand(const std::array<double, 42> &Jinv_coeffs,
                                          const geometry_msgs::msg::Twist &v)
{
  Eigen::Matrix<double,7,6, Eigen::RowMajor> Jinv;
  std::copy(Jinv_coeffs.begin(), Jinv_coeffs.end(), Jinv.data());
  Eigen::Matrix<double,6,1> vec;
  vec << v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z;
  const Eigen::Matrix<double,7,1> cmd{(Jinv*vec)};
  return {cmd.data(), cmd.data()+7};
}

#endif // EXAM_2022_EIGEN_H
