//
// Created by tcorroenne2021 on 28/03/23.
//
#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "vector"
#include "array"
#include <Eigen/Core>
#include <Eigen/QR>
#include "sensor_msgs/msg/joint_state.hpp"
#include <lab_sort/srv/jacobian.hpp>

// This file aims to reduce the number of function written in the rest of the lab.

inline std::vector<double> computeCommand(const std::array<double, 42> &Jinv_coeffs,
                                          const Eigen::Matrix<double,6,1> &vec_twist){
    Eigen::Matrix<double,7,6, Eigen::RowMajor> Jinv;
    std::copy(Jinv_coeffs.begin(), Jinv_coeffs.end(), Jinv.data());
    const Eigen::Matrix<double,7,1> cmd{(Jinv*vec_twist)};
    return {cmd.data(), cmd.data()+7};
}

inline Eigen::MatrixXd compute_Ls_inv(const double& x,const double& y,const double& Z){
    // Building ls as an eigen matrix (2,6), refer to visual servoing course
    Eigen::Matrix<double,2,6, Eigen::RowMajor> Ls ;
    Ls << -1/Z,0,x/Z,x*y,-(1+std::pow(x,2)),y,
    0,-1/Z,y/Z,1+std::pow(y,2),-x*y,-x;
    //std::cout << "Matrix Ls :" << Ls <<std::endl;
    // Computing the inverse : 
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(Ls);
    Eigen::MatrixXd Ls_inverse = cqr.pseudoInverse();
    return Ls_inverse;
}

inline void get_pos(sensor_msgs::msg::JointState s, lab_sort::srv::Jacobian_Request &req,std::string side){
    int start_array = 2;
    if(side=="right"){
    start_array = 9;
    }
    std::vector<int> to_add{2,3,0,1,4,5,6};
    for (int i =0;i<7;i++){
    req.position[i] = s.position[start_array+to_add[i]];
    }
}



#endif //UTILITIES_HPP