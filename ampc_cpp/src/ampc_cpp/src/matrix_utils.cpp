#include "ampc_cpp/matrix_utils.hpp"
#include <cmath>

namespace auv_control {

Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd& eta) {
    // eta: [depth, roll, pitch, yaw]
    if (eta.size() != 4) {
        throw std::invalid_argument("eta must be 4-dimensional");
    }
    
    double roll = eta(1), pitch = eta(2), yaw = eta(3);
    
    // 构建雅可比矩阵
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4, 4);
    J(0,0) = 1.0; // depth直接传递
    
    // 姿态角转换矩阵（简化版）
    J(1,1) = cos(pitch) * cos(yaw);
    J(1,2) = -sin(yaw);
    J(1,3) = 0;
    
    J(2,1) = cos(pitch) * sin(yaw);
    J(2,2) = cos(yaw);
    J(2,3) = 0;
    
    J(3,1) = -sin(pitch);
    J(3,2) = 0;
    J(3,3) = 1;
    
    return J;
}

Eigen::MatrixXd calculateCoriolisMatrix(const Eigen::MatrixXd& M, const Eigen::VectorXd& nu) {
    // M: 4x4质量矩阵, nu: [u, v, r, p]速度向量
    if (M.rows() != 4 || M.cols() != 4 || nu.size() != 4) {
        throw std::invalid_argument("Matrix M must be 4x4 and nu must be 4-dimensional");
    }
    
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(4, 4);
    
    // 根据AUV动力学填充科里奥利矩阵
    C(0,1) = -M(1,1) * nu(1);
    C(0,3) = -M(3,3) * nu(3);
    C(1,0) = M(0,0) * nu(0);
    C(1,3) = M(3,3) * nu(3);
    C(3,0) = M(0,0) * nu(0);
    C(3,1) = -M(1,1) * nu(1);
    
    return C;
}

} // namespace auv_control