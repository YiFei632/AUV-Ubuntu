#ifndef AUV_CONTROL_MATRIX_UTILS_HPP
#define AUV_CONTROL_MATRIX_UTILS_HPP

#include <Eigen/Dense>

namespace auv_control {

/**
 * @brief 计算AUV雅可比矩阵（对应eta2J_auv函数）
 * @param eta 位姿向量 [depth, roll, pitch, yaw]
 * @return 雅可比矩阵（4x4）
 */
Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd& eta);

/**
 * @brief 计算科里奥利矩阵（对应m2c_auv函数）
 * @param M 质量矩阵（4x4对角阵）
 * @param nu 速度向量 [u, v, r, p]
 * @return 科里奥利矩阵（4x4）
 */
Eigen::MatrixXd calculateCoriolisMatrix(const Eigen::MatrixXd& M, const Eigen::VectorXd& nu);

} // namespace auv_control

#endif // AUV_CONTROL_MATRIX_UTILS_HPP