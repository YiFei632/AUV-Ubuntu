#include "ampc_cpp/kalman_filter.hpp"

namespace auv_control {

KalmanFilter::KalmanFilter(double Q, double R) {
    // 初始化6维状态估计和协方差矩阵
    x_hat_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6);
    
    // 设置过程噪声和测量噪声协方差
    Q_ = Q * Eigen::MatrixXd::Identity(6, 6);
    R_ = R * Eigen::MatrixXd::Identity(6, 6);
}

Eigen::VectorXd KalmanFilter::filter(const Eigen::VectorXd& measurement) {
    // 检查测量数据维度
    if (measurement.size() != 6) {
        throw std::invalid_argument("Measurement must be 6-dimensional");
    }
    
    // 预测步骤（假设状态转移矩阵为单位矩阵）
    Eigen::VectorXd x_hat_minus = x_hat_;  // 预测状态
    Eigen::MatrixXd P_minus = P_ + Q_;     // 预测协方差
    
    // 更新步骤
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);  // 观测矩阵
    Eigen::MatrixXd S = H * P_minus * H.transpose() + R_; // 新息协方差
    Eigen::MatrixXd K = P_minus * H.transpose() * S.inverse(); // 卡尔曼增益
    
    // 状态更新
    Eigen::VectorXd innovation = measurement - H * x_hat_minus;
    x_hat_ = x_hat_minus + K * innovation;
    
    // 协方差更新
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_minus;
    
    return x_hat_;
}

} // namespace auv_control