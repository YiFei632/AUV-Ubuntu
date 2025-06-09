#include "ampc_cpp/adaptive.hpp"

namespace auv_control {

AdaptiveController::AdaptiveController(double lambda, int max_steps) 
    : lambda_(lambda), steps_max_(max_steps), sig_(0) {
    // 初始化参数矩阵（8x8维度）
    theta_last_ = Eigen::MatrixXd::Zero(8, 8);
}

Eigen::MatrixXd AdaptiveController::updateParameters(const Eigen::MatrixXd& x, 
                                                    const Eigen::MatrixXd& hat_x, 
                                                    const Eigen::MatrixXd& du) {
    // 检查输入维度
    if (x.rows() != hat_x.rows() || x.cols() != hat_x.cols()) {
        throw std::invalid_argument("State and estimated state dimensions must match");
    }
    
    if (sig_ < steps_max_) {
        // 构造扩展状态向量 X = [x; du]
        Eigen::MatrixXd X(x.rows() + du.rows(), x.cols());
        X.topRows(x.rows()) = x;
        X.bottomRows(du.rows()) = du;
        
        // 自适应参数更新：theta += lambda * (x - hat_x) * X^T
        Eigen::MatrixXd error = x - hat_x;
        theta_last_ += lambda_ * error * X.transpose();
    }
    
    sig_++;
    return theta_last_;
}

} // namespace auv_control