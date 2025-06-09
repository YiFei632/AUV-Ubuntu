#ifndef AUV_CONTROL_ADAPTIVE_HPP
#define AUV_CONTROL_ADAPTIVE_HPP

#include <Eigen/Dense>

namespace auv_control {

class AdaptiveController {
public:
    /**
     * @brief 自适应控制器构造函数
     * @param lambda 自适应增益参数
     * @param max_steps 最大迭代步数
     */
    AdaptiveController(double lambda = 0.02, int max_steps = 600);

    /**
     * @brief 更新自适应参数
     * @param x 当前状态向量 [dx; x]（假设8维）
     * @param hat_x 估计状态向量
     * @param du 控制输入增量
     * @return 更新后的参数矩阵
     */
    Eigen::MatrixXd updateParameters(const Eigen::MatrixXd& x, 
                                     const Eigen::MatrixXd& hat_x, 
                                     const Eigen::MatrixXd& du);

private:
    int sig_;          ///< 当前迭代步数
    int steps_max_;    ///< 最大迭代步数
    double lambda_;    ///< 自适应增益
    Eigen::MatrixXd theta_last_; ///< 上一时刻参数矩阵（假设8x8维度）
};

} // namespace auv_control

#endif // AUV_CONTROL_ADAPTIVE_HPP