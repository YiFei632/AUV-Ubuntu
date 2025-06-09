#ifndef AUV_CONTROL_MPC_HPP
#define AUV_CONTROL_MPC_HPP

#include <Eigen/Dense>
#include <vector>
#include <qpOASES.hpp>

namespace auv_control {

class MPCController {
public:
    /**
     * @brief MPC控制器构造函数
     * @param Nc 控制时域
     * @param Np 预测时域
     */
    MPCController(int Nc = 10, int Np = 10);

    /**
     * @brief 计算预测矩阵（对应MATLAB的Gain函数）
     * @param A 状态矩阵
     * @param B 输入矩阵
     * @param C 输出矩阵
     * @param F 预测状态矩阵（输出）
     * @param Phi 控制影响矩阵（输出）
     * @param Phi_Phi Phi^T*Phi矩阵（输出）
     * @param BarR 权重矩阵（输出）
     */
    void calculateGains(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C,
                        Eigen::MatrixXd& F, Eigen::MatrixXd& Phi, 
                        Eigen::MatrixXd& Phi_Phi, Eigen::MatrixXd& BarR);

    // 修正后的computeControlInput方法（使用QpOASES求解）
    Eigen::VectorXd computeControlInput(const Eigen::VectorXd& x, 
                                        const Eigen::VectorXd& reference);

private:
    int Nc_; ///< 控制时域
    int Np_; ///< 预测时域

    // QpOASES求解器实例（用于实时QP求解）
    qpOASES::SQProblem qp_solver_;
    qpOASES::Options qp_options_;  ///< QP求解器参数

    // 辅助变量（存储QP问题矩阵）
    Eigen::MatrixXd H_;  ///< 海森矩阵（二次项系数）
    Eigen::VectorXd f_;  ///< 梯度向量（一次项系数）
    Eigen::MatrixXd A_;  ///< 不等式约束矩阵
    Eigen::VectorXd lb_; ///< 控制输入下限
    Eigen::VectorXd ub_; ///< 控制输入上限
};

} // namespace auv_control

#endif // AUV_CONTROL_MPC_HPP