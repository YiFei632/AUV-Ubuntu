#include "ampc_cpp/mpc.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <qpOASES.hpp>

namespace auv_control {

MPCController::MPCController(int Nc, int Np) 
    : Nc_(Nc), Np_(Np),
      qp_solver_(Nc * 6, 0)  // 控制输入维度：Nc*6（假设6维推力，每个时域步长6维）
{
    // 初始化QpOASES求解器参数（实时模式）
    qp_options_.setToMPC();       // 优化MPC场景
    qp_options_.printLevel = qpOASES::PL_LOW; // 低级别输出
    qp_solver_.setOptions(qp_options_);

    // 初始化控制输入约束（示例：推力范围[-50, 50] Nm）
    lb_ = Eigen::VectorXd::Constant(Nc_ * 6, -50.0);
    ub_ = Eigen::VectorXd::Constant(Nc_ * 6, 50.0);
}

void MPCController::calculateGains(const Eigen::MatrixXd& A, 
                                 const Eigen::MatrixXd& B, 
                                 const Eigen::MatrixXd& C,
                                 Eigen::MatrixXd& F, 
                                 Eigen::MatrixXd& Phi, 
                                 Eigen::MatrixXd& Phi_Phi, 
                                 Eigen::MatrixXd& BarR) {
    int m1 = C.rows();
    int n1 = C.cols();
    int n_in = B.cols();
    int Np_m1 = Np_ * m1;
    int Nc_n_in = Nc_ * n_in;
    
    // 初始化矩阵
    F = Eigen::MatrixXd::Zero(Np_m1, n1 + m1);
    Phi = Eigen::MatrixXd::Zero(Np_m1, Nc_n_in);
    
    // 计算F矩阵
    Eigen::MatrixXd CAB = C * A * B;
    F.block(0, 0, m1, n1) = C * A;
    
    for(int i = 1; i < Np_; i++) {
        F.block(i*m1, 0, m1, n1) = F.block((i-1)*m1, 0, m1, n1) * A;
    }
    
    // 计算Phi矩阵
    for(int i = 0; i < Np_; i++) {
        for(int j = 0; j < Nc_; j++) {
            if(i >= j) {
                Eigen::MatrixXd temp = C;
                for(int k = 0; k < i-j; k++) {
                    temp = temp * A;
                }
                temp = temp * B;
                Phi.block(i*m1, j*n_in, m1, n_in) = temp;
            }
        }
    }
    
    // 计算Phi_Phi和BarR
    Phi_Phi = Phi.transpose() * Phi;
    int nPhi = Phi_Phi.rows();
    Eigen::VectorXd diagR = Eigen::VectorXd::Zero(nPhi);
    diagR.head(nPhi/4) << 1.0;        // 前1/4维度权重1
    diagR.segment(nPhi/4, nPhi/4) << 50.0; // 中间1/4权重50
    diagR.tail(nPhi/2) << 0.1, 0.0;  // 后1/2维度权重0.1和0
    BarR = 0.1 * diagR.asDiagonal();
}

Eigen::VectorXd MPCController::computeControlInput(const Eigen::VectorXd& x, 
                                                  const Eigen::VectorXd& reference) {
    // 校验输入维度（假设x为8维状态，reference为4维输出参考）
    if (x.size() != 8 || reference.size() != 4) {
        throw std::invalid_argument("computeControlInput: x(8D) or reference(4D) dimension mismatch");
    }

    // ----------------------
    // 步骤1：构建QP问题矩阵
    // ----------------------
    // 假设已通过calculateGains得到F和Phi（需根据实际模型填充）
    Eigen::MatrixXd F, Phi, Phi_Phi, BarR;
    calculateGains(Eigen::MatrixXd::Identity(8,8), Eigen::MatrixXd::Zero(8,6), 
                   Eigen::MatrixXd::Identity(4,8), F, Phi, Phi_Phi, BarR);

    // 成本函数：J = (y_ref - Fx - Phi*u)^T * Q * (y_ref - Fx - Phi*u) + u^T * R * u
    Eigen::MatrixXd Q = 10.0 * Eigen::MatrixXd::Identity(Np_ * 4, Np_ * 4); // 输出误差权重（4维输出×Np）
    Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(Nc_ * 6, Nc_ * 6); // 控制输入权重（6维输入×Nc）

    // 构建海森矩阵 H = 2*(Phi^T*Q*Phi + R)
    H_ = 2 * (Phi.transpose() * Q * Phi + R);

    // 构建梯度向量 f = -2*Phi^T*Q*(y_ref - Fx)
    Eigen::VectorXd y_ref = reference.replicate(Np_, 1); // 参考轨迹扩展为Np×4维
    Eigen::VectorXd Fx = F * x;
    f_ = -2 * Phi.transpose() * Q * (y_ref - Fx);

    // ----------------------
    // 步骤2：调用QpOASES求解QP问题
    // ----------------------
    qpOASES::real_t cpu_time = 0.01;  // 最大求解时间（10ms，满足实时性）
    qpOASES::returnValue ret;
    qpOASES::int_t nWSR = 300;

    // 首次求解（冷启动）或热启动（重用上次解）
    if (qp_solver_.isInitialised()) {
        // 热启动参数顺序：H, g, A, lb, ub, lbA, ubA, nWSR, cpu_time
        //int nWSR = 300;
        ret = qp_solver_.hotstart((qpOASES::real_t*)H_.data(), (qpOASES::real_t*)f_.data(), nullptr, (qpOASES::real_t*)lb_.data(), 
                                (qpOASES::real_t*)ub_.data(), nullptr, nullptr, nWSR , &cpu_time);
    } else {
        // 冷启动参数顺序与hotstart一致
        //int nWSR = 300;
        ret = qp_solver_.init((qpOASES::real_t*)H_.data(), (qpOASES::real_t*)f_.data(), nullptr, (qpOASES::real_t*)lb_.data(), 
                                (qpOASES::real_t*)ub_.data(), nullptr, nullptr, nWSR ,&cpu_time);
    }

    if (ret != qpOASES::SUCCESSFUL_RETURN) {
        throw std::runtime_error("QP solver failed to find solution");
    }

    // ----------------------
    // 步骤3：提取最优控制输入（取第一个时域步长的控制量）
    // ----------------------
    qpOASES::real_t u_opt[Nc_ * 6];
    qp_solver_.getPrimalSolution(u_opt);

    // 转换为Eigen向量，返回第一个6维控制输入（当前时刻的推力）
    Eigen::VectorXd du(6);
    for (int i = 0; i < 6; ++i) {
        du(i) = u_opt[i];
    }

    return du;
}

// End of MPCController class
}