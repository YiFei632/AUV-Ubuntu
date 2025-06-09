#ifndef AUV_CONTROL_KALMAN_FILTER_HPP
#define AUV_CONTROL_KALMAN_FILTER_HPP

#include <Eigen/Dense>

namespace auv_control {

class KalmanFilter {
public:
    /**
     * @brief 卡尔曼滤波构造函数
     * @param Q 过程噪声协方差(标量，将会扩展为对角矩阵)
     * @param R 观测噪声协方差(标量，将会扩展为对角矩阵)
     */
    KalmanFilter(double Q = 1.0, double R = 5.0);

    /**
     * @brief 执行滤波更新
     * @param measurement 新观测数据(6维位姿向量)
     * @return 滤波后的6维状态估计
     */
    Eigen::VectorXd filter(const Eigen::VectorXd& measurement);

private:
    Eigen::VectorXd x_hat_;    ///< 6维状态估计值
    Eigen::MatrixXd P_;        ///< 6x6估计误差协方差矩阵
    Eigen::MatrixXd Q_;        ///< 6x6过程噪声协方差矩阵
    Eigen::MatrixXd R_;        ///< 6x6观测噪声协方差矩阵
};

} // namespace auv_control

#endif // AUV_CONTROL_KALMAN_FILTER_HPP