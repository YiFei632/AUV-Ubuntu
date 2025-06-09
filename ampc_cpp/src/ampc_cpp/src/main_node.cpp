#include <Eigen/Dense>  // 必须先包含Eigen头文件（关键！）
#include <ros/ros.h>
#include "ampc_cpp/ros_interface.hpp"
#include "ampc_cpp/mpc.hpp"
#include "ampc_cpp/adaptive.hpp"
#include "ampc_cpp/kalman_filter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "auv_control_node");
    auv_control::ROSInterface ros_if;
    auv_control::MPCController mpc(10,10); // Nc=10, Np=10
    auv_control::AdaptiveController adaptive(0.02,600); // lambda=0.02, steps=600
    auv_control::KalmanFilter kalman(1.0,5.0); // Q=1, R=5
    Eigen::VectorXd pose_last = Eigen::VectorXd::Zero(6);

    ros::Rate rate(10); // 10Hz控制频率
    while (ros::ok()) {
        ros::spinOnce();
        
        // 获取当前位姿和参考值（假设已组合为Eigen向量）
        Eigen::VectorXd pose = ros_if.getCurrentPose();
        Eigen::VectorXd d_pose = pose - pose_last;
        pose_last = pose; // 更新上次位姿
        Eigen::VectorXd reference = ros_if.getReference();
        
        // 运行自适应算法
        // 使用卡尔曼滤波器进行状态估计
        Eigen::VectorXd estimated_state = kalman.filter(pose);
        adaptive.updateParameters(pose, estimated_state, d_pose); // 需实现状态估计
        

        // 运行MPC算法
        Eigen::VectorXd du = mpc.computeControlInput(pose, reference);
        
        // 发布输出（六自由度推力和推力矩）
        ros_if.publishThrust(du); // 假设du为六元组
        
        rate.sleep();
    }
    return 0;
}