#include "ampc_cpp/ros_interface.hpp"
#include <stdexcept>

namespace auv_control {

ROSInterface::ROSInterface() {
    // 初始化订阅和发布
    sub_pose_part1_ = nh_.subscribe("/pose_part1", 10, &ROSInterface::posePart1Callback, this);
    sub_pose_part2_ = nh_.subscribe("/pose_part2", 10, &ROSInterface::posePart2Callback, this);
    sub_reference_ = nh_.subscribe("/pose_reference", 10, &ROSInterface::referenceCallback, this);
    pub_thrust_ = nh_.advertise<std_msgs::Float64MultiArray>("/thrust_output", 10);

    // 初始化位姿为0（6维）
    current_pose_ = Eigen::VectorXd::Zero(6);
    reference_ = Eigen::VectorXd::Zero(6);
    
    // 初始化位姿数据容器
    pose_part1_.resize(4, 0.0);
    pose_part2_.resize(2, 0.0);
}

void ROSInterface::posePart1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_); // 加锁保护
    if (msg->data.size() >= 4) {
        pose_part1_ = std::vector<double>(msg->data.begin(), msg->data.begin() + 4);
        combinePoses(); // 数据更新后重新组合位姿
    } else {
        ROS_WARN("Received pose_part1 with insufficient data: %zu elements", msg->data.size());
    }
}

void ROSInterface::posePart2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_); // 加锁保护
    if (msg->data.size() >= 2) {
        pose_part2_ = std::vector<double>(msg->data.begin(), msg->data.begin() + 2);
        combinePoses(); // 数据更新后重新组合位姿
    } else {
        ROS_WARN("Received pose_part2 with insufficient data: %zu elements", msg->data.size());
    }
}

void ROSInterface::referenceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(reference_mutex_); // 加锁保护
    if (msg->data.size() >= 6) {
        // 假设参考位姿为6维：[x, y, depth, roll, pitch, yaw]
        for (int i = 0; i < 6; ++i) {
            reference_(i) = msg->data[i];
        }
    } else {
        ROS_WARN("Received reference with insufficient data: %zu elements", msg->data.size());
    }
}

void ROSInterface::combinePoses() {
    // 校验数据维度（pose_part1_应为4维，pose_part2_应为2维）
    if (pose_part1_.size() != 4 || pose_part2_.size() != 2) {
        ROS_WARN("Invalid pose data dimensions: part1=%zu, part2=%zu", 
                 pose_part1_.size(), pose_part2_.size());
        return;
    }

    // 组合位姿：[x, y, depth, roll, pitch, yaw]
    current_pose_ << 
        pose_part2_[0],  // x
        pose_part2_[1],  // y
        pose_part1_[0],  // depth
        pose_part1_[1],  // roll
        pose_part1_[2],  // pitch
        pose_part1_[3];  // yaw
}

void ROSInterface::publishThrust(const Eigen::VectorXd& thrust) {
    std_msgs::Float64MultiArray msg;
    msg.data.assign(thrust.data(), thrust.data() + thrust.size());
    pub_thrust_.publish(msg);
}

// 线程安全的位姿获取方法
Eigen::VectorXd ROSInterface::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(pose_mutex_); // 加锁保护
    return current_pose_;
}

// 参考位姿获取方法
Eigen::VectorXd ROSInterface::getReference() const {
    std::lock_guard<std::mutex> lock(reference_mutex_); // 加锁保护
    return reference_;
}

} // namespace auv_control