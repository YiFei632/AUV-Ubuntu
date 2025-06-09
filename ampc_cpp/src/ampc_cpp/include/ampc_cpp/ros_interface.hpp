#ifndef AUV_CONTROL_ROS_INTERFACE_HPP
#define AUV_CONTROL_ROS_INTERFACE_HPP

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <mutex>  // 新增：互斥锁头文件

namespace auv_control {

class ROSInterface {
public:
    ROSInterface();

    // 发布推力输出
    void publishThrust(const Eigen::VectorXd& thrust);

    // 新增：获取当前位姿（线程安全）
    Eigen::VectorXd getCurrentPose() const;

    Eigen::VectorXd getReference() const;

private:
    // 位姿回调函数
    void posePart1Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void posePart2Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void referenceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // 组合位姿数据（线程安全）
    void combinePoses();

    ros::NodeHandle nh_;
    ros::Subscriber sub_pose_part1_;  // 订阅位姿第一部分（depth, roll, pitch, yaw）
    ros::Subscriber sub_pose_part2_;  // 订阅位姿第二部分（x, y）
    ros::Subscriber sub_reference_;   // 订阅参考位姿
    ros::Publisher pub_thrust_;       // 发布推力输出

    // 新增：互斥锁保护共享数据
    mutable std::mutex pose_mutex_;
    Eigen::VectorXd current_pose_;    // 组合后的当前位姿（6维：x, y, depth, roll, pitch, yaw）
    std::vector<double> pose_part1_;  // 位姿第一部分数据（4维）
    std::vector<double> pose_part2_;  // 位姿第二部分数据（2维）

     // 新增：参考位姿相关成员
    mutable std::mutex reference_mutex_;  // 参考位姿互斥锁
    Eigen::VectorXd reference_;           // 存储参考位姿（维度与需求一致，如6维）
};

} // namespace auv_control

#endif // AUV_CONTROL_ROS_INTERFACE_HPP