#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import struct
from std_msgs.msg import Float32MultiArray

def main():
    """
    主函数：初始化ROS节点，打开串口，读取、解析并发布数据。
    """
    # 1. 初始化ROS节点
    rospy.init_node('serial_to_float_publisher', anonymous=True)

    # 2. 获取ROS参数
    # 从参数服务器获取串口号和波特率，如果未设置则使用默认值
    # 你可以在launch文件中或通过rosparam set来设置这些参数
    port_name = rospy.get_param('~port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baud', 9600)

    # 3. 创建Publisher
    # 创建一个Publisher，发布名为'/float_data'的话题，消息类型为Float32MultiArray
    pub = rospy.Publisher('/float_data', Float32MultiArray, queue_size=10)

    # 4. 配置并打开串口
    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=1  # 设置读取超时时间为1秒
        )
        rospy.loginfo("成功打开串口: %s @ %d" % (port_name, baud_rate))
    except serial.SerialException as e:
        rospy.logerr("无法打开串口 %s: %s" % (port_name, e))
        return

    # 设置循环频率为50Hz
    rate = rospy.Rate(50)

    # 5. 主循环：读取、解析和发布数据
    while not rospy.is_shutdown():
        # 检查串口中是否有足够的数据 (4个浮点数 * 4字节/浮点数 = 16字节)
        if ser.in_waiting >= 16:
            # 读取16个字节的数据
            serial_data = ser.read(16)

            try:
                # '<ffff' 表示小端模式（little-endian）的四个单精度浮点数
                # 如果你的设备使用大端模式，请使用 '>ffff'
                unpacked_data = struct.unpack('<ffff', serial_data)

                # 创建一个Float32MultiArray消息
                float_array_msg = Float32MultiArray()
                float_array_msg.data = unpacked_data

                # 发布消息
                pub.publish(float_array_msg)

                # 在终端打印解析出的数据，方便调试
                rospy.loginfo("发布的数据: %s" % str(unpacked_data))

            except struct.error as e:
                rospy.logwarn("解析数据时出错: %s。请检查数据格式和字节顺序。" % e)
                # 清空缓冲区，防止错误数据累积
                ser.reset_input_buffer()

        rate.sleep()

    # 关闭串口
    ser.close()
    rospy.loginfo("串口已关闭。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass