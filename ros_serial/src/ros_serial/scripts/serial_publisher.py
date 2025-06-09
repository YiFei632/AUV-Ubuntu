#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import struct
import serial  # 新增串口库
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

def main():
    # 初始化ROS节点
    rospy.init_node('float_to_serial_publisher')
    
    # 配置串口参数（根据实际设备调整）
    serial_port = '/dev/ttyUSB1'  # 用户指定的串口
    baudrate = 115200             # 常用波特率，需与设备一致
    timeout = 0.1                 # 读取超时时间（这里主要用发送功能）
    
    # 初始化串口（添加异常处理）
    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
        rospy.loginfo(f"成功打开串口：{serial_port}，波特率：{baudrate}")
    except serial.SerialException as e:
        rospy.logerr(f"无法打开串口 {serial_port}: {e}")
        return  # 串口打开失败时退出节点
    
    # 创建ROS发布者（可选保留，用于调试）
    pub = rospy.Publisher('~byte_array_topic', ByteMultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 发布频率
    
    # 预留的4个单精度浮点数（示例数据）
    reserved_floats = [1.0, 2.5, 3.14, 42.0]  # 实际使用时修改
    
    try:
        while not rospy.is_shutdown():
            # 打包浮点数为二进制数据
            packed_data = struct.pack('ffff', *reserved_floats)  # 4个float→16字节
            
            # 添加首尾标记（0x77和0xFF各1字节）
            full_byte_data = b'\x77' + packed_data + b'\xff'  # 总18字节
            
            # 串口发送（直接发送字节对象）
            try:
                ser.write(full_byte_data)
                rospy.loginfo(f"已通过串口发送{len(full_byte_data)}字节数据")
            except serial.SerialException as e:
                rospy.logerr(f"串口发送失败: {e}")
            
            # 可选：保留ROS话题发布（用于调试）
            uint8_array = list(full_byte_data)  # 转换为uint8列表
            byte_msg = ByteMultiArray(
                layout=MultiArrayLayout(
                    dim=[MultiArrayDimension(label="byte_array", size=len(uint8_array))],
                    data_offset=0
                ),
                data=uint8_array
            )
            pub.publish(byte_msg)
            
            rate.sleep()
    finally:
        # 确保退出时关闭串口
        if ser.is_open:
            ser.close()
            rospy.loginfo("串口已关闭")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    