#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import ServoControl

class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller')
        
        # 参数配置
        self.servo_id = rospy.get_param('~servo_id', 1)       # 默认ID=1
        self.move_time = rospy.get_param('~move_time', 1000)  # 默认1000ms
        
        # 订阅角度话题
        rospy.Subscriber('/servo_angle', Float32, self.angle_callback)
        
        # 初始化串口
        self.init_serial()
        
        rospy.loginfo(f"舵机控制节点已启动，ID={self.servo_id}")

    def init_serial(self):
        """检查并打开串口"""
        try:
            if not ServoControl.serialHandle.is_open:
                ServoControl.serialHandle.open()
            rospy.loginfo("串口初始化成功")
        except Exception as e:
            rospy.logerr(f"串口初始化失败: {str(e)}")
            rospy.signal_shutdown("硬件故障")

    def angle_to_pulse(self, angle):
        """角度(0~240°)转脉冲值(0~1000)"""
        return int((angle / 240.0) * 1000)

    def angle_callback(self, msg):
        """处理角度指令"""
        target_angle = msg.data
        if target_angle < 0 or target_angle > 240:
            rospy.logwarn(f"角度超出范围: {target_angle}°")
            return
        
        # 转换并发送指令
        pulse = self.angle_to_pulse(target_angle)
        try:
            ServoControl.setBusServoMove(
                servo_id=self.servo_id,
                servo_pulse=pulse,
                time=self.move_time
            )
            rospy.loginfo(f"指令发送成功: ID={self.servo_id}, 角度={target_angle}°, 脉冲={pulse}")
        except Exception as e:
            rospy.logerr(f"指令发送失败: {str(e)}")

    def shutdown(self):
        """关闭节点时清理资源"""
        if ServoControl.serialHandle.is_open:
            ServoControl.serialHandle.close()
        rospy.loginfo("节点已关闭")

if __name__ == '__main__':
    try:
        controller = ServoController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass