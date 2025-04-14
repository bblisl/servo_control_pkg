#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import ServoControl

class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller')
        
        # 参数配置
        self.move_time = 2000  # 统一运动时间2000ms（慢速）
        self.sequence_delay = 0.8  # 动作间隔缓冲
        
        # 初始化串口
        self.init_serial()
        
        rospy.loginfo("四舵机控制节点已启动")
        
        # 执行主控制序列
        self.run_control_sequence()
        
        # 保留话题订阅功能（可扩展为多ID控制）
        rospy.Subscriber('/servo_angle', Float32, self.angle_callback)

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

    def move_servo(self, servo_id, angle):
        """通用舵机控制函数"""
        if angle < 0 or angle > 240:
            rospy.logwarn(f"舵机{servo_id}角度超出范围: {angle}°")
            return False
        
        pulse = self.angle_to_pulse(angle)
        try:
            ServoControl.setBusServoMove(
                servo_id=servo_id,
                servo_pulse=pulse,
                time=self.move_time
            )
            rospy.loginfo(f"舵机{servo_id}运动到{angle}° 成功")
            return True
        except Exception as e:
            rospy.logerr(f"舵机{servo_id}控制失败: {str(e)}")
            return False

    def run_control_sequence(self):
        """执行完整的控制序列"""
        # 第一阶段：初始运动
        seq_forward = [
            (1, 120),  # 夹爪舵机
            (2, 150),  # 手腕舵机
            (3, 167),  # 肘关节
            (4, 130)   # 旋转底座
        ]
        
        # 第二阶段：返回运动
        seq_return = [
            (1, 145),
            (2, 120),
            (3, 217),
            (4, 120)
        ]
        
        # 执行正向序列
        rospy.loginfo("开始执行正向运动序列...")
        for servo_id, angle in seq_forward:
            if self.move_servo(servo_id, angle):
                rospy.sleep(self.move_time/1000 + self.sequence_delay)
        
        # 执行返回序列
        rospy.loginfo("开始执行返回运动序列...")
        for servo_id, angle in seq_return:
            if self.move_servo(servo_id, angle):
                rospy.sleep(self.move_time/1000 + self.sequence_delay)
        
        rospy.loginfo("完整运动序列执行完成")

    def angle_callback(self, msg):
        """保留话题控制功能（示例控制1号舵机）"""
        self.move_servo(1, msg.data)

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