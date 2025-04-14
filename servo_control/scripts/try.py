#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import ServoControl

class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller')
        
        # 参数配置
        self.base_move_time = 2000    # 基础运动时间2000ms
        self.slow_move_time = 3000    # 慢速运动时间
        self.fast_move_time = 500     # 快速运动时间
        self.sequence_delay = 0.5     # 动作间隔缓冲
        
        # 初始化串口
        self.init_serial()
        
        rospy.loginfo("四舵机控制节点已启动")
        self.run_control_sequence()
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
        """角度转脉冲（支持浮点数）"""
        return int((angle / 240.0) * 1000)

    def move_servo(self, servo_id, angle, move_time=None):
        """支持三档速度的舵机控制"""
        # 默认使用基础速度
        if move_time is None:
            move_time = self.base_move_time
            
        if angle < 0 or angle > 240:
            rospy.logwarn(f"舵机{servo_id}角度超出范围: {angle}°")
            return False
        
        pulse = self.angle_to_pulse(angle)
        try:
            ServoControl.setBusServoMove(
                servo_id=servo_id,
                servo_pulse=pulse,
                time=move_time
            )
            # 生成速度类型标签
            if move_time == self.fast_move_time:
                speed_tag = "快速"
            elif move_time == self.slow_move_time:
                speed_tag = "慢速"
            else:
                speed_tag = "常规"
            rospy.loginfo(f"舵机{servo_id} {speed_tag}运动到{angle:.1f}° ({move_time}ms)")
            return True
        except Exception as e:
            rospy.logerr(f"舵机{servo_id}控制失败: {str(e)}")
            return False

    def move_servos_sync(self, servo_list, angle_list, move_time):
        """多舵机同步控制"""
        try:
            # 生成舵机参数列表 [ID1, pulse1, ID2, pulse2...]
            params = []
            for sid, angle in zip(servo_list, angle_list):
                pulse = self.angle_to_pulse(angle)
                params.extend([sid, pulse])
            
            ServoControl.setMoreBusServoMove(
                servos=params,
                servos_count=len(servo_list),
                time=move_time
            )
            
            id_str = "+".join(map(str, servo_list))
            rospy.loginfo(f"舵机[{id_str}] 同步运动中... ({move_time}ms)")
            return True
        except Exception as e:
            rospy.logerr(f"同步控制失败: {str(e)}")
            return False
        
    def run_control_sequence(self):
        """三阶段运动控制"""
        # ===== 第一阶段：初始化运动 =====
        rospy.loginfo("开始初始运动序列...")
        
        # 4号常规速度，3号慢速
        self.move_servo(4, 150.0)
        self.move_servo(1, 64.8, self.base_move_time) 
        rospy.sleep(self.base_move_time/1000)
        self.move_servo(3, 103.2, self.slow_move_time)
        rospy.sleep(max(self.base_move_time, self.slow_move_time)/1000 + self.sequence_delay)
        
        # 定位 
        self.move_servo(1, 130, self.base_move_time) 
        rospy.sleep(self.base_move_time/1000)
        self.move_servo(2, 90)
        rospy.sleep(self.fast_move_time/500)
        self.move_servo(3, 115.2, self.fast_move_time)
        rospy.sleep(self.fast_move_time/500)
        # ===== 第二阶段：快速运动序列 =====
        rospy.loginfo("执行快速附加运动...")
        fast_actions = [
            (4, 60, self.base_move_time),
            (3, 103.2, self.fast_move_time),
            (2, 120, self.fast_move_time),
            (1, 69.6, self.base_move_time)
        ]
        
        for action in fast_actions:
            servo_id, angle, move_time = action
            if self.move_servo(servo_id, angle, move_time):
                rospy.sleep(move_time/1000 + 0.3)

        # ===== 第三阶段：回正运动 =====
        rospy.loginfo("开始回正运动序列...")
        return_actions = [
            # 2号和3号同步回正（使用3号的慢速时间）
            {"type": "sync", "ids": [2,3], "angles": [120.0,217.2], "time": self.slow_move_time},
            # 1号和4号同步回正
            {"type": "sync", "ids": [1,4], "angles": [145.2,120.0], "time": self.base_move_time}
        ]

        for action in return_actions:
            if action["type"] == "sync":
                success = self.move_servos_sync(
                    action["ids"], 
                    action["angles"],
                    action["time"]
                )
                delay = action["time"]/1000
                if success:
                    rospy.sleep(delay + self.sequence_delay)
        # rospy.loginfo("开始回正运动序列...")
        # return_actions = [
        #     # 单舵机运动
        #     {"type": "single", "id": 3, "angle": 217.2, "time": self.slow_move_time},
        #     {"type": "single", "id": 2, "angle": 120.0, "time": self.base_move_time},
        #     # 双舵机同步
        #     {"type": "sync", "ids": [1,4], "angles": [145.2, 120.0], "time": self.base_move_time}
        # ]
        
        # for action in return_actions:
        #     if action["type"] == "single":
        #         success = self.move_servo(action["id"], action["angle"], action["time"])
        #         delay = action["time"]/1000
        #     elif action["type"] == "sync":
        #         success = self.move_servos_sync(
        #             action["ids"], 
        #             action["angles"],
        #             action["time"]
        #         )
        #         delay = action["time"]/1000
                
        #     if success:
        #         rospy.sleep(delay + self.sequence_delay)

        rospy.loginfo("完整运动序列执行完成")

    def angle_callback(self, msg):
        """实时控制接口"""
        self.move_servo(1, msg.data)

    def shutdown(self):
        """资源清理"""
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