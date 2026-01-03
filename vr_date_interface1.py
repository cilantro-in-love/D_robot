from motors.feetech import FeetechMotorsBus
from motors.configs import FeetechMotorsBusConfig
from motors.feetech import TorqueMode
from vr_monitor import VRMonitor
import time
import threading
import asyncio
import numpy as np
import math
import pygame



def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
    
    Parameters:
        x: End effector x coordinate
        y: End effector y coordinate
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1350 m)
        
    Returns:
        joint2, joint3: Joint angles in radians as defined in the URDF file
    """
    # Calculate joint2 and joint3 offsets in theta1 and theta2
    theta1_offset = -math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
    theta2_offset = -math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0
    
    # Calculate distance from origin to target point
    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2  # Maximum reachable distance
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if r > r_max:
        scale_factor = r_max / r
        x *= scale_factor
        y *= scale_factor
        r = r_max
    
    # If target point is less than minimum workspace (|l1-l2|), scale it
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale_factor = r_min / r
        x *= scale_factor
        y *= scale_factor
        r = r_min
    
    # Use law of cosines to calculate theta2
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    
    # Calculate theta2 (elbow angle)
    theta2 = math.pi - math.acos(cos_theta2)
    
    # Calculate theta1 (shoulder angle)
    beta = math.atan2(y, x)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma
    
    # Convert theta1 and theta2 to joint2 and joint3 angles
    joint2 = theta1 - theta1_offset
    joint3 = theta2 - theta2_offset
    
    # Ensure angles are within URDF limits
    joint2 = max(-0.1, min(3.45, joint2))
    joint3 = max(-0.2, min(math.pi, joint3))
    
    return joint2, joint3



class VrDateInterface:
    def __init__(self):
        self.reference_set = False
        self.left_reference_pos = None
        self.left_reference_angles = {'roll': 0, 'pitch': 0}
        self.vr_monitor = VRMonitor()
        self.k_motor = 1024/90
                # 初始化XYZ值为0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def process_robot_data(self, robot_pos):
        """处理机器人数据"""
        l4 = 0.15
        robot_position = [0,0.29,0.116,0,0]
        # TODO: 根据机器人位置更新VR手柄位置
        robot_angel = np.zeros(5)
        robot_position = np.array(robot_position)
        robot_pos = robot_position + robot_pos
        x,y,z,wrist_flex,wrist_roll = robot_pos
        r_distance = math.sqrt(x**2 + y**2)
        rotation_angle = math.atan2(x, y)
        z = z - math.sin(wrist_flex)*l4
        r_distance = r_distance - math.cos(wrist_flex)*l4
        robot_angel[0] = rotation_angle
        
        robot_angel[4] = -wrist_roll
        robot_angel[1],robot_angel[2] = inverse_kinematics(r_distance,z)
        robot_angel[3] = -wrist_flex*math.pi/180+robot_angel[2]-robot_angel[1]
        return robot_angel
    
    def limit_range_pos(self, pos):
        """限制位置在合理范围内"""
        l1 = 0.1159
        l2 = 0.1350
        l3 = 0.1500
        l_base = 0.1
        r1 = (l1 + l2 + l3)*0.9
        r2 = (l2 + l3)*0.9
        axis_pos = np.array([0,l2+l3,l1,0,0])
        pos = axis_pos + np.array(pos)
        if pos[1] < 0:
            pos[1] = 0
        if pos[2] >= 0:
            d = math.sqrt(pos[2]**2 + pos[0]**2 + pos[1]**2)
            if  d > r1:
                pos[0] = pos[0] * r1 / d
                pos[1] = pos[1] * r1 / d
                pos[2] = pos[2] * r1 / d
                pos = np.array(pos)-axis_pos
                return pos
            else:
                pos = np.array(pos)-axis_pos
                return pos
        elif pos[2] < 0 and pos[2] >= -l_base:
            d1 = math.sqrt(pos[0]**2 + pos[1]**2)
            x1 = (pos[0] / d1) * l2
            y1 = (pos[1] / d1) * l2
            d = math.sqrt((pos[0] - x1)**2 + (pos[1] - y1)**2 + pos[2]**2)
            if  d > r2:
                pos[0] = (pos[0] - x1) * r2 / d + x1
                pos[1] = (pos[1] - y1) * r2 / d + y1
                pos[2] = pos[2] * r2 / d
                pos = np.array(pos)-axis_pos
                return pos
            else:
                pos = np.array(pos)-axis_pos
                return pos
        else:
            d1 = math.sqrt(pos[0]**2 + pos[1]**2)
            x1 = (pos[0] / d1) * l2
            y1 = (pos[1] / d1) * l2
            d = math.sqrt((pos[0] - x1)**2 + (pos[1] - y1)**2 + pos[2]**2)
            if  d > r2:
                pos[0] = (pos[0] - x1) * r2 / d + x1
                pos[1] = (pos[1] - y1) * r2 / d + y1
                pos[2] = -0.1
                pos = np.array(pos)-axis_pos
                return pos
            else:
                pos = np.array(pos)-axis_pos
                return pos


    def set_vr_reference(self):
        """设置VR手柄当前位置为参考位置"""
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        if not dual_goals:
            print("❌ 无法获取VR数据，无法设置参考位置")
            return False
        
        left_goal = dual_goals.get("left")
        
        # 设置左手参考位置
        if left_goal and left_goal.target_position is not None:
            self.left_reference_pos = np.array(left_goal.target_position)
            if left_goal.wrist_roll_deg is not None:
                self.left_reference_angles['roll'] = left_goal.wrist_roll_deg
            if left_goal.wrist_flex_deg is not None:
                self.left_reference_angles['pitch'] = left_goal.wrist_flex_deg
            print(f"✅ 左手参考位置设置: {self.left_reference_pos}")
    
        
        if self.left_reference_pos is not None:
            self.reference_set = True
            print("🎯 VR参考位置设置完成！现在可以开始相对控制")
            return True
        else:
            print("❌ 无法检测到VR手柄，请确保VR系统正常运行")
            return False
    
    def process_vr_data(self):
        """处理VR数据 - 修正版相对位置控制"""
        
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        print(dual_goals)
        if not dual_goals:
            print("❌ 无法获取VR数据，无法设置参考位置")
            return False
        left_goal = dual_goals.get("left")

        if left_goal and left_goal.target_position is not None:
            left_vr_pos = np.array(left_goal.target_position)
            roll_deg = left_goal.wrist_roll_deg - self.left_reference_angles['roll']
            flex_deg = left_goal.wrist_flex_deg - self.left_reference_angles['pitch']
            flex_deg = -flex_deg
            left_vr_pos = left_vr_pos - self.left_reference_pos
            robot_pos = [left_vr_pos[0], -left_vr_pos[2], left_vr_pos[1],flex_deg,roll_deg]
            # ax = [robot_pos[0], robot_pos[1], robot_pos[2]]
            r_distance = math.sqrt(robot_pos[0]**2 + robot_pos[1]**2)
            r_distance_ = r_distance - math.cos(flex_deg * math.pi / 180)*0.1
            print(robot_pos[0],robot_pos[1],robot_pos[2])
            self.x = robot_pos[0] * r_distance_ / r_distance
            self.y = robot_pos[1] * r_distance_ / r_distance
            self.z = robot_pos[2] - math.sin(flex_deg * math.pi / 180)*0.1
            robot_pos = np.array(robot_pos)
            print(self.x,self.y,self.z)
            print(flex_deg)
            print(math.cos(flex_deg * math.pi / 180))
            print(self.x-robot_pos[0],"  ",self.y-robot_pos[1],"  ",self.z-robot_pos[2])
        return robot_pos


    def main(self):


        self.vr_monitor = VRMonitor()
        vr_thread = threading.Thread(
            target=lambda: asyncio.run(self.vr_monitor.start_monitoring()), 
            daemon=True
        )
        vr_thread.start()
        while self.vr_monitor.get_latest_goal_nowait().get("left") is None:
            time.sleep(1)
            print("等待VR手柄初始化...")
        print("VR手柄初始化完成")
        time.sleep(3)
        self.set_vr_reference()
        while True:
            position = self.process_vr_data()
            robot_pos = [self.x, self.y, self.z, position[3], position[4]]
            robot_pos = self.limit_range_pos(robot_pos)
            robot_angel = self.process_robot_data(robot_pos)

            time.sleep(0.01)
 

if __name__ == "__main__":
    vr_date_interface = VrDateInterface()
    vr_date_interface.main()