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

# ================================
# 机械臂参数配置 - 可自定义
# ================================
ARM_PARAMETERS = {
    'base_height': 0.1,        # 底座高度 (m)
    'shoulder_length': 0.15,   # 肩部到肘部长度 (大臂长度)
    'elbow_length': 0.12,      # 肘部到腕部长度 (小臂长度)
    # 腕部和夹爪不需要长度定义，因为它们主要负责姿态调整
}

# 配置串口参数和电机信息
config1 = FeetechMotorsBusConfig(
    port="/dev/ttyACM0",  # 左机械臂串口
    motors={
        "base_rotation": [1, "sts3215"],     # 底盘旋转
        "shoulder": [2, "sts3215"],          # 肩部关节
        "elbow": [3, "sts3215"],             # 肘部关节
        "wrist_pitch": [4, "sts3215"],       # 腕部俯仰
        "wrist_roll": [5, "sts3215"],        # 腕部旋转
        "gripper": [6, "sts3215"],           # 夹爪
    }
)

config2 = FeetechMotorsBusConfig(
    port="/dev/ttyACM0",  # 右机械臂串口
    motors={
        "base_rotation": [11, "sts3215"],      # 底盘旋转
        "shoulder": [12, "sts3215"],           # 肩部关节
        "elbow": [13, "sts3215"],              # 肘部关节  
        "wrist_pitch": [14, "sts3215"],        # 腕部俯仰
        "wrist_roll": [15, "sts3215"],         # 腕部旋转
        "gripper": [16, "sts3215"],            # 夹爪
    }
)

class RobotArm:
    """6自由度机械臂控制类 - 修正版"""
    
    def __init__(self, config, name, arm_params=ARM_PARAMETERS):
        self.name = name
        self.motors_bus = FeetechMotorsBus(config)
        
        # 舵机转换常数
        self.SERVO_CENTER = 2048  # 舵机中位 (0度)
        self.SERVO_SCALE = 1000.0 / 90.0  # 舵机单位/度 (≈11.111)
        
        # 机械臂物理参数
        self.arm_params = arm_params.copy()
        self.base_height = arm_params['base_height']
        self.L1 = arm_params['shoulder_length']  # 大臂长度
        self.L2 = arm_params['elbow_length']     # 小臂长度
        
        print(f"{self.name} 机械臂参数:")
        print(f"  底座高度: {self.base_height:.3f}m")
        print(f"  大臂长度: {self.L1:.3f}m") 
        print(f"  小臂长度: {self.L2:.3f}m")
        print(f"  最大伸展: {self.L1 + self.L2:.3f}m")
        print(f"  最小收缩: {abs(self.L1 - self.L2):.3f}m")
        
        # 关节角度限制 (舵机位置值，2048为中位0度)
        self.joint_limits = {
            'base_rotation': (548, 3548),        # 底盘旋转 ±135度
            'shoulder': (1048, 3048),            # 肩部 -90度到+90度 (ZY平面内)
            'elbow': (1048, 3048),               # 肘部 -90度到+90度 (ZY平面内)
            'wrist_pitch': (1048, 3048),         # 腕部俯仰 -90度到+90度 (ZY平面内)
            'wrist_roll': (548, 3548),           # 腕部旋转 ±135度
            'gripper': (1400, 2800)              # 夹爪开合范围
        }
        
        # 当前关节位置
        self.current_positions = {}
        self.target_positions = {}
        
        # 初始化目标位置为中位
        for joint in self.joint_limits:
            if joint == 'gripper':
                self.target_positions[joint] = 1800  # 夹爪稍微闭合
            else:
                self.target_positions[joint] = self.SERVO_CENTER  # 其他关节中位
    
    def angle_to_servo(self, angle_deg):
        """将角度(度)转换为舵机位置值"""
        return int(self.SERVO_CENTER + angle_deg * self.SERVO_SCALE)
    
    def servo_to_angle(self, servo_pos):
        """将舵机位置值转换为角度(度)"""
        return (servo_pos - self.SERVO_CENTER) / self.SERVO_SCALE
    
    def move_to_home_position(self):
        """移动到初始位置"""
        print(f"{self.name} 移动到初始位置...")
        for joint in self.target_positions:
            if joint == 'gripper':
                self.target_positions[joint] = 2500
            else:
                self.target_positions[joint] = self.SERVO_CENTER
        
        return self.write_positions()
    
    def connect(self):
        """连接机械臂"""
        self.motors_bus.connect()
        self.motors_bus.write("Torque_Enable", TorqueMode.ENABLED.value)
        print(f"{self.name} 机械臂已连接")
    
    def disconnect(self):
        """断开连接"""
        self.motors_bus.disconnect()
        print(f"{self.name} 机械臂已断开连接")
    
    def read_positions(self):
        """读取当前关节位置"""
        try:
            positions = self.motors_bus.read("Present_Position")
            joint_names = list(self.joint_limits.keys())
            for i, joint in enumerate(joint_names):
                if i < len(positions):
                    self.current_positions[joint] = positions[i]
            return self.current_positions
        except Exception as e:
            print(f"读取{self.name}位置失败: {e}")
            return self.current_positions
    
    def write_positions(self):
        """写入目标位置到机械臂"""
        try:
            # 限制关节角度在安全范围内
            clamped_positions = []
            for joint in self.joint_limits:
                pos = self.target_positions[joint]
                min_pos, max_pos = self.joint_limits[joint]
                clamped_pos = max(min_pos, min(max_pos, int(pos)))
                clamped_positions.append(clamped_pos)
            
            self.motors_bus.write("Goal_Position", clamped_positions)
            return True
        except Exception as e:
            print(f"写入{self.name}位置失败: {e}")
            return False
    
    def inverse_kinematics_zy_plane(self, r, z, end_effector_angle_deg=0):
        """
        在ZY平面内的逆运动学求解
        
        参数:
            r: 距离底盘旋转轴的水平距离 (在ZY平面内，相当于Y方向距离)
            z: 距离底盘的垂直高度 (Z方向)
            end_effector_angle_deg: 末端执行器期望角度 (度)
        
        返回:
            (shoulder_angle, elbow_angle, wrist_pitch_angle): 肩部、肘部、腕部俯仰角度 (度)
        """
        try:
            # 1. 调整目标点，考虑底座高度
            target_z = z - self.base_height
            target_r = r
            
            # 2. 检查工作空间限制
            distance_to_target = math.sqrt(target_r**2 + target_z**2)
            max_reach = self.L1 + self.L2
            min_reach = abs(self.L1 - self.L2)
            
            if distance_to_target > max_reach:
                print(f"目标点超出最大工作空间: {distance_to_target:.3f}m > {max_reach:.3f}m")
                # 缩放到边界
                scale = max_reach / distance_to_target
                target_r *= scale
                target_z *= scale
                distance_to_target = max_reach
            
            if distance_to_target < min_reach:
                print(f"目标点小于最小工作空间: {distance_to_target:.3f}m < {min_reach:.3f}m")
                # 缩放到边界
                if distance_to_target > 0:
                    scale = min_reach / distance_to_target
                    target_r *= scale
                    target_z *= scale
                    distance_to_target = min_reach
                else:
                    # 如果距离为0，设置为最小可达点
                    target_r = min_reach
                    target_z = 0
                    distance_to_target = min_reach
            
            # 3. 使用余弦定理计算肘关节角度
            cos_elbow = (self.L1**2 + self.L2**2 - distance_to_target**2) / (2 * self.L1 * self.L2)
            cos_elbow = max(-1.0, min(1.0, cos_elbow))  # 限制范围
            
            # 肘关节角度 (内角，0度表示完全伸直，正角度表示弯曲)
            elbow_angle_rad = math.acos(cos_elbow)
            elbow_angle_deg = math.degrees(elbow_angle_rad)
            
            # 4. 计算肩关节角度
            # 目标点相对于肩关节的角度
            angle_to_target = math.atan2(target_z, target_r)
            
            # 大臂与目标方向的偏移角
            cos_offset = (self.L1**2 + distance_to_target**2 - self.L2**2) / (2 * self.L1 * distance_to_target)
            cos_offset = max(-1.0, min(1.0, cos_offset))
            offset_angle = math.acos(cos_offset)
            
            # 肩关节角度 (相对于Y轴正方向)
            shoulder_angle_rad = angle_to_target - offset_angle
            shoulder_angle_deg = math.degrees(shoulder_angle_rad)
            
            # 5. 计算腕部俯仰角度
            # 当前小臂的角度
            forearm_angle_rad = shoulder_angle_rad + elbow_angle_rad
            
            # 为了使末端执行器达到期望角度，腕部需要的补偿角度
            desired_end_angle_rad = math.radians(end_effector_angle_deg)
            wrist_pitch_angle_rad = desired_end_angle_rad - forearm_angle_rad
            wrist_pitch_angle_deg = math.degrees(wrist_pitch_angle_rad)
            
            # 6. 角度范围检查和限制
            shoulder_angle_deg = max(-90, min(90, shoulder_angle_deg))
            elbow_angle_deg = max(-90, min(90, elbow_angle_deg))
            wrist_pitch_angle_deg = max(-90, min(90, wrist_pitch_angle_deg))
            
            return shoulder_angle_deg, elbow_angle_deg, wrist_pitch_angle_deg
            
        except Exception as e:
            print(f"逆运动学计算失败: {e}")
            return 0, 0, 0
    
    def cartesian_to_joint_angles(self, x, y, z, roll_deg=0, end_effector_angle_deg=0):
        """
        笛卡尔坐标到关节角度的逆运动学求解
        
        参数:
            x, y, z: 目标位置 (米)
            roll_deg: 腕部旋转角度 (度)
            end_effector_angle_deg: 末端执行器在ZY平面内的期望角度 (度)
        
        返回:
            关节角度字典 (舵机位置值)
        """
        try:
            # 1. 计算底盘旋转角度 (关节1)
            base_rotation_rad = math.atan2(y, x)
            base_rotation_deg = math.degrees(base_rotation_rad)
            
            # 2. 计算在ZY平面内的距离
            r_horizontal = math.sqrt(x**2 + y**2)  # 水平距离
            
            # 3. 在ZY平面内求解逆运动学 (关节2、3、4)
            shoulder_deg, elbow_deg, wrist_pitch_deg = self.inverse_kinematics_zy_plane(
                r_horizontal, z, end_effector_angle_deg
            )
            
            # 4. 转换为舵机位置值
            positions = {
                'base_rotation': self.angle_to_servo(base_rotation_deg),
                'shoulder': self.angle_to_servo(shoulder_deg),
                'elbow': self.angle_to_servo(elbow_deg),
                'wrist_pitch': self.angle_to_servo(wrist_pitch_deg),
                'wrist_roll': self.angle_to_servo(roll_deg),
                'gripper': self.target_positions['gripper']  # 保持当前夹爪位置
            }
            
            # 5. 调试信息
            if self.name == "左臂":  # 只为一个臂打印调试信息，避免刷屏
                print(f"{self.name} 逆运动学结果:")
                print(f"  目标位置: ({x:.3f}, {y:.3f}, {z:.3f})")
                print(f"  底盘旋转: {base_rotation_deg:.1f}°")
                print(f"  ZY平面距离: r={r_horizontal:.3f}m, z={z:.3f}m")
                print(f"  关节角度: 肩部={shoulder_deg:.1f}°, 肘部={elbow_deg:.1f}°, 腕部={wrist_pitch_deg:.1f}°")
            
            return positions
            
        except Exception as e:
            print(f"逆运动学计算失败: {e}")
            return None
    
    def get_end_effector_position(self):
        """
        正运动学：根据当前关节角度计算末端执行器位置
        
        返回:
            (x, y, z): 末端执行器在笛卡尔坐标系中的位置
        """
        try:
            # 读取当前关节角度
            base_rotation_deg = self.servo_to_angle(self.current_positions.get('base_rotation', 2048))
            shoulder_deg = self.servo_to_angle(self.current_positions.get('shoulder', 2048))
            elbow_deg = self.servo_to_angle(self.current_positions.get('elbow', 2048))
            wrist_pitch_deg = self.servo_to_angle(self.current_positions.get('wrist_pitch', 2048))
            
            # 转换为弧度
            base_rotation_rad = math.radians(base_rotation_deg)
            shoulder_rad = math.radians(shoulder_deg)
            elbow_rad = math.radians(elbow_deg)
            
            # 在ZY平面内计算位置
            # 肩关节位置
            shoulder_r = 0  # 肩关节在底盘旋转轴上
            shoulder_z = self.base_height
            
            # 肘关节位置 (在ZY平面内)
            elbow_r = shoulder_r + self.L1 * math.cos(shoulder_rad)
            elbow_z = shoulder_z + self.L1 * math.sin(shoulder_rad)
            
            # 腕关节位置 (在ZY平面内)
            forearm_angle = shoulder_rad + elbow_rad
            wrist_r = elbow_r + self.L2 * math.cos(forearm_angle)
            wrist_z = elbow_z + self.L2 * math.sin(forearm_angle)
            
            # 转换到3D笛卡尔坐标系
            x = wrist_r * math.cos(base_rotation_rad)
            y = wrist_r * math.sin(base_rotation_rad)
            z = wrist_z
            
            return x, y, z
            
        except Exception as e:
            print(f"正运动学计算失败: {e}")
            return 0, 0, 0

class VRRobotController:
    """VR机械臂控制器主类 - 修正版"""
    
    def __init__(self):
        # 初始化机械臂
        self.left_arm = RobotArm(config1, "左臂")
        self.right_arm = RobotArm(config2, "右臂")
        
        # 初始化VR监控器
        self.vr_monitor = VRMonitor()
        
        # 控制参数
        self.vr_scale = 0.8  # VR坐标到实际坐标的缩放
        self.position_filter_alpha = 0.15  # 位置滤波系数
        self.angle_filter_alpha = 0.25     # 角度滤波系数
        
        # VR参考位置控制
        self.reference_set = False
        self.left_reference_pos = None
        self.right_reference_pos = None
        self.left_reference_angles = {'roll': 0, 'pitch': 0}
        self.right_reference_angles = {'roll': 0, 'pitch': 0}
        
        # 机械臂参考位置 (相对安全的初始位置)
        self.left_arm_reference_pos = np.array([0.15, -0.1, 0.25])   # 左臂初始位置
        self.right_arm_reference_pos = np.array([0.15, 0.1, 0.25])   # 右臂初始位置
        
        # 滤波后的目标位置
        self.filtered_left_pos = self.left_arm_reference_pos.copy()
        self.filtered_right_pos = self.right_arm_reference_pos.copy()
        self.filtered_left_angles = {'roll': 0, 'pitch': 0, 'end_effector': 0}
        self.filtered_right_angles = {'roll': 0, 'pitch': 0, 'end_effector': 0}
        
        # 控制状态
        self.is_running = False
        self.left_active = False
        self.right_active = False
        
        # pygame初始化
        pygame.init()
        self.screen = pygame.display.set_mode((900, 700))
        pygame.display.set_caption("VR机械臂控制系统 - 修正版")
        try:
            font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
            self.font = pygame.font.Font(font_path, 16)
        except:
            self.font = pygame.font.Font(None, 16)
        self.clock = pygame.time.Clock()
    
    def set_vr_reference(self):
        """设置VR手柄当前位置为参考位置"""
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        if not dual_goals:
            print("❌ 无法获取VR数据，无法设置参考位置")
            return False
        
        left_goal = dual_goals.get("left")
        right_goal = dual_goals.get("right")
        
        # 设置左手参考位置
        if left_goal and left_goal.target_position is not None:
            self.left_reference_pos = np.array(left_goal.target_position)
            if left_goal.wrist_roll_deg is not None:
                self.left_reference_angles['roll'] = left_goal.wrist_roll_deg
            if left_goal.wrist_flex_deg is not None:
                self.left_reference_angles['pitch'] = left_goal.wrist_flex_deg
            print(f"✅ 左手参考位置设置: {self.left_reference_pos}")
        
        # 设置右手参考位置
        if right_goal and right_goal.target_position is not None:
            self.right_reference_pos = np.array(right_goal.target_position)
            if right_goal.wrist_roll_deg is not None:
                self.right_reference_angles['roll'] = right_goal.wrist_roll_deg
            if right_goal.wrist_flex_deg is not None:
                self.right_reference_angles['pitch'] = right_goal.wrist_flex_deg
            print(f"✅ 右手参考位置设置: {self.right_reference_pos}")
        
        if self.left_reference_pos is not None or self.right_reference_pos is not None:
            self.reference_set = True
            print("🎯 VR参考位置设置完成！现在可以开始相对控制")
            return True
        else:
            print("❌ 无法检测到VR手柄，请确保VR系统正常运行")
            return False
    
    def start(self):
        """启动控制系统"""
        print("启动VR机械臂控制系统...")
        
        # 连接机械臂
        self.left_arm.connect()
        self.right_arm.connect()
        
        # 移动到初始位置
        print("移动机械臂到初始位置...")
        self.left_arm.move_to_home_position()
        self.right_arm.move_to_home_position()
        time.sleep(2)
        
        # 启动VR监控
        vr_thread = threading.Thread(
            target=lambda: asyncio.run(self.vr_monitor.start_monitoring()), 
            daemon=True
        )
        vr_thread.start()
        time.sleep(2)  # 等待VR初始化
        
        self.is_running = True
        print("系统启动完成!")
        print("\n🎮 重要提示：")
        print("   请按 R 键设置VR手柄的当前位置为参考位置")
        print("   设置后才能开始相对位置控制")
        
        # 主控制循环
        self.main_loop()
    
    def stop(self):
        """停止控制系统"""
        self.is_running = False
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        pygame.quit()
        print("系统已停止")
    
    def low_pass_filter(self, new_value, old_value, alpha):
        """低通滤波器"""
        if isinstance(new_value, np.ndarray):
            return alpha * new_value + (1 - alpha) * old_value
        else:
            return alpha * new_value + (1 - alpha) * old_value
    
    def process_vr_data(self):
        """处理VR数据 - 修正版相对位置控制"""
        if not self.reference_set:
            return
        
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        if not dual_goals:
            return
        
        left_goal = dual_goals.get("left")
        right_goal = dual_goals.get("right")
        
        # 处理左手控制器 → 左机械臂
        if left_goal and left_goal.target_position is not None and self.left_reference_pos is not None:
            vr_pos = np.array(left_goal.target_position)
            
            # 计算相对于参考位置的偏移
            vr_offset = vr_pos - self.left_reference_pos
            
            # VR坐标系转换到机械臂坐标系的偏移
            # VR: x右，y上，z前  ->  机械臂: x前，y左，z上
            robot_offset = np.array([
                -vr_offset[2] * self.vr_scale,  # VR的z(前) -> 机械臂的x(前)，取负号调整方向
                -vr_offset[0] * self.vr_scale,  # VR的x(右) -> 机械臂的y(左)，取负号
                vr_offset[1] * self.vr_scale    # VR的y(上) -> 机械臂的z(上)
            ])
            
            # 计算目标位置 = 参考位置 + 偏移
            target_pos = self.left_arm_reference_pos + robot_offset
            
            # 位置滤波
            self.filtered_left_pos = self.low_pass_filter(
                target_pos, self.filtered_left_pos, self.position_filter_alpha
            )
            
            # 姿态角度处理
            if left_goal.wrist_roll_deg is not None:
                roll_offset = left_goal.wrist_roll_deg - self.left_reference_angles['roll']
                self.filtered_left_angles['roll'] = self.low_pass_filter(
                    roll_offset, self.filtered_left_angles['roll'], self.angle_filter_alpha
                )
            
            if left_goal.wrist_flex_deg is not None:
                pitch_offset = left_goal.wrist_flex_deg - self.left_reference_angles['pitch']
                # 将pitch映射到末端执行器角度
                self.filtered_left_angles['end_effector'] = self.low_pass_filter(
                    pitch_offset * 0.5, self.filtered_left_angles['end_effector'], self.angle_filter_alpha
                )
            
            # 计算逆运动学
            joint_positions = self.left_arm.cartesian_to_joint_angles(
                self.filtered_left_pos[0],
                self.filtered_left_pos[1], 
                self.filtered_left_pos[2],
                self.filtered_left_angles['roll'],
                self.filtered_left_angles['end_effector']
            )
            
            if joint_positions:
                self.left_arm.target_positions.update(joint_positions)
                self.left_active = True
            
            # 夹爪控制
            if left_goal.metadata and 'trigger' in left_goal.metadata:
                if left_goal.metadata['trigger'] > 0.5:
                    self.left_arm.target_positions['gripper'] = 2500  # 张开
                else:
                    self.left_arm.target_positions['gripper'] = 1650  # 闭合
        else:
            self.left_active = False
        
        # 处理右手控制器 → 右机械臂
        if right_goal and right_goal.target_position is not None and self.right_reference_pos is not None:
            vr_pos = np.array(right_goal.target_position)
            
            # 计算相对于参考位置的偏移
            vr_offset = vr_pos - self.right_reference_pos
            
            # VR坐标系转换到机械臂坐标系的偏移
            robot_offset = np.array([
                -vr_offset[2] * self.vr_scale,  # VR的z(前) -> 机械臂的x(前)
                -vr_offset[0] * self.vr_scale,  # VR的x(右) -> 机械臂的y(左)
                vr_offset[1] * self.vr_scale    # VR的y(上) -> 机械臂的z(上)
            ])
            
            # 计算目标位置 = 参考位置 + 偏移
            target_pos = self.right_arm_reference_pos + robot_offset
            
            # 位置滤波
            self.filtered_right_pos = self.low_pass_filter(
                target_pos, self.filtered_right_pos, self.position_filter_alpha
            )
            
            # 姿态角度处理
            if right_goal.wrist_roll_deg is not None:
                roll_offset = right_goal.wrist_roll_deg - self.right_reference_angles['roll']
                self.filtered_right_angles['roll'] = self.low_pass_filter(
                    roll_offset, self.filtered_right_angles['roll'], self.angle_filter_alpha
                )
            
            if right_goal.wrist_flex_deg is not None:
                pitch_offset = right_goal.wrist_flex_deg - self.right_reference_angles['pitch']
                self.filtered_right_angles['end_effector'] = self.low_pass_filter(
                    pitch_offset * 0.5, self.filtered_right_angles['end_effector'], self.angle_filter_alpha
                )
            
            # 计算逆运动学
            joint_positions = self.right_arm.cartesian_to_joint_angles(
                self.filtered_right_pos[0],
                self.filtered_right_pos[1],
                self.filtered_right_pos[2], 
                self.filtered_right_angles['roll'],
                self.filtered_right_angles['end_effector']
            )
            
            if joint_positions:
                self.right_arm.target_positions.update(joint_positions)
                self.right_active = True
            
            # 夹爪控制
            if right_goal.metadata and 'trigger' in right_goal.metadata:
                if right_goal.metadata['trigger'] > 0.5:
                    self.right_arm.target_positions['gripper'] = 2500  # 张开
                else:
                    self.right_arm.target_positions['gripper'] = 1650  # 闭合
        else:
            self.right_active = False
    
    def update_robots(self):
        """更新机械臂位置"""
        # 读取当前位置
        self.left_arm.read_positions()
        self.right_arm.read_positions()
        
        # 发送目标位置
        if self.left_active:
            success = self.left_arm.write_positions()
            if not success:
                print("左臂控制失败")
        
        if self.right_active:
            success = self.right_arm.write_positions()
            if not success:
                print("右臂控制失败")
    
    def draw_ui(self):
        """绘制用户界面"""
        self.screen.fill((0, 0, 0))
        
        y_offset = 10
        
        # 标题
        title = self.font.render("VR机械臂控制系统 - 修正版", True, (255, 255, 255))
        self.screen.blit(title, (10, y_offset))
        y_offset += 40
        
        # 机械臂参数显示
        param_title = self.font.render("机械臂参数:", True, (255, 255, 0))
        self.screen.blit(param_title, (10, y_offset))
        y_offset += 25
        
        param_text = self.font.render(
            f"大臂长度: {self.left_arm.L1:.3f}m, 小臂长度: {self.left_arm.L2:.3f}m, 底座高度: {self.left_arm.base_height:.3f}m", 
            True, (200, 200, 200)
        )
        self.screen.blit(param_text, (20, y_offset))
        y_offset += 25
        
        max_reach = self.left_arm.L1 + self.left_arm.L2
        min_reach = abs(self.left_arm.L1 - self.left_arm.L2)
        reach_text = self.font.render(
            f"工作空间: 最大伸展 {max_reach:.3f}m, 最小收缩 {min_reach:.3f}m", 
            True, (200, 200, 200)
        )
        self.screen.blit(reach_text, (20, y_offset))
        y_offset += 35
        
        # VR连接状态
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        left_connected = dual_goals and dual_goals.get("has_left", False)
        right_connected = dual_goals and dual_goals.get("has_right", False)
        
        vr_status = self.font.render(f"VR状态: 左手{'✓' if left_connected else '✗'} 右手{'✓' if right_connected else '✗'}", True, (0, 255, 0) if (left_connected or right_connected) else (255, 0, 0))
        self.screen.blit(vr_status, (10, y_offset))
        y_offset += 30
        
        # 参考位置设置状态显示
        ref_status_color = (0, 255, 0) if self.reference_set else (255, 165, 0)
        ref_status_text = "参考位置: ✓ 已设置" if self.reference_set else "参考位置: ⚠ 未设置 (按R键设置)"
        ref_status = self.font.render(ref_status_text, True, ref_status_color)
        self.screen.blit(ref_status, (10, y_offset))
        y_offset += 35
        
        # 机械臂状态
        left_status = self.font.render(f"左臂: {'激活' if self.left_active else '待机'}", True, (0, 255, 0) if self.left_active else (128, 128, 128))
        self.screen.blit(left_status, (10, y_offset))
        y_offset += 25
        
        right_status = self.font.render(f"右臂: {'激活' if self.right_active else '待机'}", True, (0, 255, 0) if self.right_active else (128, 128, 128))
        self.screen.blit(right_status, (10, y_offset))
        y_offset += 35
        
        # 目标位置显示
        left_pos_text = self.font.render(f"左臂目标: [{self.filtered_left_pos[0]:.3f}, {self.filtered_left_pos[1]:.3f}, {self.filtered_left_pos[2]:.3f}]", True, (255, 255, 255))
        self.screen.blit(left_pos_text, (10, y_offset))
        y_offset += 25
        
        right_pos_text = self.font.render(f"右臂目标: [{self.filtered_right_pos[0]:.3f}, {self.filtered_right_pos[1]:.3f}, {self.filtered_right_pos[2]:.3f}]", True, (255, 255, 255))
        self.screen.blit(right_pos_text, (10, y_offset))
        y_offset += 35
        
        # 当前末端执行器位置显示 (正运动学)
        try:
            left_current_pos = self.left_arm.get_end_effector_position()
            right_current_pos = self.right_arm.get_end_effector_position()
            
            left_current_text = self.font.render(f"左臂当前: [{left_current_pos[0]:.3f}, {left_current_pos[1]:.3f}, {left_current_pos[2]:.3f}]", True, (100, 255, 100))
            self.screen.blit(left_current_text, (10, y_offset))
            y_offset += 25
            
            right_current_text = self.font.render(f"右臂当前: [{right_current_pos[0]:.3f}, {right_current_pos[1]:.3f}, {right_current_pos[2]:.3f}]", True, (100, 255, 100))
            self.screen.blit(right_current_text, (10, y_offset))
            y_offset += 35
        except:
            pass
        
        # 关节位置显示 (显示舵机位置值和对应角度)
        if self.left_arm.current_positions:
            y_offset += 10
            left_title = self.font.render("左臂关节状态:", True, (200, 200, 255))
            self.screen.blit(left_title, (10, y_offset))
            y_offset += 20
            
            for joint, pos in self.left_arm.current_positions.items():
                if joint != 'gripper':
                    angle = self.left_arm.servo_to_angle(pos)
                    target_pos = self.left_arm.target_positions.get(joint, pos)
                    target_angle = self.left_arm.servo_to_angle(target_pos)
                    joint_text = f"{joint}: {pos:4d} ({angle:+6.1f}°) -> {target_pos:4d} ({target_angle:+6.1f}°)"
                else:
                    target_pos = self.left_arm.target_positions.get(joint, pos)
                    joint_text = f"{joint}: {pos:4d} -> {target_pos:4d}"
                
                joint_display = self.font.render(joint_text, True, (180, 180, 180))
                self.screen.blit(joint_display, (10, y_offset))
                y_offset += 18
        
        if self.right_arm.current_positions:
            y_offset += 10
            right_title = self.font.render("右臂关节状态:", True, (255, 200, 200))
            self.screen.blit(right_title, (10, y_offset))
            y_offset += 20
            
            for joint, pos in self.right_arm.current_positions.items():
                if joint != 'gripper':
                    angle = self.right_arm.servo_to_angle(pos)
                    target_pos = self.right_arm.target_positions.get(joint, pos)
                    target_angle = self.right_arm.servo_to_angle(target_pos)
                    joint_text = f"{joint}: {pos:4d} ({angle:+6.1f}°) -> {target_pos:4d} ({target_angle:+6.1f}°)"
                else:
                    target_pos = self.right_arm.target_positions.get(joint, pos)
                    joint_text = f"{joint}: {pos:4d} -> {target_pos:4d}"
                
                joint_display = self.font.render(joint_text, True, (180, 180, 180))
                self.screen.blit(joint_display, (10, y_offset))
                y_offset += 18
        
        # 操作提示
        y_offset += 10
        instructions_title = self.font.render("操作说明:", True, (255, 255, 0))
        self.screen.blit(instructions_title, (10, y_offset))
        y_offset += 25
        
        instructions = [
            "• R键 - 设置VR手柄当前位置为参考位置 ★",
            "• 移动VR控制器进行相对位置控制",
            "• 扳机键控制夹爪开合", 
            "• 手柄俯仰角度控制末端执行器角度",
            "• 手柄旋转角度控制腕部旋转",
            "• H键 - 回到初始位置",
            "• T键 - 测试逆运动学算法",
            "• ESC键 - 退出程序",
            "",
            "机械臂结构说明:",
            "• 关节2,3,4只能在ZY平面内运动",
            "• 关节1负责水平方向旋转",
            "• 关节5负责腕部旋转，关节6为夹爪",
            "",
            "舵机位置说明:",
            "• 2048 = 0度 (中位)",
            "• 3048 ≈ +90度, 1048 ≈ -90度"
        ]
        
        for instruction in instructions:
            color = (255, 255, 0) if "R键" in instruction else (180, 180, 180)
            text = self.font.render(instruction, True, color)
            self.screen.blit(text, (10, y_offset))
            y_offset += 20
        
        pygame.display.flip()
    
    def test_inverse_kinematics(self):
        """测试逆运动学算法"""
        print("\n🧮 测试逆运动学算法...")
        print("=" * 50)
        
        test_positions = [
            (0.15, 0.0, 0.25),   # 正前方
            (0.2, 0.1, 0.3),     # 右前方
            (0.1, -0.1, 0.2),    # 左前方
            (0.25, 0.0, 0.15),   # 正前方低位
            (0.05, 0.0, 0.35),   # 正前方高位
        ]
        
        for i, (x, y, z) in enumerate(test_positions):
            print(f"\n测试位置 {i+1}: ({x:.3f}, {y:.3f}, {z:.3f})")
            print("-" * 30)
            
            # 测试左臂逆运动学
            joint_angles = self.left_arm.cartesian_to_joint_angles(x, y, z, 0, 0)
            if joint_angles:
                print("✅ 逆运动学求解成功")
                
                # 验证正运动学
                # 临时设置关节位置用于验证
                temp_positions = self.left_arm.target_positions.copy()
                self.left_arm.target_positions.update(joint_angles)
                
                # 计算正运动学
                fk_x, fk_y, fk_z = self.left_arm.get_end_effector_position()
                
                # 恢复原始位置
                self.left_arm.target_positions = temp_positions
                
                # 计算误差
                error_x = abs(x - fk_x)
                error_y = abs(y - fk_y) 
                error_z = abs(z - fk_z)
                total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                
                print(f"📐 正运动学验证:")
                print(f"   目标位置: ({x:.3f}, {y:.3f}, {z:.3f})")
                print(f"   计算位置: ({fk_x:.3f}, {fk_y:.3f}, {fk_z:.3f})")
                print(f"   位置误差: ({error_x:.4f}, {error_y:.4f}, {error_z:.4f})")
                print(f"   总误差: {total_error:.4f}m")
                
                if total_error < 0.01:  # 1cm误差容限
                    print("✅ 验证通过 (误差 < 1cm)")
                else:
                    print("⚠️  误差较大 (误差 > 1cm)")
            else:
                print("❌ 逆运动学求解失败")
        
        print("\n" + "=" * 50)
        print("测试完成！")
    
    def main_loop(self):
        """主控制循环"""
        while self.is_running:
            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    self.is_running = False
                    break
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_h:
                    # H键回到初始位置
                    print("回到初始位置...")
                    self.left_arm.move_to_home_position()
                    self.right_arm.move_to_home_position()
                    self.left_active = False
                    self.right_active = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                    # R键设置参考位置
                    print("🎯 正在设置VR参考位置...")
                    success = self.set_vr_reference()
                    if success:
                        print("✅ 参考位置设置成功！现在可以开始相对控制")
                    else:
                        print("❌ 参考位置设置失败，请检查VR连接")
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_t:
                    # T键测试逆运动学
                    self.test_inverse_kinematics()
            
            try:
                # 处理VR数据
                self.process_vr_data()
                
                # 更新机械臂
                self.update_robots()
                
                # 绘制界面
                self.draw_ui()
                
                # 控制循环频率
                self.clock.tick(50)  # 50Hz控制频率
                
            except Exception as e:
                print(f"控制循环错误: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        self.stop()

def main():
    """主函数"""
    print("🤖 VR机械臂控制系统 - 修正版")
    print("=" * 60)
    print("机械臂结构:")
    print("  • 底盘固定，关节1为底盘旋转 (水平360°)")
    print("  • 关节2-4在ZY平面内运动 (肩部-肘部-腕部)")
    print("  • 关节5为腕部旋转，关节6为夹爪")
    print("  • 坐标系: X前, Y左, Z上")
    print("=" * 60)
    print(f"当前机械臂参数:")
    print(f"  底座高度: {ARM_PARAMETERS['base_height']:.3f}m")
    print(f"  大臂长度: {ARM_PARAMETERS['shoulder_length']:.3f}m")
    print(f"  小臂长度: {ARM_PARAMETERS['elbow_length']:.3f}m")
    print("=" * 60)
    print("🔧 如需修改参数，请编辑代码顶部的 ARM_PARAMETERS")
    print("=" * 60)
    
    controller = VRRobotController()
    try:
        controller.start()
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.stop()

if __name__ == "__main__":
    main()