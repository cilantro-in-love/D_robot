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


config1 = FeetechMotorsBusConfig(
    port="/dev/ttyACM0",  # 左机械臂串口
    motors={
        "base_rotation": [1, "sts3215"],     # 底盘旋转
        "shoulder": [2, "sts3215"],          # 肩部关节
        "elbow": [3, "sts3215"],            # 肘部关节
        "wrist_pitch": [4, "sts3215"],      # 腕部俯仰
        "wrist_roll": [5, "sts3215"],       # 腕部旋转
        "gripper": [6, "sts3215"],          # 夹爪
    }
)

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

    def date_key(self):
        # 初始化Pygame
        pygame.init()
        screen = pygame.display.set_mode((500, 350))
        pygame.display.set_caption("XYZ值控制")
        font = pygame.font.SysFont(None, 36)
        small_font = pygame.font.SysFont(None, 28)
        

        
        # 步长
        step_size = 0.001
        
        print("控制说明:")
        print("W/S: 增加/减少 X值")
        print("A/D: 增加/减少 Y值")
        print("↑/↓: 增加/减少 Z值")
        print("R: 重置所有值为0")
        print("Q或ESC: 退出")
        
        running = True
        clock = pygame.time.Clock()
        
        while running:
            # 处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        running = False
                    elif event.key == pygame.K_r:
                        self.x = 0.0
                        self.y = 0.0
                        self.z = 0.0
                        print("已重置所有值为0")
            
            # 获取按键状态
            keys = pygame.key.get_pressed()
            
            # 控制X值
            if keys[pygame.K_w]:
                self.y += step_size
            if keys[pygame.K_s]:
                self.y -= step_size
            
            # 控制Y值
            if keys[pygame.K_d]:
                self.x += step_size
            if keys[pygame.K_a]:
                self.x -= step_size
            
            # 控制Z值
            if keys[pygame.K_UP]:
                self.z += step_size
            if keys[pygame.K_DOWN]:
                self.z -= step_size
            
            # 绘制界面
            screen.fill((30, 30, 40))
            font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
            font = pygame.font.Font(font_path, 16)
            small_font = pygame.font.Font(font_path, 12)
            
            # 标题
            title = font.render("XYZ值控制", True, (255, 255, 255))
            screen.blit(title, (150, 30))
            
            # 显示X值
            x_text = font.render(f"X: {self.x:.3f}", True, (255, 100, 100))
            screen.blit(x_text, (50, 100))
            x_control = small_font.render("(W/S)", True, (200, 200, 200))
            screen.blit(x_control, (300, 105))
            
            # 显示Y值
            y_text = font.render(f"Y: {self.y:.3f}", True, (100, 255, 100))
            screen.blit(y_text, (50, 160))
            y_control = small_font.render("(A/D)", True, (200, 200, 200))
            screen.blit(y_control, (300, 165))
            
            # 显示Z值
            z_text = font.render(f"Z: {self.z:.3f}", True, (100, 150, 255))
            screen.blit(z_text, (50, 220))
            z_control = small_font.render("(↑/↓)", True, (200, 200, 200))
            screen.blit(z_control, (300, 225))
            
            # 底部提示
            hint = small_font.render("R:重置  Q/ESC:退出", True, (150, 150, 150))
            screen.blit(hint, (120, 290))
            
            pygame.display.flip()
            clock.tick(60)  # 60 FPS
            
            # 输出到控制台（可选）
            # print(f"X: {self.x:.3f}, Y: {self.y:.3f}, Z: {self.z:.3f}")
        
        # 清理
        pygame.quit()
        print(f"\n最终值: X={self.x:.3f}, Y={self.y:.3f}, Z={self.z:.3f}")


    def process_robot_data(self, robot_pos):
        """处理机器人数据"""
        l4 = 0.15
        # TODO: 根据机器人位置更新VR手柄位置
        robot_angel = np.zeros(5)
        robot_position = [0,0.29,0.116,0,0]
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


    def main(self):
        motors_bus1 = FeetechMotorsBus(config1)
        motors_bus1.connect()  # 打开串口，建立通信
        

        vr_thread = threading.Thread(
            target=lambda: asyncio.run(self.date_key()), 
            daemon=True
        )
        vr_thread.start()
        print("VR手柄初始化完成")
        time.sleep(3)
        while True:
            robot_pos = [self.x, self.y, self.z, 0,0]
            robot_pos = self.limit_range_pos(robot_pos)
            robot_angel = self.process_robot_data(robot_pos)
            robot_angel_base = int(robot_angel[0]*180 / math.pi*self.k_motor)
            robot_angel_flex = int(robot_angel[3]*180 / math.pi*self.k_motor)
            robot_angel_roll = int(robot_angel[4]*self.k_motor)
            robot_angel_shouder = int(((robot_angel[1]*180 / math.pi)-90)*self.k_motor)
            robot_angel_elbow = int(((robot_angel[2]*180 / math.pi)-90)*self.k_motor)
            motors_bus1.write("Goal_Position", [2048+robot_angel_base,2048-robot_angel_shouder,2048+robot_angel_elbow,2048-robot_angel_flex,2048+robot_angel_roll,2048])
            print("VR data processed")
            time.sleep(0.01)
 

if __name__ == "__main__":
    vr_date_interface = VrDateInterface()
    vr_date_interface.main()