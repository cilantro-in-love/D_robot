
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
motors_bus1 = FeetechMotorsBus(config1)
motors_bus1.connect()  # 打开串口，建立通信
motors_bus1.write("Goal_Position",[2024,2024,2024,2024,2024,2024])
input("请按回车键继续")
motors_bus1.write("Goal_Position",[2024,2024,2500,2024,2024,2024])

