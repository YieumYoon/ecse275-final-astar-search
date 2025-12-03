# -*- coding: utf-8 -*-
"""
Configuration parameters for the Robot Navigation System.
"""

import math

# Map Configuration
RESOLUTION = 100
WORLD_SIZE = 10.0  # meters
CELL_SIZE = WORLD_SIZE / RESOLUTION

# LiDAR Configuration
LIDAR_THRESHOLD = 0.2  # meters

# Vision Configuration
VISION_SENSOR_NAME = 'visionSensor'
VISION_WIDTH = 256
VISION_HEIGHT = 256
VISION_FOV_DEG = 60
VISION_FOV_RAD = math.radians(VISION_FOV_DEG)

# Connection Configuration
ZMQ_PORT = 23000  # Default port, though usually auto-discovered
ROBOT_NAME = '/Robot_0'
LIDAR_NAME = '/Robot_0/fastHokuyo_0'
GOAL_NAME = '/goal_point'
