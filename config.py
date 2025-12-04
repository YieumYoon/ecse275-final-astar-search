# -*- coding: utf-8 -*-
"""
Configuration module for Robot Navigation System

This module centralizes all configurable parameters for the multi-robot
navigation system. Modify values here instead of hardcoding in main files.

Author: Junsu Lee
Date: Dec 3, 2025
"""

from dataclasses import dataclass, field
from typing import List
import logging


@dataclass
class LoggingConfig:
    """Configuration for logging output."""
    level: str = "INFO"            # Logging level: DEBUG, INFO, WARNING, ERROR, CRITICAL
    format: str = '[%(asctime)s] [%(threadName)s] %(levelname)s: %(message)s'
    date_format: str = '%H:%M:%S'

    def get_level(self) -> int:
        """Convert string level to logging constant."""
        return getattr(logging, self.level.upper(), logging.INFO)


@dataclass
class MapConfig:
    """Configuration for the occupancy grid map."""
    resolution: int = 100           # Grid resolution (NxN cells)
    world_size: float = 10.0        # World size in meters

    @property
    def cell_size(self) -> float:
        """Calculate the cell size (R) in meters."""
        return self.world_size / self.resolution


@dataclass
class SensorConfig:
    """Configuration for robot sensors."""
    lidar_threshold: float = 0.2    # LiDAR segmentation threshold
    vision_fov_deg: float = 60.0    # Vision sensor field of view (degrees)
    vision_resolution: tuple = (256, 256)  # Vision sensor resolution


@dataclass
class NavigationConfig:
    """Configuration for navigation and pathfinding."""
    scan_interval: float = 1.0      # Seconds between sensor scans
    goal_reached_threshold: float = 0.5  # Distance to consider goal reached
    use_8_connected: bool = False   # Use 8-connected (diagonal) movement


@dataclass
class TerrainCosts:
    """Cost values for different terrain types."""
    floor: float = 0.0
    grass: float = 2.0
    sand: float = 4.0
    water: float = 8.0
    # obstacle: float.inf (handled separately)


@dataclass
class RobotConfig:
    """Configuration for robots in the simulation."""
    names: List[str] = field(default_factory=lambda: ["/Robot_0", "/Robot_1"])
    lidar_sensor_suffix: str = "/fastHokuyo_0"
    vision_sensor_suffix: str = "/visionSensor"


@dataclass
class GoalConfig:
    """Configuration for goal points."""
    names: List[str] = field(default_factory=lambda: [
        "/goal_point",
        "/goal_point_1",
        "/goal_point_2",
        "/goal_point_3",
        "/goal_point_4"
    ])


@dataclass
class ProjectConfig:
    """Master configuration combining all sub-configurations."""
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    map: MapConfig = field(default_factory=MapConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    terrain_costs: TerrainCosts = field(default_factory=TerrainCosts)
    robots: RobotConfig = field(default_factory=RobotConfig)
    goals: GoalConfig = field(default_factory=GoalConfig)


# Default configuration instance
default_config = ProjectConfig()


# Usage example:
# from config import default_config as cfg
# resolution = cfg.map.resolution
# robot_names = cfg.robots.names
