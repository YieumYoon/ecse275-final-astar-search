# -*- coding: utf-8 -*-
"""
Function Library for Robot Navigation System.
"""

import numpy as np
from enum import Enum
import math
import heapq
from typing import Tuple, List, Optional, Union, Dict, Set
import config

def process_vision_sensor_rgb(sim, sensor_name: str) -> Tuple[np.ndarray, Tuple[int, int]]:
    """
    Captures the RGB image from the vision sensor.

    Args:
        sim: The CoppeliaSim remote API object.
        sensor_name (str): The name of the vision sensor object.

    Returns:
        Tuple[np.ndarray, Tuple[int, int]]:
            - RGB map: (H, W, 3) numpy array.
            - Resolution: (W, H) tuple.
    """
    sensor_handle = sim.getObject(sensor_name)
    img_raw, res = sim.getVisionSensorImg(sensor_handle)
    img = np.frombuffer(img_raw, dtype=np.uint8).reshape(res[1], res[0], 3)
    return img, res

def process_vision_sensor_depth(sim, sensor_name: str) -> Tuple[np.ndarray, Tuple[int, int]]:
    """
    Captures the depth map from the vision sensor.

    Args:
        sim: The CoppeliaSim remote API object.
        sensor_name (str): The name of the vision sensor object.

    Returns:
        Tuple[np.ndarray, Tuple[int, int]]:
            - Depth map: (H, W) numpy array with linear depth values.
            - Resolution: (W, H) tuple.
    """
    sensor_handle = sim.getObject(sensor_name)
    depth_raw, res = sim.getVisionSensorDepth(sensor_handle)
    depth_values = sim.unpackFloatTable(depth_raw)
    
    width, height = res[0], res[1]
    depth_buffer = np.array(depth_values).reshape(height, width)

    near = sim.getObjectFloatParam(sensor_handle, sim.visionfloatparam_near_clipping)
    far = sim.getObjectFloatParam(sensor_handle, sim.visionfloatparam_far_clipping)

    linear_depth = near + depth_buffer * (far - near)
    return linear_depth, (width, height)

def compute_pos_from_pix(pixel_uv: Tuple[int, int], resolution: Tuple[int, int], f: float, z: float) -> Tuple[float, float, float]:
    """
    Computes the 3D position in the camera frame from pixel coordinates and depth.
    """
    width, height = resolution
    u, v = pixel_uv
    cx = width / 2
    cy = height / 2

    # f is expected to be FOV in degrees based on original code usage
    angle_rad = math.radians(f)
    view_width = 2 * z * math.tan(angle_rad / 2)
    view_height = view_width * (height / width)

    pw = view_width / width
    ph = view_height / height

    x = (u - cx) * pw
    y = -(v - cy) * ph
    
    return (x, y, z)

def mask_color(img: np.ndarray, target: str) -> np.ndarray:
    """
    Creates a binary mask where the target color dominates.
    target in {"red", "green", "blue"}
    """
    r = img[:, :, 0].astype(np.int32)
    g = img[:, :, 1].astype(np.int32)
    b = img[:, :, 2].astype(np.int32)

    if target == "red":
        mask = (r > g + 20) & (r > b + 20)
    elif target == "green":
        mask = (g > r + 20) & (g > b + 20)
    elif target == "blue":
        mask = (b > r + 20) & (b > g + 20)
    else:
        raise ValueError(f"Invalid target color: {target}")
    return mask

def centroid_from_mask(mask: np.ndarray) -> Optional[np.ndarray]:
    """
    Returns the centroid of a binary mask.
    """
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    return np.array([np.mean(xs), np.mean(ys)])

def process_lidar_depth(sim, sensor_name: str, threshold: float) -> List[np.ndarray]:
    """
    Processes LiDAR data to find segments.
    """
    script_handle = sim.getScript(sim.scripttype_childscript, sensor_name)
    raw_data = sim.callScriptFunction('getMeasuredData', script_handle)
    shaped_data = np.array(raw_data).reshape(-1, 3)
    return segment_lidar(shaped_data, threshold)

def transform_point(mat: List[float], point: Union[np.ndarray, List[float], Tuple[float, ...]]) -> np.ndarray:
    """
    Transforms a point using a flattened 3x4 matrix.
    """
    x = mat[0]*point[0] + mat[1]*point[1] + mat[2]*point[2] + mat[3]
    y = mat[4]*point[0] + mat[5]*point[1] + mat[6]*point[2] + mat[7]
    z = mat[8]*point[0] + mat[9]*point[1] + mat[10]*point[2] + mat[11]
    return np.array([x, y, z])

def segment_lidar(points: np.ndarray, threshold: float) -> List[np.ndarray]:
    """
    Segments LiDAR points based on distance jumps.
    """
    if len(points) == 0:
        return []
        
    diffs = np.diff(points[:, :2], axis=0)
    dist = np.linalg.norm(diffs, axis=1)
    break_idx = np.where(dist > threshold)[0]

    segments = []
    start = 0
    for idx in break_idx:
        segments.append(points[start:idx+1])
        start = idx + 1
    
    if start < len(points):
        segments.append(points[start:])
    return segments

class TerrainType(Enum):
    FLOOR = 0
    GRASS = 1
    SAND = 2
    WATER = 3
    OBSTACLE = 4

class Terrain:
    def __init__(self, width: float = 0, coordinate: Optional[List[float]] = None, terrain_type: TerrainType = TerrainType.FLOOR, resolution: int = 5):
        self.width = width
        self.obstacle_coords = coordinate
        self.terrain_type = terrain_type
        self.resolution = resolution
        
        # Calculate map indices (i, j) from world coordinates if available
        if coordinate:
            # Assuming coordinate is [x, y] or [x, y, z]
            R = config.WORLD_SIZE / resolution
            map_coords = convert_world_to_map(coordinate[0], coordinate[1], R, resolution)
            self.i = map_coords[0]
            self.j = map_coords[1]
        else:
            self.i = 0
            self.j = 0

    def get_width(self) -> float:
        return self.width
    
    def set_width(self, width: float):
        self.width = width
    
    def get_coordinate_array(self) -> Optional[List[float]]:
        return self.obstacle_coords
    
    def is_floor(self) -> bool:
        return self.terrain_type == TerrainType.FLOOR
    
    def is_obstacle(self) -> bool:
        return self.terrain_type == TerrainType.OBSTACLE
    
    def get_terrain_cost(self) -> float:
        if self.terrain_type == TerrainType.FLOOR:
            return 0
        elif self.terrain_type == TerrainType.GRASS:
            return 2
        elif self.terrain_type == TerrainType.SAND:
            return 4
        elif self.terrain_type == TerrainType.WATER:
            return 8
        elif self.terrain_type == TerrainType.OBSTACLE:
            return math.inf
        return 0

    def get_terrain_num(self) -> int:
        return self.terrain_type.value
    
    def set_terrain_type(self, terrain_type: TerrainType):
        self.terrain_type = terrain_type
    
    def get_world_coords(self) -> Tuple[float, float]:
        R = config.WORLD_SIZE / self.resolution
        return tuple(convert_map_to_world(self.i, self.j, R, self.resolution))

def convert_world_to_map(xw: float, yw: float, r: float, resolution: int) -> List[int]:
    half = resolution / 2
    i = xw / r + half
    j = yw / r + half
    return [int(i), int(j)]

def convert_map_to_world(i: int, j: int, r: float, resolution: int) -> List[float]:
    half = resolution / 2
    xw = (i - half) * r
    yw = (j - half) * r
    return [xw, yw]

def get_cells_to_fill(terrain_obj: Terrain, resolution: int, i: int, j: int, grid: np.ndarray):
    r = config.WORLD_SIZE / resolution
    width = terrain_obj.get_width()
    if width <= r:
        return
    
    num_squares_to_fill = int(width / r)
    leftmost_i = (i - num_squares_to_fill // 2)
    topmost_j = (j - num_squares_to_fill // 2)
    
    rightmost = leftmost_i + num_squares_to_fill
    bottom_most = topmost_j + num_squares_to_fill
    
    for w in range(leftmost_i, rightmost):
        for l in range(topmost_j, bottom_most):
            if 0 <= w < resolution and 0 <= l < resolution:
                grid[w, l] = terrain_obj

def update_map(grid: np.ndarray, terrain_array: List[Terrain], resolution: int):
    for curr_terrain in terrain_array:
        terrain_world_coord = curr_terrain.get_coordinate_array()
        if terrain_world_coord:
            terrain_map_coord = convert_world_to_map(terrain_world_coord[0], terrain_world_coord[1], config.WORLD_SIZE/resolution, resolution)
            i, j = terrain_map_coord
            if 0 <= i < resolution and 0 <= j < resolution:
                grid[i][j] = curr_terrain
                get_cells_to_fill(curr_terrain, resolution, i, j, grid)

def create_map_with_resolution(resolution: int) -> np.ndarray:
    grid = np.zeros((resolution, resolution), dtype=object)
    r = config.WORLD_SIZE / resolution
    for i in range(resolution):
        for j in range(resolution):
            grid[i][j] = Terrain(
                width=0,
                coordinate=convert_map_to_world(i, j, r, resolution),
                terrain_type=TerrainType.FLOOR,
                resolution=resolution
            )
            # Manually set i and j since they are known here
            grid[i][j].i = i
            grid[i][j].j = j
    return grid

def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(pos: Tuple[int, int], grid_size: int) -> List[Tuple[int, int]]:
    i, j = pos
    neighbors = []
    if i > 0: neighbors.append((i - 1, j))
    if i < grid_size - 1: neighbors.append((i + 1, j))
    if j > 0: neighbors.append((i, j - 1))
    if j < grid_size - 1: neighbors.append((i, j + 1))
    return neighbors

def astar(grid: np.ndarray, start: List[float], goal: List[float], r: float, resolution: int) -> Optional[List[Tuple[int, int]]]:
    start_map = tuple(convert_world_to_map(start[0], start[1], r, resolution))
    goal_map = tuple(convert_world_to_map(goal[0], goal[1], r, resolution))
    
    # Bounds check
    if not (0 <= start_map[0] < resolution and 0 <= start_map[1] < resolution):
        print(f"Start position {start_map} is out of bounds")
        return None
    if not (0 <= goal_map[0] < resolution and 0 <= goal_map[1] < resolution):
        print(f"Goal position {goal_map} is out of bounds")
        return None

    if grid[start_map[0]][start_map[1]].is_obstacle():
        print(f"Start position {start_map} is an obstacle!")
        return None
    
    if grid[goal_map[0]][goal_map[1]].is_obstacle():
        print(f"Goal position {goal_map} is an obstacle!")
        return None
    
    open_set = []
    heapq.heappush(open_set, (0, start_map))
    
    came_from = {}
    g_score = {start_map: 0}
    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
        closed_set.add(current)
        
        if current == goal_map:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_map)
            return path[::-1]
        
        for neighbor in get_neighbors(current, resolution):
            if neighbor in closed_set:
                continue
                
            neighbor_cell = grid[neighbor[0]][neighbor[1]]
            if neighbor_cell.is_obstacle():
                continue
            
            terrain_cost = neighbor_cell.get_terrain_cost()
            tentative_g = g_score[current] + 1 + terrain_cost
            
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal_map)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current

    print(f"A* exhausted. Explored {len(closed_set)} cells")
    return None

def a_star_path_to_coppelia_points(path: List[Tuple[int, int]], grid: np.ndarray, z_height: float = 0.0) -> List[float]:
    pts = []
    for (i, j) in path:
        terrain_obj = grid[i][j]
        x, y = terrain_obj.get_world_coords()
        pts.extend([x, y, z_height])
    return pts
