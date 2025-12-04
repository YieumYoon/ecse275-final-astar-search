# -*- coding: utf-8 -*-
"""
Created on Sun Nov  9 18:26:56 2025

@author: halas
"""

from __future__ import annotations
from typing import List, Tuple, Optional, Any
import numpy as np
from numpy.typing import NDArray
from enum import Enum
import array
import math
import heapq
import logging

# Module logger - configuration is done in main module
logger = logging.getLogger(__name__)


def process_vision_Sensor_RBG(sim: Any, sensor_name: str) -> Tuple[NDArray[np.uint8], Tuple[int, int]]:
    '''
    This funcion takes a sim object and the Sensor Name as a string.,
    Then it returns a 256,256 array of RGB

    Args:
        sim: The CoppeliaSim remote API object (e.g., from client.getObject('sim')).
        sensor_name (str): The name of the vision sensor object in the scene.
    Returns:
        Tuple: RBG_map , Tuple: Resolution 
            - RGB map : 2D numpy array of (H x W) each containg a tuple of [R,G,B]
            -Resolution: tuple of (H,W)
    '''

    # Assumes Connection and simulation are running aldready

    sensorHandle = sim.getObject(f'/{sensor_name}')

    # captures the RAW image form the camera sensor
    imgL_raw, resL = sim.getVisionSensorImg(sensorHandle)

    # creates a Np array from raw buffer data and then reshapes this into a H x W , of 3 elements array
    imgL = np.frombuffer(imgL_raw, dtype=np.uint8).reshape(resL[1], resL[0], 3)

    return imgL, resL


def process_vision_sensor_depth(sim: Any, sensor_name: str) -> Tuple[NDArray[np.float64], Tuple[int, int]]:
    """
    This function takes a sim object and the sensor name as a string,
    then returns a depth map of the vision sensor.

    Args:
        sim: The CoppeliaSim remote API object
        sensor_name (str): Name of the vision sensor in the scene

    Returns:
        Tuple: depth_map, resolution
            - depth_map: 2D numpy array of shape (H, W) containing depth values (meters)
            - resolution: tuple (W, H)
    """
    # Get the sensor handle
    sensorHandle = sim.getObject(f'/{sensor_name}')

    # Capture the depth buffer from the camera
    depth_raw, res = sim.getVisionSensorDepth(sensorHandle)

    # Convert to float array
    depth_values = sim.unpackFloatTable(depth_raw)

    width, height = res[0], res[1]
    depth_buffer = np.array(depth_values).reshape(height, width)

    # get near & far planes
    near = sim.getObjectFloatParam(
        sensorHandle, sim.visionfloatparam_near_clipping)
    far = sim.getObjectFloatParam(
        sensorHandle, sim.visionfloatparam_far_clipping)

    # convert to linear depth
    linear_depth = near + depth_buffer * (far - near)

    return linear_depth, (width, height)


def compute_pos_from_pix(pixel_uv: Tuple[int, int], resolution: Tuple[int, int], f: float, z: float) -> Tuple[float, float, float]:

    width, height = resolution
    u, v = pixel_uv
    cx = width / 2
    cy = height / 2

    # Convert angle to radians
    angle_rad = math.radians(f)

    # Calculate the physical width and height of the view at distance z
    view_width = 2 * z * math.tan(angle_rad / 2)
    view_height = view_width * (height / width)

    # Pixel width and height in meters
    pw = view_width / width   # pixel width in meters
    ph = view_height / height  # pixel height in meters

    x = (u - cx) * pw
    y = -(v - cy) * ph

    point_cam = (x, y, z)

    return point_cam


def mask_color(img: NDArray[np.uint8], target: str) -> NDArray[np.bool_]:
    """
    target ∈ {"red", "green", "blue"}
    Creates a binary mask where the target color dominates.
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
        raise ValueError("Invalid target color")
    return mask


def centroid_from_mask(mask: NDArray[np.bool_]) -> Optional[NDArray[np.float64]]:
    """
    Returns the centroid of a binary mask and the indices of all points.

    Args:
        mask: 2D binary numpy array

    Returns:
        tuple: (centroid, indices)
            - centroid: np.array([mean_x, mean_y])
            - indices: tuple of arrays (ys, xs)
    """
    ys, xs = np.where(mask)  # rows → y, cols → x

    if len(xs) == 0:
        return None

    centroid = np.array([np.mean(xs), np.mean(ys)])
    return centroid


def depth_from_rgb_mask(depth_map: NDArray[np.float64], rgb_mask: NDArray[np.bool_]) -> NDArray[np.float32]:
    if depth_map.shape != rgb_mask.shape:
        raise ValueError("Depth map and RGB mask must have the same shape")
    masked_depth = np.zeros_like(depth_map, dtype=np.float32)
    masked_depth[rgb_mask] = depth_map[rgb_mask]
    return masked_depth


def process_Lidar_depth(sim: Any, sensor_name: str, threshold: float) -> List[NDArray[np.float64]]:
    '''
    This function calls the getMeasure data of Hokuyo, and then makes it into a
    1 x 3 array from x , y , z. with each point is a point of contact with an obstacle. 
    '''
    scriptHandle = sim.getScript(sim.scripttype_childscript, sensor_name)
    raw_Data = sim.callScriptFunction('getMeasuredData', scriptHandle)
    shaped_data = np.array(raw_Data).reshape(-1, 3)

    segmented_points = segment_lidar(shaped_data, threshold)

    return segmented_points


def transform_point(mat: List[float], point: NDArray[np.float64]) -> NDArray[np.float64]:
    # mat: 3x4 flattened → same as CoppeliaSim
    x = mat[0]*point[0] + mat[1]*point[1] + mat[2]*point[2] + mat[3]
    y = mat[4]*point[0] + mat[5]*point[1] + mat[6]*point[2] + mat[7]
    z = mat[8]*point[0] + mat[9]*point[1] + mat[10]*point[2] + mat[11]
    return np.array([x, y, z])


def segment_lidar(points: NDArray[np.float64], threshold: float) -> List[NDArray[np.float64]]:
    diffs = np.diff(points[:, :2], axis=0)  # use x,y only
    dist = np.linalg.norm(diffs, axis=1)

    # indices where a jump happens
    break_idx = np.where(dist > threshold)[0]

    segments = []
    start = 0

    for idx in break_idx:
        segments.append(points[start:idx+1])
        start = idx+1

    # last segment
    if start < len(points):
        segments.append(points[start:])

    return segments


'''
Update map function, this is a function that would take the grip_map 
and takes a N length array of terrain objects to update the map.

    grid - gridmap of terrain objects the current world map
    terrain_array - terrain objects array 
    Resolution - Pass in the Resolution of the map
    
    updates the grid map given
'''


def Update_map(grid: NDArray[np.object_], terrain_array: List['terrain'], Resolution: int, world_size: float = 10.0) -> None:
    n = len(grid)
    R = world_size / Resolution
    for idx in range(len(terrain_array)):
        currTerrain = terrain_array[idx]
        Terrain_world_coord = currTerrain.getCoordinateArray()
        Terrain_map_coord = Convert_world_to_map(
            Terrain_world_coord[0], Terrain_world_coord[1], R, Resolution)
        # Place terrain at its position
        i, j = Terrain_map_coord
        grid[i][j] = currTerrain
        get_cells_to_fill(currTerrain, Resolution, i, j, grid, world_size)


'''
Helper function for the Update map function that gets all the cells to fill 
function not eleborated properly since it is a backend use only code. 

'''


def get_cells_to_fill(terrain_obj: 'terrain', Resolution: int, i: int, j: int, grid: NDArray[np.object_], world_size: float = 10.0) -> None:
    R = world_size / Resolution
    width = terrain_obj.getWidth()
    if width <= R:
        return
    else:
        print("else case is triggered")
        num_squares_to_fill = int(width/R)
        leftmost_i = (i - num_squares_to_fill//2)
        topmost_j = (j - num_squares_to_fill//2)

        rightmost = leftmost_i + num_squares_to_fill
        bottom_most = topmost_j + num_squares_to_fill
        for iterating_width in range(leftmost_i, rightmost):
            for iterating_length in range(topmost_j, bottom_most):
                # Bounds check
                if 0 <= iterating_width < Resolution and 0 <= iterating_length < Resolution:
                    grid[iterating_width, iterating_length] = terrain_obj


"""
This is a function to convert from a world coordinate point into a discreet map point
assumes (0,0) is the center of the map

Xw - X position of point detected 
Yw - Y position of the point detected 
R - width of the sqaure world discret space
Resolution - The resolution of the map  

Returns [X position in map,Y position in map]
"""


def Convert_world_to_map(Xw: float, Yw: float, R: float, Resolution: int) -> List[int]:
    half = Resolution / 2
    i = Xw / R + half
    j = Yw / R + half
    return [int(i), int(j)]


'''
This is a function that goes from map coordianted to world coordinates , with 
'''


def Convert_map_to_world(i: int, j: int, R: float, Resolution: int) -> List[float]:
    half = Resolution / 2
    Xw = (i - half) * R
    Yw = (j - half) * R
    return [Xw, Yw]


"""
This Function creates a Numpy array of N by N Zeros where N is the resolution of the map
The value at each coordinate is the cost to come for that coordinate
"""


def createMap_withResolution(Resolution: int, world_size: float = 10.0) -> NDArray[np.object_]:
    Map = np.zeros((Resolution, Resolution), dtype=object)
    R = world_size / Resolution
    for i in range(0, Resolution):
        for j in range(0, Resolution):
            Map[i][j] = terrain(
                width=0,
                Coordinate=Convert_map_to_world(i, j, R, Resolution),
                terrain=TerrainType.FLOOR,
                resolution=Resolution
            )

    return Map


"""
Classes , obstacle , terrain and interupt interface
"""


class TerrainType(Enum):
    FLOOR = 0
    GRASS = 1
    SAND = 2
    WATER = 3
    OBSTACLE = 4


class terrain():
    def __init__(self, width=0, Coordinate=None, terrain: TerrainType = None, resolution=5):
        self.width = width
        self.obstacleCoords = Coordinate
        self.terrain = terrain
        self.resolution = resolution

    def getWidth(self):
        return self.width

    def setWidth(self, width):
        self.width = width

    def getCoordinateArray(self):
        return self.obstacleCoords

    def isFloor(self):
        if self.getTerrainNum() == 0:
            return True
        return False

    def isObstacle(self):
        if self.getTerrainNum() == 4:
            return True
        return False

    def getTerrainCost(self):
        match self.terrain:
            case TerrainType.FLOOR:
                return 0
            case TerrainType.GRASS:
                return 2
            case TerrainType.SAND:
                return 4
            case TerrainType.WATER:
                return 8
            case TerrainType.OBSTACLE:
                return math.inf
        return 0

    def getTerrainNum(self):
        return self.terrain.value

    def setTerrainNum(self, terrain: TerrainType):
        self.terrain = terrain

    def get_world_coords(self) -> Tuple[float, float]:
        """Returns the world coordinates stored in this terrain cell."""
        if self.obstacleCoords is not None:
            return (self.obstacleCoords[0], self.obstacleCoords[1])
        return (0.0, 0.0)


"""
A* Algorithm Implementation (4-connected)
------------------------------------------------
Heuristic: Euclidean distance between current cell and goal.
"""


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def get_neighbors(pos: Tuple[int, int], grid_size: int) -> List[Tuple[int, int]]:
    i, j = pos
    n = grid_size
    neighbors = []
    # 4-connected neighborhood (up, down, left, right)
    if i > 0:
        neighbors.append((i - 1, j))
    if i < n - 1:
        neighbors.append((i + 1, j))
    if j > 0:
        neighbors.append((i, j - 1))
    if j < n - 1:
        neighbors.append((i, j + 1))
    return neighbors


def astar(grid: NDArray[np.object_], start: List[float], goal: List[float], R: float, Resolution: int) -> Optional[List[Tuple[int, int]]]:
    """
    A* pathfinding algorithm.

    Args:
        grid: 2D numpy array of terrain objects
        start: World coordinates [x, y] or [x, y, z]
        goal: World coordinates [x, y] or [x, y, z]
        R: Cell size (world_size / Resolution)
        Resolution: Grid resolution

    Returns:
        List of (i, j) map coordinates from start to goal, or None if no path.
    """
    n = len(grid)

    # Convert world coordinates to map coordinates
    start_map = Convert_world_to_map(start[0], start[1], R, Resolution)
    start_map = tuple(start_map)
    goal_map = Convert_world_to_map(goal[0], goal[1], R, Resolution)
    goal_map = tuple(goal_map)

    # Check if start or goal is an obstacle
    # if grid[start_map[0]][start_map[1]].isObstacle():
    #   print(f"Goal start {goal_map} is an obstacle!")
    #  return None

    if grid[goal_map[0]][goal_map[1]].isObstacle():
        print(f"Goal position {goal_map} is an obstacle! !!!!!!")
        return None

    # Priority queue: (f_cost, node)
    open_set = []
    heapq.heappush(open_set, (0, start_map))

    came_from = {}
    g_score = {start_map: 0}  # cost_to_come

    # Track visited nodes to avoid reprocessing
    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        # Skip if already processed
        if current in closed_set:
            continue
        closed_set.add(current)

        # Goal reached
        if current == goal_map:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_map)
            return path[::-1]  # Reverse: start -> goal

        for neighbor in get_neighbors(current, n):
            if neighbor in closed_set:
                continue

            neighbor_cell = grid[neighbor[0]][neighbor[1]]

            # Skip obstacles
            if neighbor_cell.isObstacle():
                continue

            terrain_cost = neighbor_cell.getTerrainCost()

            # Cost = previous cost + 1 (movement) + terrain penalty
            tentative_g = g_score[current] + 1 + terrain_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal_map)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current

    # At the end of astar, before "return None"
    print(f"A* exhausted. Explored {len(closed_set)} cells")
    print(f"Closed set: {closed_set}")
    print(f"Start map: {start_map}, Goal map: {goal_map}")

    return None  # No path found
