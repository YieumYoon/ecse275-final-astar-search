# Project Overview and Improvement Plan

## Executive Summary

This project implements a **multi-robot autonomous navigation system** using CoppeliaSim simulation with A\* pathfinding. The system features LiDAR and vision sensor integration, terrain mapping, and coordinated goal assignment for multiple robots performing search and rescue operations.

---

## Project Architecture

### File Structure

```
ecse275-final-astar-search/
├── Final_project_main.py    # Main entry point - robot control threads & coordination
├── FP_funcs.py              # Core utilities - sensors, mapping, A* algorithm
├── final_project.ttt        # CoppeliaSim scene file
└── README.md                # Project documentation
```

### Core Components

| Component              | File                    | Description                           |
| ---------------------- | ----------------------- | ------------------------------------- |
| Multi-Robot Controller | `Final_project_main.py` | Thread-based robot coordination       |
| Sensor Processing      | `FP_funcs.py`           | LiDAR & Vision sensor data processing |
| Terrain Mapping        | `FP_funcs.py`           | Occupancy grid with terrain types     |
| A\* Pathfinding        | `FP_funcs.py`           | 4-connected grid navigation           |
| Goal Management        | `Final_project_main.py` | Multi-goal assignment & tracking      |

### Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                        CoppeliaSim Scene                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │ Robot_0  │  │ Robot_1  │  │  Goals   │  │ Terrain  │            │
│  │ ├─LiDAR  │  │ ├─LiDAR  │  └──────────┘  └──────────┘            │
│  │ └─Vision │  │ └─Vision │                                         │
│  └──────────┘  └──────────┘                                         │
└───────────────────────────┬─────────────────────────────────────────┘
                            │ ZMQ Remote API
                            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     Python Control System                           │
│  ┌─────────────────┐                                                │
│  │ Main Thread     │ Initializes robots, goals, spawns threads     │
│  └────────┬────────┘                                                │
│           │                                                         │
│  ┌────────▼────────┐  ┌─────────────────┐                          │
│  │ Robot Thread 0  │  │ Robot Thread 1  │  Each has own ZMQ client │
│  │ ├─Scan LiDAR    │  │ ├─Scan LiDAR    │                          │
│  │ ├─Process Vision│  │ ├─Process Vision│                          │
│  │ ├─Update Map    │  │ ├─Update Map    │                          │
│  │ └─A* + Navigate │  │ └─A* + Navigate │                          │
│  └─────────────────┘  └─────────────────┘                          │
│           │                    │                                    │
│           └────────┬───────────┘                                    │
│                    ▼                                                │
│  ┌─────────────────────────────────────────┐                       │
│  │         Shared Resources (with locks)    │                       │
│  │  ├─ worldmap (map_lock)                 │                       │
│  │  └─ goals_data (goals_lock)             │                       │
│  └─────────────────────────────────────────┘                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Current Implementation Analysis

### Strengths ✅

1. **Multi-threading architecture** - Each robot has its own ZMQ client (ZMQ is not thread-safe)
2. **Thread-safe shared resources** - Uses `threading.Lock()` for map and goal access
3. **Modular sensor processing** - Separate functions for LiDAR and vision
4. **Terrain-aware pathfinding** - A\* considers terrain costs
5. **Dynamic goal assignment** - Robots find nearest uncompleted goals
6. **Good documentation** - README covers installation and usage

### Issues & Bugs 🐛

#### Critical Issues

1. **Import Statement Bug** (Line 9, `Final_project_main.py`)

   ```python
   # Current (incorrect)
   import coppeliasim_zmqremoteapi_client as zmq

   # Should be
   from coppeliasim_zmqremoteapi_client import RemoteAPIClient
   ```

   The current import style doesn't match the official API pattern.

2. **Inconsistent API Usage** (Lines 44-45, `Final_project_main.py`)

   ```python
   # Current
   client = zmq.RemoteAPIClient()
   sim = client.getObject('sim')

   # Official pattern uses
   sim = client.require('sim')
   ```

3. **TerrainCost Match Bug** (`FP_funcs.py`, lines 319-329)

   ```python
   def getTerrainCost(self):
       match self.terrain:  # self.terrain is TerrainType enum, not int
           case 0:  # Should be TerrainType.floor
               return 0
   ```

   Comparing enum to integers won't work correctly.

4. **Unused Function** (`FP_funcs.py`, line 479)
   ```python
   def a_star_path_to_coppelia_points(...)  # References non-existent getTerrain()
   ```

#### Medium Issues

5. **Variable Shadowing** (`FP_funcs.py`, line 206)

   ```python
   for i in range(len(terrain_array)):
       ...
       i, j = Terrain_map_coord  # 'i' shadows loop variable
   ```

6. **Hardcoded Values**

   - Map size: `10` (world units) hardcoded in multiple places
   - FOV: `60` degrees hardcoded
   - Scan interval: `1` second fixed

7. **No Error Recovery** - If A\* returns `None`, robot just continues without path

8. **Synchronization Mode** - Not using CoppeliaSim's stepping mode for deterministic simulation

---

## Improvement Plan

### Phase 1: Critical Bug Fixes (Priority: HIGH)

#### 1.1 Fix Import and API Pattern

```python
# Final_project_main.py - Line 9
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# And update usage throughout:
client = RemoteAPIClient()
sim = client.require('sim')  # Use require() instead of getObject()
```

#### 1.2 Fix TerrainCost Match Statement

```python
# FP_funcs.py - getTerrainCost method
def getTerrainCost(self):
    match self.terrain:
        case TerrainType.floor:
            return 0
        case TerrainType.grass:
            return 2
        case TerrainType.Sand:
            return 4
        case TerrainType.water:
            return 8
        case TerrainType.obstacle:
            return math.inf
    return 0
```

#### 1.3 Fix Variable Shadowing

```python
# FP_funcs.py - Update_map function
def Update_map(grid, terrain_array, Resolution):
    n = len(grid)
    for idx in range(len(terrain_array)):  # Use 'idx' instead of 'i'
        currTerrain = terrain_array[idx]
        Terrain_world_coord = currTerrain.getCoordinateArray()
        Terrain_map_coord = Convert_world_to_map(...)
        i, j = Terrain_map_coord  # Now safe
        ...
```

### Phase 2: Code Quality & Convention (Priority: MEDIUM)

#### 2.1 PEP 8 Naming Conventions

| Current                 | Recommended                                 |
| ----------------------- | ------------------------------------------- |
| `FP_funcs.py`           | `fp_functions.py`                           |
| `TerrainType.Sand`      | `TerrainType.SAND` (enum members uppercase) |
| `Convert_world_to_map`  | `convert_world_to_map`                      |
| `Resolution` (variable) | `resolution`                                |
| `Xw, Yw`                | `x_world, y_world`                          |

#### 2.2 Type Hints (Python 3.9+)

```python
from typing import List, Tuple, Optional
import numpy as np
from numpy.typing import NDArray

def convert_world_to_map(
    x_world: float,
    y_world: float,
    cell_size: float,
    resolution: int
) -> Tuple[int, int]:
    ...

def astar(
    grid: NDArray[np.object_],
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    cell_size: float,
    resolution: int
) -> Optional[List[Tuple[int, int]]]:
    ...
```

#### 2.3 Configuration Management

```python
# config.py (new file)
from dataclasses import dataclass

@dataclass
class MapConfig:
    resolution: int = 100
    world_size: float = 10.0

    @property
    def cell_size(self) -> float:
        return self.world_size / self.resolution

@dataclass
class SensorConfig:
    lidar_threshold: float = 0.2
    vision_fov_deg: float = 60.0
    scan_interval: float = 1.0

@dataclass
class RobotConfig:
    names: list = None
    def __post_init__(self):
        if self.names is None:
            self.names = ["/Robot_0", "/Robot_1"]
```

### Phase 3: Architecture Improvements (Priority: MEDIUM)

#### 3.1 Use CoppeliaSim Stepping Mode

```python
# For deterministic, synchronized simulation
client = RemoteAPIClient()
sim = client.require('sim')

sim.setStepping(True)
sim.startSimulation()

while running:
    # Process all robots
    for robot in robots:
        robot.scan_and_plan()

    # Advance simulation by one step
    sim.step()

sim.stopSimulation()
```

#### 3.2 Use `client.getScriptFunctions()` for cleaner script calls

```python
# Current approach
sim.callScriptFunction('setPath', script_handle, world_path)

# Modern approach (recommended by CoppeliaSim)
script = sim.getObject('/Robot_0')
funcs = client.getScriptFunctions(script)
funcs.setPath(world_path)
```

#### 3.3 Improved Error Handling

```python
class PathfindingError(Exception):
    pass

def find_path_with_retry(robot_name, worldmap, start, goal, R, Resolution, max_retries=3):
    """Attempt pathfinding with fallback strategies."""
    for attempt in range(max_retries):
        with map_lock:
            path = astar(worldmap, start, goal, R, Resolution)

        if path is not None:
            return path

        # Fallback: expand search area, reduce obstacles temporarily
        print(f"[{robot_name}] Retry {attempt + 1}/{max_retries}")
        time.sleep(0.5)

    raise PathfindingError(f"No path found after {max_retries} attempts")
```

### Phase 4: Feature Enhancements (Priority: LOW)

#### 4.1 8-Connected A\* Movement

```python
def get_neighbors_8connected(pos, grid_size):
    """8-connected neighborhood including diagonals."""
    i, j = pos
    n = grid_size
    neighbors = []

    for di in [-1, 0, 1]:
        for dj in [-1, 0, 1]:
            if di == 0 and dj == 0:
                continue
            ni, nj = i + di, j + dj
            if 0 <= ni < n and 0 <= nj < n:
                # Diagonal cost is sqrt(2) ≈ 1.414
                cost = 1.414 if di != 0 and dj != 0 else 1.0
                neighbors.append(((ni, nj), cost))

    return neighbors
```

#### 4.2 Path Smoothing

```python
def smooth_path(path: List[Tuple[int, int]], grid) -> List[Tuple[int, int]]:
    """Remove unnecessary waypoints using line-of-sight checks."""
    if len(path) <= 2:
        return path

    smoothed = [path[0]]
    current_idx = 0

    while current_idx < len(path) - 1:
        # Find furthest visible point
        for check_idx in range(len(path) - 1, current_idx, -1):
            if has_line_of_sight(path[current_idx], path[check_idx], grid):
                smoothed.append(path[check_idx])
                current_idx = check_idx
                break

    return smoothed
```

#### 4.3 Async API Support (for better performance)

```python
# Using asyncio version for parallel sensor reads
from coppeliasim_zmqremoteapi_client.asyncio import RemoteAPIClient
import asyncio

async def read_all_sensors(robots):
    """Read all robot sensors in parallel."""
    tasks = []
    for robot in robots:
        tasks.append(read_robot_sensors(robot))

    return await asyncio.gather(*tasks)
```

---

## Refactored Code Examples

### Improved terrain class

```python
# FP_funcs.py
from enum import Enum, auto
from dataclasses import dataclass
from typing import Tuple, Optional
import math

class TerrainType(Enum):
    FLOOR = auto()
    GRASS = auto()
    SAND = auto()
    WATER = auto()
    OBSTACLE = auto()

TERRAIN_COSTS = {
    TerrainType.FLOOR: 0,
    TerrainType.GRASS: 2,
    TerrainType.SAND: 4,
    TerrainType.WATER: 8,
    TerrainType.OBSTACLE: math.inf,
}

@dataclass
class Terrain:
    """Represents a terrain cell in the occupancy grid."""
    x: float
    y: float
    terrain_type: TerrainType = TerrainType.FLOOR
    width: float = 0.0

    @property
    def cost(self) -> float:
        return TERRAIN_COSTS.get(self.terrain_type, 0)

    @property
    def is_obstacle(self) -> bool:
        return self.terrain_type == TerrainType.OBSTACLE

    @property
    def is_traversable(self) -> bool:
        return self.terrain_type != TerrainType.OBSTACLE

    @property
    def coordinates(self) -> Tuple[float, float]:
        return (self.x, self.y)
```

### Improved Main Entry Point

```python
# Final_project_main.py (header)
#!/usr/bin/env python3
"""
Multi-Robot Navigation System for CoppeliaSim

This module implements a coordinated multi-robot navigation system
using A* pathfinding, LiDAR, and vision sensors.

Author: [Your Name]
Date: 2025
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Dict, List, Optional
import threading
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(threadName)s] %(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)
```

---

## Testing Checklist

- [ ] Unit tests for coordinate conversion functions
- [ ] Unit tests for A\* algorithm with known maps
- [ ] Integration test: Single robot navigation
- [ ] Integration test: Multi-robot coordination
- [ ] Stress test: Large map resolution
- [ ] Edge case: All goals blocked
- [ ] Edge case: Robot starts on obstacle

---

## Implementation Priority

| Priority  | Task                   | Effort | Impact |
| --------- | ---------------------- | ------ | ------ |
| 🔴 HIGH   | Fix import statements  | Low    | High   |
| 🔴 HIGH   | Fix TerrainCost match  | Low    | High   |
| 🔴 HIGH   | Fix variable shadowing | Low    | Medium |
| 🟡 MEDIUM | Add type hints         | Medium | Medium |
| 🟡 MEDIUM | Use stepping mode      | Medium | High   |
| 🟡 MEDIUM | Extract configuration  | Medium | Medium |
| 🟢 LOW    | 8-connected movement   | Medium | Low    |
| 🟢 LOW    | Path smoothing         | Medium | Low    |
| 🟢 LOW    | Async API usage        | High   | Medium |

---

## References

- [CoppeliaSim ZMQ Remote API - Official Python Client](https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python)
- [CoppeliaSim API Documentation](https://www.coppeliarobotics.com/helpFiles/)
- [PEP 8 Style Guide](https://pep8.org/)
- [Python Type Hints](https://docs.python.org/3/library/typing.html)
