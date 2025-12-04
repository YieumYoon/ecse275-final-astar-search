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
├── config.py                # Centralized configuration (MapConfig, SensorConfig, etc.)
├── final_project.ttt        # CoppeliaSim scene file
├── coppeliasim_script/      # Lua scripts for CoppeliaSim scene
│   ├── Robot_0.lua
│   ├── Robot_1.lua
│   ├── Robot_0_fastHokuyo_0.lua
│   ├── Robot_1_fastHokuyo_0.lua
│   └── coppeliasim_scene_hierarchy.md
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
| Configuration          | `config.py`             | Centralized project configuration     |

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
7. **Centralized configuration** - `config.py` provides dataclass-based configuration management
8. **Correct API usage** - Uses `RemoteAPIClient()` and `client.require('sim')` patterns
9. **Proper enum comparisons** - TerrainCost uses `TerrainType` enum members correctly

### Issues & Bugs 🐛

#### ~~Critical Issues~~ (RESOLVED ✅)

1. ~~**Import Statement Bug**~~ - **FIXED**: Now correctly uses `from coppeliasim_zmqremoteapi_client import RemoteAPIClient`

2. ~~**Inconsistent API Usage**~~ - **FIXED**: Now uses `client.require('sim')` pattern

3. ~~**TerrainCost Match Bug**~~ - **FIXED**: Now compares against `TerrainType.floor`, `TerrainType.grass`, etc.

4. ~~**Variable Shadowing**~~ - **FIXED**: Loop variable renamed to `idx` in `Update_map()`

#### ~~Remaining Medium Issues~~ (MOSTLY RESOLVED ✅)

1. **Unused Function** (`FP_funcs.py`, line 479)

   ```python
   def a_star_path_to_coppelia_points(...)  # References non-existent getTerrain() method
   ```

   The `getTerrain()` method doesn't exist on the terrain class. _(Still needs fix or removal)_

2. ~~**Hardcoded Values**~~ - **FIXED**: All values now use `config.py`:

   - ✅ Map size: Now uses `cfg.map.world_size`
   - ✅ FOV: Now uses `cfg.sensor.vision_fov_deg`
   - ✅ Scan interval: Now uses `cfg.navigation.scan_interval`
   - ✅ LiDAR threshold: Now uses `cfg.sensor.lidar_threshold`
   - ✅ Robot names: Now uses `cfg.robots.names`
   - ✅ Goal names: Now uses `cfg.goals.names`

3. ~~**Config not fully integrated**~~ - **FIXED**: `config.py` is now imported and used throughout main files

4. ~~**No Error Recovery**~~ - **FIXED**: A\* failures now log detailed message and retry on next scan cycle

5. **Synchronization Mode** - Not using CoppeliaSim's stepping mode for deterministic simulation _(Future enhancement)_

---

## Improvement Plan

### ~~Phase 1: Critical Bug Fixes~~ (COMPLETED ✅)

All critical bugs have been resolved in the current implementation:

- ✅ Import statements now use `from coppeliasim_zmqremoteapi_client import RemoteAPIClient`
- ✅ API calls use `client.require('sim')` pattern
- ✅ TerrainCost match statement uses proper enum members
- ✅ Variable shadowing fixed (uses `idx` instead of `i`)

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

A `config.py` file has been created with dataclass-based configuration. **Next step**: integrate it into the main files.

```python
# config.py (already exists!)
from dataclasses import dataclass, field
from typing import List

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
    vision_resolution: tuple = (256, 256)

@dataclass
class NavigationConfig:
    scan_interval: float = 1.0
    goal_reached_threshold: float = 0.5
    use_8_connected: bool = False

@dataclass
class RobotConfig:
    names: List[str] = field(default_factory=lambda: ["/Robot_0", "/Robot_1"])
    lidar_sensor_suffix: str = "/fastHokuyo_0"
    vision_sensor_suffix: str = "/visionSensor"

@dataclass
class ProjectConfig:
    map: MapConfig = field(default_factory=MapConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    robots: RobotConfig = field(default_factory=RobotConfig)

# Usage in Final_project_main.py:
# from config import default_config as cfg
# Resolution = cfg.map.resolution
# R = cfg.map.cell_size
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

The current `terrain` class implementation is functional. Here's the existing code with suggested dataclass refactor:

```python
# Current implementation in FP_funcs.py (working)
class TerrainType(Enum):
    floor = 0
    grass = 1
    Sand = 2
    water = 3
    obstacle = 4

class terrain():
    def __init__(self, width=0, Coordinate=None, terrain: TerrainType = None, resolution=5):
        self.width = width
        self.obstacleCoords = Coordinate
        self.terrain = terrain
        self.resolution = resolution

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
    # ... other methods
```

**Suggested refactor** (for future improvement):

```python
# FP_funcs.py - Dataclass-based refactor
from enum import Enum, auto
from dataclasses import dataclass
from typing import Tuple
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
```

### Improved Main Entry Point

The current implementation already follows good practices:

```python
# Final_project_main.py (current - working)
#!/usr/bin/env python3
"""
Created on Thu Nov 13 20:21:34 2025
@author: halas
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import FP_funcs as Func
import math
import numpy as np
import time
import threading

# Global locks for thread-safe access
map_lock = threading.Lock()
goals_lock = threading.Lock()

# Global goals tracking
goals_data = {
    'positions': [],      # List of goal world positions [(x, y, z), ...]
    'completed': [],      # List of booleans tracking completion status
    'assigned_to': []     # List of robot names or None for each goal
}
```

**Suggested enhancement** (add logging):

```python
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

## Current Implementation Notes

### Working Features (as of Dec 2025)

1. **Multi-robot coordination** - Two robots (`/Robot_0`, `/Robot_1`) run in separate threads
2. **Goal assignment** - Nearest unassigned goal is selected dynamically
3. **LiDAR obstacle detection** - Uses `process_Lidar_depth()` with configurable threshold
4. **Vision-based terrain detection** - Red→obstacle, Green→grass, Blue→water
5. **A\* pathfinding** - 4-connected grid with terrain costs
6. **Thread-safe map updates** - Uses `map_lock` and `goals_lock`
7. **Lua script integration** - Paths sent via `setPath()`, status via `getGoalStatus()`
8. **Centralized configuration** - All parameters configurable via `config.py`
9. **Error recovery** - A\* failures trigger retry on next scan cycle

### Key Parameters (configurable via `config.py`)

| Parameter       | Default                  | Config Location                |
| --------------- | ------------------------ | ------------------------------ |
| Map Resolution  | 100x100                  | `cfg.map.resolution`           |
| World Size      | 10m                      | `cfg.map.world_size`           |
| Cell Size (R)   | 0.1m                     | `cfg.map.cell_size`            |
| Scan Interval   | 1s                       | `cfg.navigation.scan_interval` |
| Vision FOV      | 60°                      | `cfg.sensor.vision_fov_deg`    |
| LiDAR Threshold | 0.2m                     | `cfg.sensor.lidar_threshold`   |
| Robot Names     | ["/Robot_0", "/Robot_1"] | `cfg.robots.names`             |
| Goal Names      | ["/goal_point", ...]     | `cfg.goals.names`              |

---

## Implementation Priority

| Priority      | Task                                          | Effort | Impact | Status      |
| ------------- | --------------------------------------------- | ------ | ------ | ----------- |
| ~~🔴 HIGH~~   | ~~Fix import statements~~                     | Low    | High   | ✅ DONE     |
| ~~🔴 HIGH~~   | ~~Fix TerrainCost match~~                     | Low    | High   | ✅ DONE     |
| ~~🔴 HIGH~~   | ~~Fix variable shadowing~~                    | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Integrate config.py~~                       | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Add A\* error recovery~~                    | Low    | Medium | ✅ DONE     |
| 🟡 MEDIUM     | Add type hints                                | Medium | Medium | Not Started |
| 🟡 MEDIUM     | Use stepping mode                             | Medium | High   | Not Started |
| 🟡 MEDIUM     | Fix unused `a_star_path_to_coppelia_points()` | Low    | Low    | Not Started |
| 🟢 LOW        | 8-connected movement                          | Medium | Low    | Not Started |
| 🟢 LOW        | Path smoothing                                | Medium | Low    | Not Started |
| 🟢 LOW        | Async API usage                               | High   | Medium | Not Started |
| 🟢 LOW        | Add logging module                            | Low    | Medium | Not Started |

---

## References

- [CoppeliaSim ZMQ Remote API - Official Python Client](https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python)
- [CoppeliaSim API Documentation](https://www.coppeliarobotics.com/helpFiles/)
- [PEP 8 Style Guide](https://pep8.org/)
- [Python Type Hints](https://docs.python.org/3/library/typing.html)
