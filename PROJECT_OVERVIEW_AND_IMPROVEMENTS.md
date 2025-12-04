# Project Overview and Improvement Plan

## Executive Summary

This project implements a **multi-robot autonomous navigation system** using CoppeliaSim simulation with A\* pathfinding. The system features LiDAR and vision sensor integration, terrain mapping, and coordinated goal assignment for multiple robots performing search and rescue operations.

**Last Updated:** December 3, 2025

---

## Project Architecture

### File Structure

```
ecse275-final-astar-search/
├── Final_project_main.py    # Main entry point - robot control threads & coordination
├── FP_funcs.py              # Core utilities - sensors, mapping, A* algorithm
├── config.py                # Centralized configuration (MapConfig, SensorConfig, etc.)
├── final_project.ttt        # CoppeliaSim scene file
├── CHANGELOG.md             # Version history and changes
├── PROJECT_OVERVIEW_AND_IMPROVEMENTS.md  # This document
├── README.md                # Project documentation
├── coppeliasim_script/      # Lua scripts for CoppeliaSim scene
│   ├── Robot_0.lua
│   ├── Robot_1.lua
│   ├── Robot_0_fastHokuyo_0.lua
│   ├── Robot_1_fastHokuyo_0.lua
│   └── coppeliasim_scene_hierarchy.md
└── __pycache__/             # Python bytecode cache
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
10. **Type hints** - `FP_funcs.py` uses modern Python type hints with `typing` module
11. **Logging support** - Configurable logging via `LoggingConfig` dataclass
12. **Goal cell protection** - Goals are re-protected from being marked as obstacles after map updates

### Issues & Bugs 🐛

#### ~~Critical Issues~~ (ALL RESOLVED ✅)

1. ~~**Import Statement Bug**~~ - **FIXED**: Now correctly uses `from coppeliasim_zmqremoteapi_client import RemoteAPIClient`

2. ~~**Inconsistent API Usage**~~ - **FIXED**: Now uses `client.require('sim')` pattern

3. ~~**TerrainCost Match Bug**~~ - **FIXED**: Now compares against `TerrainType.FLOOR`, `TerrainType.GRASS`, etc.

4. ~~**Variable Shadowing**~~ - **FIXED**: Loop variable renamed to `idx` in `Update_map()`

#### ~~Remaining Issues~~ (ALL RESOLVED ✅)

1. ~~**Unused Code**~~ - **FIXED**: Cleaned up commented code in `astar()` function

2. ~~**PEP 8 Naming**~~ - **FIXED**: Function names now follow PEP 8 conventions:

   - `convert_world_to_map()` (with backward compatibility alias)
   - `convert_map_to_world()` (with backward compatibility alias)
   - `update_map()` (with backward compatibility alias)
   - `create_map_with_resolution()` (with backward compatibility alias)

3. ~~**Hardcoded terrain widths**~~ - **FIXED**: Now uses `cfg.terrain_width.vision_detected` and `cfg.terrain_width.lidar_obstacle`

4. ~~**No graceful shutdown**~~ - **FIXED**: Added signal handlers and `shutdown_event` for clean termination

5. **Synchronization Mode** - Not using CoppeliaSim's stepping mode for deterministic simulation _(Future enhancement)_

---

## Improvement Plan

### ~~Phase 1: Critical Bug Fixes~~ (COMPLETED ✅)

All critical bugs have been resolved in the current implementation:

- ✅ Import statements now use `from coppeliasim_zmqremoteapi_client import RemoteAPIClient`
- ✅ API calls use `client.require('sim')` pattern
- ✅ TerrainCost match statement uses proper enum members (`TerrainType.FLOOR`, etc.)
- ✅ Variable shadowing fixed (uses `idx` instead of `i`)
- ✅ TerrainType enum uses uppercase naming (`FLOOR`, `GRASS`, `SAND`, `WATER`, `OBSTACLE`)
- ✅ Type hints added to `FP_funcs.py`
- ✅ Logging infrastructure added via `LoggingConfig`

### ~~Phase 2: Code Quality & Cleanup~~ (COMPLETED ✅)

- ✅ Cleaned up commented code in `astar()` function
- ✅ Moved terrain widths to `TerrainWidthConfig` in `config.py`
- ✅ Added graceful shutdown with signal handlers (`SIGINT`, `SIGTERM`)
- ✅ PEP 8 function naming with backward compatibility aliases

### Phase 3: Architecture Improvements (Priority: LOW)

#### 3.1 Use CoppeliaSim Stepping Mode (Future Enhancement)

> **Note:** Currently disabled because stepping mode conflicts with multi-threaded architecture.
> Each robot thread has its own ZMQ client and timing. Would require refactoring to single-threaded event loop.

```python
# For deterministic, synchronized simulation (single-threaded)
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

#### 3.2 Logging Best Practices

Current implementation already includes logging:

```python
# Final_project_main.py
logging.basicConfig(
    level=cfg.logging.get_level(),
    format=cfg.logging.format,
    datefmt=cfg.logging.date_format
)
logger = logging.getLogger(__name__)
```

**Enhancement:** Replace `print()` statements with `logger.info()`, `logger.debug()`, etc. for consistent output control.

### ~~Phase 4: Feature Enhancements~~ (PARTIALLY COMPLETED ✅)

#### ~~4.1 8-Connected A\* Movement~~ (COMPLETED ✅)

Implemented in `FP_funcs.py`:

- `get_neighbors_4connected()` - Original 4-direction movement
- `get_neighbors_8connected()` - New 8-direction movement including diagonals
- `get_neighbors()` - Unified function with `use_8_connected` parameter
- Diagonal moves have cost √2 ≈ 1.414
- Corner-cutting prevention: diagonal moves blocked if adjacent cells are obstacles

Enable via config: `cfg.navigation.use_8_connected = True`

#### ~~4.2 Path Smoothing~~ (COMPLETED ✅)

Implemented in `FP_funcs.py`:

- `has_line_of_sight()` - Bresenham's line algorithm for obstacle checking
- `smooth_path()` - Removes unnecessary waypoints using line-of-sight
- `smooth_path_with_terrain()` - Variant that also avoids high-cost terrain

Enable via config: `cfg.navigation.use_path_smoothing = True`

#### 4.3 Dynamic Obstacle Avoidance

Add real-time obstacle detection and path replanning when obstacles appear in the planned path.

#### 4.4 Visualization Dashboard

Add matplotlib-based real-time visualization of:

- Robot positions
- Current paths
- Obstacle map
- Goal status

---

## Current Implementation Details

### Working Features (Verified December 3, 2025)

1. **Multi-robot coordination** - Two robots (`/Robot_0`, `/Robot_1`) run in separate threads
2. **Goal assignment** - Nearest unassigned goal is selected dynamically
3. **LiDAR obstacle detection** - Uses `process_Lidar_depth()` with configurable threshold
4. **Vision-based terrain detection** - Red→obstacle, Green→grass, Blue→water
5. **A\* pathfinding** - 4-connected grid with terrain costs
6. **Thread-safe map updates** - Uses `map_lock` and `goals_lock`
7. **Lua script integration** - Paths sent via `setPath()`, status via `getGoalStatus()`
8. **Centralized configuration** - All parameters configurable via `config.py`
9. **Error recovery** - A\* failures trigger retry on next scan cycle
10. **Goal cell protection** - Goals are marked as `FLOOR` after each map update to prevent blocking
11. **Logging infrastructure** - Configured via `LoggingConfig` dataclass
12. **Type hints** - Modern Python typing in `FP_funcs.py`
13. **Graceful shutdown** - Signal handlers for clean termination (Ctrl+C)
14. **PEP 8 compliant** - Function names follow snake_case with backward compatibility

### Configuration Parameters (via `config.py`)

| Parameter            | Default                  | Config Location                     |
| -------------------- | ------------------------ | ----------------------------------- |
| Map Resolution       | 100x100                  | `cfg.map.resolution`                |
| World Size           | 10m                      | `cfg.map.world_size`                |
| Cell Size (R)        | 0.1m                     | `cfg.map.cell_size`                 |
| Scan Interval        | 1s                       | `cfg.navigation.scan_interval`      |
| Vision FOV           | 60°                      | `cfg.sensor.vision_fov_deg`         |
| LiDAR Threshold      | 0.2m                     | `cfg.sensor.lidar_threshold`        |
| Vision Terrain Width | 0.5m                     | `cfg.terrain_width.vision_detected` |
| LiDAR Obstacle Width | 1.0m                     | `cfg.terrain_width.lidar_obstacle`  |
| Robot Names          | ["/Robot_0", "/Robot_1"] | `cfg.robots.names`                  |
| Goal Names           | 5 goal points            | `cfg.goals.names`                   |
| Log Level            | INFO                     | `cfg.logging.level`                 |
| Log Format           | Thread-aware format      | `cfg.logging.format`                |

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

| Priority      | Task                                   | Effort | Impact | Status      |
| ------------- | -------------------------------------- | ------ | ------ | ----------- |
| ~~🔴 HIGH~~   | ~~Fix import statements~~              | Low    | High   | ✅ DONE     |
| ~~🔴 HIGH~~   | ~~Fix TerrainCost match~~              | Low    | High   | ✅ DONE     |
| ~~🔴 HIGH~~   | ~~Fix variable shadowing~~             | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Integrate config.py~~                | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Add A\* error recovery~~             | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Add type hints~~                     | Medium | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Add logging infrastructure~~         | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Uppercase enum members~~             | Low    | Low    | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Clean up commented code in astar()~~ | Low    | Low    | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Move terrain widths to config~~      | Low    | Low    | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~Add graceful shutdown~~              | Low    | Medium | ✅ DONE     |
| ~~🟡 MEDIUM~~ | ~~PEP 8 function naming~~              | Medium | Low    | ✅ DONE     |
| 🟢 LOW        | Use stepping mode (requires refactor)  | High   | Medium | Not Started |
| ~~🟢 LOW~~    | ~~8-connected movement~~               | Medium | Low    | ✅ DONE     |
| ~~🟢 LOW~~    | ~~Path smoothing~~                     | Medium | Low    | ✅ DONE     |
| 🟢 LOW        | Visualization dashboard                | High   | Low    | Not Started |

---

## References

- [CoppeliaSim ZMQ Remote API - Official Python Client](https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python)
- [CoppeliaSim API Documentation](https://www.coppeliarobotics.com/helpFiles/)
- [PEP 8 Style Guide](https://pep8.org/)
- [Python Type Hints](https://docs.python.org/3/library/typing.html)
