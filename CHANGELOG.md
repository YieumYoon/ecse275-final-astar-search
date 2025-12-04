# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] - 2025-12-03

### Fixed

- **Import Statement Bug** - Changed from `import coppeliasim_zmqremoteapi_client as zmq` to `from coppeliasim_zmqremoteapi_client import RemoteAPIClient` to match official CoppeliaSim API pattern
- **API Usage** - Changed from `client.getObject('sim')` to `client.require('sim')` as per official documentation
- **TerrainCost Match Statement** - Fixed comparison in `getTerrainCost()` method to use `TerrainType` enum values instead of integers (was comparing enum to int which never matched)
- **Variable Shadowing** - Fixed loop variable `i` being overwritten in `Update_map()` function by renaming loop variable to `idx`
- **TerrainType Enum Naming** - Changed to uppercase (`FLOOR`, `GRASS`, `SAND`, `WATER`, `OBSTACLE`) per PEP 8 conventions
- **Cleaned Up Dead Code** - Removed commented-out code in `astar()` function

### Added

- **Project Overview Document** - Created `PROJECT_OVERVIEW_AND_IMPROVEMENTS.md` with comprehensive analysis
- **Configuration Module** - Created `config.py` with centralized configuration using dataclasses:
  - `MapConfig` - Map resolution and world size
  - `SensorConfig` - LiDAR and vision sensor parameters
  - `NavigationConfig` - Scan interval and goal threshold
  - `RobotConfig` - Robot names and sensor suffixes
  - `GoalConfig` - Goal point names
  - `LoggingConfig` - Logging level and format
  - `TerrainCosts` - Cost values for terrain types
  - `TerrainWidthConfig` - Width values for vision-detected and LiDAR obstacles
- **Type Hints** - Added Python type hints throughout `FP_funcs.py` using `typing` and `numpy.typing`
- **Logging Infrastructure** - Added configurable logging via `LoggingConfig` dataclass
- **Goal Cell Protection** - Goals are re-protected from obstacles after each map update
- **Graceful Shutdown** - Added signal handlers (`SIGINT`, `SIGTERM`) and `shutdown_event` for clean termination
- **8-Connected A\* Movement** - Added diagonal movement support to A\* algorithm:
  - New `get_neighbors_8connected()` function with 8 directional moves
  - `DIAGONAL_COST = √2` for proper diagonal distance calculation
  - Corner-cutting prevention (blocks diagonal moves when adjacent cells are obstacles)
  - `use_8_connected` parameter in `astar()` function
  - `use_8_connected` config option in `NavigationConfig`
- **Path Smoothing** - Added line-of-sight based path smoothing:
  - `has_line_of_sight()` function using Bresenham's line algorithm
  - `smooth_path()` function for basic path optimization
  - `smooth_path_with_terrain()` for terrain-aware smoothing (avoids high-cost cells)
  - `use_path_smoothing` config option in `NavigationConfig`
- **Changelog** - Added this CHANGELOG.md file

### Changed

- **PEP 8 Function Naming** - Renamed functions to follow snake_case convention with backward compatibility aliases:
  - `Convert_world_to_map` → `convert_world_to_map`
  - `Convert_map_to_world` → `convert_map_to_world`
  - `Update_map` → `update_map`
  - `createMap_withResolution` → `create_map_with_resolution`
  - `get_cells_to_fill` → `_get_cells_to_fill` (private function)
- **Terrain Widths** - Moved hardcoded terrain width values (0.5, 1.0) to `TerrainWidthConfig` in `config.py`

### Documentation

- Updated `PROJECT_OVERVIEW_AND_IMPROVEMENTS.md` with:
  - Current implementation status
  - Accurate list of working features
  - Updated improvement plan with completed tasks
  - Remaining low-priority enhancements

## Previous Version

### Features

- Multi-robot autonomous navigation using A\* pathfinding
- LiDAR sensor integration for obstacle detection
- Vision sensor processing for terrain classification (red, green, blue)
- Thread-based multi-robot coordination
- Thread-safe shared map access
- Dynamic goal assignment system

### Known Issues (all addressed in this release)

- ~~Import pattern didn't match official API~~
- ~~Terrain cost function had type mismatch~~
- ~~Variable shadowing in map update function~~
- ~~Enum members not uppercase~~
- ~~No type hints~~
- ~~No logging support~~
