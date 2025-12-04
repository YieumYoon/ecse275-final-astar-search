# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] - 2025-12-03

### Fixed

- **Import Statement Bug** - Changed from `import coppeliasim_zmqremoteapi_client as zmq` to `from coppeliasim_zmqremoteapi_client import RemoteAPIClient` to match official CoppeliaSim API pattern
- **API Usage** - Changed from `client.getObject('sim')` to `client.require('sim')` as per official documentation
- **TerrainCost Match Statement** - Fixed comparison in `getTerrainCost()` method to use `TerrainType` enum values instead of integers (was comparing enum to int which never matched)
- **Variable Shadowing** - Fixed loop variable `i` being overwritten in `Update_map()` function by renaming loop variable to `idx`

### Added

- **Project Overview Document** - Created `PROJECT_OVERVIEW_AND_IMPROVEMENTS.md` with comprehensive analysis
- **Configuration Module** - Created `config.py` with centralized configuration using dataclasses
- **Changelog** - Added this CHANGELOG.md file

### Documentation

- Added architecture diagrams
- Added data flow documentation
- Added improvement plan with prioritized tasks
- Added code examples for future refactoring

## Previous Version

### Features

- Multi-robot autonomous navigation using A\* pathfinding
- LiDAR sensor integration for obstacle detection
- Vision sensor processing for terrain classification (red, green, blue)
- Thread-based multi-robot coordination
- Thread-safe shared map access
- Dynamic goal assignment system

### Known Issues (addressed in this release)

- ~~Import pattern didn't match official API~~
- ~~Terrain cost function had type mismatch~~
- ~~Variable shadowing in map update function~~
