# Project Overview Report
**Robot Navigation System with Vision and LiDAR**

---

## Executive Summary

This project implements an autonomous robot navigation system in CoppeliaSim that combines vision sensors, LiDAR sensors, terrain mapping, and intelligent pathfinding. The system enables a robot to perceive its environment, build maps, detect colored objects, and navigate optimally through varied terrain using A* pathfinding.

**Status:** Version 8 - Functional but Unconsolidated  
**Development Stage:** Core functionality complete, integration pending  
**Primary Language:** Python 3  
**Simulation Platform:** CoppeliaSim

---

## Technical Architecture

### System Components

```
┌─────────────────────────────────────────────────────────┐
│                  CoppeliaSim Scene                      │
│  ┌──────────┐  ┌──────────┐  ┌────────────┐            │
│  │  Robot   │  │   Goal   │  │  Obstacles │            │
│  │  + LiDAR │  │  Point   │  │            │            │
│  │  + Camera│  │          │  │            │            │
│  └────┬─────┘  └────┬─────┘  └─────┬──────┘            │
└───────┼─────────────┼──────────────┼───────────────────┘
        │             │              │
        │    ZMQ Remote API          │
        └─────────────┼──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │   Final_project_main.py    │
        │  (Main Control Script)     │
        └─────────────┬──────────────┘
                      │
        ┌─────────────▼──────────────┐
        │      FP_funcs.py           │
        │  (Function Library)         │
        │                             │
        │  • Vision Processing        │
        │  • LiDAR Processing         │
        │  • Mapping System           │
        │  • A* Pathfinding          │
        │  • Coordinate Transforms    │
        └─────────────────────────────┘
```

### Data Flow

1. **Sensor Acquisition**
   - LiDAR captures 3D point cloud
   - Vision sensor captures RGB + Depth (256×256)

2. **Processing Pipeline**
   - Segment LiDAR points into objects
   - Transform sensor coordinates to world frame
   - Extract color masks for object detection
   - Convert pixel coordinates to 3D positions

3. **Map Building**
   - Create/update occupancy grid
   - Assign terrain types and costs
   - Mark obstacles and traversable areas

4. **Path Planning**
   - Compute optimal path using A*
   - Consider terrain costs
   - Output waypoints for robot control

---

## Functional Specifications

### Vision System

**Capabilities:**
- RGB image capture (256×256 resolution)
- Depth sensing with near/far plane conversion
- Color-based object detection (red, green, blue)
- Centroid computation for detected objects
- Camera intrinsic calibration (60° FOV)

**Key Functions:**
```python
process_vision_Sensor_RBG()      # Capture RGB image
process_vision_sensor_depth()     # Extract depth map
mask_color()                      # Detect specific colors
centroid_from_mask()              # Locate object centers
compute_pos_from_pix()            # 2D pixel → 3D world position
```

### LiDAR System

**Capabilities:**
- FastHokuyo sensor data acquisition
- Point cloud segmentation (threshold-based)
- Centroid extraction for object clusters
- Coordinate frame transformations

**Key Functions:**
```python
process_Lidar_depth()             # Process LiDAR scans
segment_lidar()                   # Cluster nearby points
transform_point()                 # Sensor → world coords
```

### Terrain Mapping

**Map Specifications:**
- Default resolution: 100×100 cells
- World size: 10m × 10m
- Cell size: 0.1m × 0.1m
- Origin at world center (0, 0)

**Terrain Types:**
| Type     | Cost | Traversable | Use Case                    |
|----------|------|-------------|-----------------------------|
| Floor    | 0    | Yes         | Clear path                  |
| Grass    | 2    | Yes         | Low difficulty terrain      |
| Sand     | 4    | Yes         | Medium difficulty terrain   |
| Water    | 8    | Yes         | High difficulty terrain     |
| Obstacle | ∞    | No          | Impassable objects          |

**Key Functions:**
```python
createMap_withResolution()        # Initialize map
Update_map()                      # Add terrain objects
Convert_world_to_map()            # Coordinate conversion
Convert_map_to_world()            # Reverse conversion
```

### Pathfinding

**A* Implementation:**
- **Connectivity:** 4-connected grid (up, down, left, right)
- **Heuristic:** Euclidean distance
- **Cost Function:** movement + terrain penalty
- **Output:** Sequence of (i, j) map coordinates

**Algorithm Properties:**
- Optimal path guarantee (admissible heuristic)
- Obstacle avoidance
- Terrain-aware routing
- Closed set prevents cycles

**Key Functions:**
```python
astar()                           # Main pathfinding algorithm
heuristic()                       # Distance estimation
get_neighbors()                   # Valid adjacent cells
```

---

## Code Structure

### Main Script (`Final_project_main.py` - 107 lines)

**Purpose:** Orchestrates the robot navigation system

**Key Operations:**
1. Establish ZMQ connection to CoppeliaSim
2. Query robot and goal positions
3. Initialize map with specified resolution
4. Process LiDAR data and segment obstacles
5. Capture vision sensor images (RGB + depth)
6. Detect colored objects (red mask example)
7. Transform detected objects to world frame
8. Display processed images

**Configuration:**
- Resolution: 100
- LiDAR segmentation threshold: 0.2m
- Target color: red
- Camera FOV: 60°

### Function Library (`FP_funcs.py` - 479 lines)

**Module Breakdown:**

| Category            | Lines  | Functions | Description                           |
|---------------------|--------|-----------|---------------------------------------|
| Vision Processing   | 1-150  | 7         | RGB, depth, color detection           |
| LiDAR Processing    | 151-195| 3         | Point cloud segmentation              |
| Mapping System      | 196-290| 7         | Map creation, updates, conversions    |
| Terrain Classes     | 291-360| 2         | Terrain and TerrainType definitions   |
| Pathfinding         | 361-479| 5         | A* algorithm and utilities            |

---

## Technical Details

### Coordinate Systems

**Three Reference Frames:**

1. **World Frame**
   - CoppeliaSim global coordinates
   - Origin at (0, 0, 0)
   - Used for robot positions, goals

2. **Sensor Frame**
   - Local to each sensor (LiDAR/camera)
   - Transformations via 3×4 matrices
   - Converted to world for mapping

3. **Map Frame**
   - Discrete grid indices (i, j)
   - Resolution-dependent scaling
   - Origin at map center

### Transformation Mathematics

**Pixel to 3D Coordinate:**
```
Given: pixel (u, v), depth z, FOV angle θ
view_width = 2 * z * tan(θ/2)
pixel_width = view_width / image_width
x = (u - cx) * pixel_width
y = -(v - cy) * pixel_height
position = (x, y, z)
```

**Sensor to World:**
```
Given: 3×4 transformation matrix M, point p
world_x = M[0]*p[0] + M[1]*p[1] + M[2]*p[2] + M[3]
world_y = M[4]*p[0] + M[5]*p[1] + M[6]*p[2] + M[7]
world_z = M[8]*p[0] + M[9]*p[1] + M[10]*p[2] + M[11]
```

**World to Map:**
```
i = Xw / cell_size + resolution/2
j = Yw / cell_size + resolution/2
```

### Color Detection Algorithm

**Strategy:** Dominant channel detection with threshold margin

```python
# Example for red detection
mask = (R > G + 20) AND (R > B + 20)
```

**Margin:** 20-unit separation ensures robust detection in varying lighting

---

## Performance Characteristics

### Computational Complexity

| Operation              | Complexity        | Notes                          |
|------------------------|-------------------|--------------------------------|
| LiDAR segmentation     | O(n)              | n = number of points           |
| Color mask creation    | O(w×h)            | w×h = image dimensions         |
| Map update             | O(k)              | k = terrain objects            |
| A* pathfinding         | O(n² log n)       | n = grid cells (worst case)    |

### Memory Requirements

- Map storage: `Resolution² × sizeof(terrain)` ≈ 100×100 objects
- RGB image: 256×256×3 bytes = 192 KB
- Depth map: 256×256×4 bytes = 256 KB
- Point cloud: Variable (depends on scene)

---

## Integration Points

### CoppeliaSim Scene Requirements

**Required Objects:**
- `/Robot_0` - Main robot body
- `/Robot_0/fastHokuyo_0` - LiDAR sensor
- `/Robot_0/visionSensor` - RGB-D camera
- `/goal_point` - Navigation target

**Object Properties:**
- Vision sensor: 256×256 resolution, depth enabled
- LiDAR: Child script with `getMeasuredData` function
- Objects properly named and parented

### API Dependencies

**ZMQ Remote API Functions Used:**
```python
sim.getObjectHandle()         # Query object by name
sim.getObjectPosition()       # Get object position
sim.getObjectMatrix()         # Get transformation matrix
sim.getVisionSensorImg()      # Capture RGB image
sim.getVisionSensorDepth()    # Capture depth buffer
sim.callScriptFunction()      # Execute LiDAR script
```

---

## Development Roadmap

### Current Status (Version 8)

✅ **Completed:**
- Vision sensor RGB/depth processing
- LiDAR data acquisition and segmentation
- Terrain mapping system with multiple types
- A* pathfinding with terrain costs
- Coordinate transformation utilities
- Color-based object detection
- All interface tools functional

### Pending Integration

🔄 **Next Steps:**

1. **Multi-Robot System**
   - Add support for multiple robots
   - Implement robot selection logic
   - Coordinate shared map updates

2. **Dynamic Camera Switching**
   - Switch between robot cameras
   - Optimize perception based on position
   - Aggregate multi-viewpoint data

3. **Real-Time Map Updates**
   - Integrate sensor data continuously
   - Trigger map updates based on events
   - Handle dynamic obstacles

4. **A* Integration**
   - Connect pathfinding to robot control
   - Implement path following
   - Add replanning on obstacle detection

5. **GitHub Setup**
   - Create repository structure
   - Add documentation
   - Version control consolidation

### Known Issues

- Code needs consolidation (per `Verision_8_unconsolidated.txt`)
- Main script demonstrates capability but lacks control loop
- No robot movement commands implemented yet
- Map updates not automated

---

## Testing & Validation

### Current Testing Approach

**Manual Verification:**
- Visual inspection of processed images
- Print statements for coordinate verification
- matplotlib display for RGB/depth maps

**Test Scenarios:**
- Single robot, single goal
- Static obstacles
- Color object detection (red target)

### Recommended Testing

**Unit Tests Needed:**
- Coordinate transformation accuracy
- Color mask precision/recall
- A* path optimality
- Terrain cost calculations

**Integration Tests Needed:**
- End-to-end sensor-to-action pipeline
- Multi-robot coordination
- Dynamic obstacle handling
- Real-time performance benchmarks

---

## Dependencies & Environment

### Python Packages

```
numpy          # Array operations, numerical computing
matplotlib     # Visualization, image display
math           # Mathematical functions
heapq          # Priority queue for A*
array          # Array data type
enum           # Enum class for terrain types
```

### External Systems

- **CoppeliaSim** (EDU or PRO version)
- **ZMQ Remote API** for Python
- **Conda environment:** `ecse275env`

### Installation

```bash
# Create conda environment
conda create -n ecse275env python=3.x

# Activate environment
conda activate ecse275env

# Install dependencies
pip install numpy matplotlib

# Install CoppeliaSim ZMQ Remote API
# (Follow CoppeliaSim documentation)
```

---

## Usage Example

### Basic Workflow

```python
# 1. Start CoppeliaSim and load scene
# 2. Run main script
python Final_project_main.py

# System will:
# - Connect to CoppeliaSim
# - Initialize sensors
# - Process LiDAR and vision data
# - Detect objects
# - Display results
```

### Extending Functionality

```python
# Example: Find path from robot to goal
path = astar(worldmap, robot_pos, goal_world, R, Resolution)

if path:
    # Convert path to waypoints
    waypoints = [Convert_map_to_world(i, j, R, Resolution) 
                 for (i, j) in path]
    # Send waypoints to robot controller
else:
    print("No path found!")
```

---

## Configuration Parameters

### Tunable Parameters

| Parameter            | Default | Location              | Impact                        |
|----------------------|---------|-----------------------|-------------------------------|
| Resolution           | 100     | main script           | Map granularity               |
| LiDAR threshold      | 0.2     | process_Lidar_depth() | Object segmentation           |
| Color margin         | 20      | mask_color()          | Detection sensitivity         |
| FOV                  | 60°     | main script           | Camera field of view          |
| Terrain costs        | 0-∞     | TerrainType class     | Pathfinding preferences       |

---

## Maintenance Notes

### Code Quality

**Strengths:**
- Modular function design
- Clear separation of concerns
- Comprehensive docstrings
- Type hints for terrain system

**Areas for Improvement:**
- Consolidate scattered functionality
- Add error handling and validation
- Implement unit tests
- Remove debug print statements
- Standardize naming conventions

### Documentation

**Current State:**
- Inline docstrings for most functions
- Code comments for complex operations
- Project notes in `Verision_8_unconsolidated.txt`

**Recommendations:**
- Add type hints throughout
- Create API reference documentation
- Document coordinate system conventions
- Add algorithm complexity notes

---

## Conclusion

This project has successfully implemented all core components needed for autonomous robot navigation in CoppeliaSim. The system demonstrates sophisticated sensor processing, intelligent mapping, and optimal pathfinding capabilities. 

**Key Achievements:**
- Robust vision and LiDAR processing pipelines
- Flexible terrain mapping with multiple traversability costs
- Efficient A* pathfinding with terrain awareness
- Complete coordinate transformation system

**Path Forward:**
The remaining work is primarily integration and orchestration. All "interface tools" are functional and ready for the next phase: connecting these components into a fully autonomous multi-robot navigation system with real-time map updates and dynamic replanning.

**Development Priority:**
Focus on consolidating the codebase, implementing the control loop, and integrating multi-robot coordination before adding new features.

---

**Report Generated:** November 28, 2025  
**Version:** 8 (Unconsolidated)  
**Status:** Ready for Integration Phase
