# -*- coding: utf-8 -*-
"""
Created on Thu Nov 13 20:21:34 2025

@author: halas
"""


from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from config import default_config as cfg
import FP_funcs as Func
import math
import numpy as np
import time
import matplotlib.pyplot as plt
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


def robot_control_thread(robot_name, robot_info, worldmap, R, Resolution, scan_interval):
    """
    Thread function for controlling a single robot.
    Each robot runs its own scanning, detection, and pathfinding loop.

    Args:
        robot_name: Name of the robot (e.g., "/Robot_0")
        robot_info: Dictionary containing robot handles
        worldmap: Shared map object (thread-safe with lock)
        R: Cell size
        Resolution: Map resolution
        scan_interval: Time between scans
    """
    # Create a separate ZMQ client for this thread (ZMQ is not thread-safe)
    print(f"[{robot_name}] Creating dedicated ZMQ connection...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    print(f"[{robot_name}] ZMQ connection established!")

    # Get fresh handles using this thread's sim object
    print(f"[{robot_name}] Getting object handles...")
    robot_handle = sim.getObjectHandle(robot_name)
    lidar_handle = sim.getObjectHandle(f"{robot_name}/fastHokuyo_0")
    vision_handle = sim.getObjectHandle(f"{robot_name}/visionSensor")
    script_handle = sim.getScript(sim.scripttype_childscript, robot_name)
    print(f"[{robot_name}] Handles acquired successfully!")

    # Track current goal assignment for this robot
    current_goal_index = None

    scan_count = 0

    while True:
        scan_count += 1
        print(f"\n[{robot_name}] Scan #{scan_count} - Time: {time.time():.2f}")

        # Update robot position
        robot_pos = sim.getObjectPosition(robot_handle, sim.handle_world)
        robot_map = Func.Convert_world_to_map(
            robot_pos[0], robot_pos[1], R, Resolution)

        print(f"[{robot_name}] Position: World {robot_pos[:2]}, Map {robot_map}")

        # ===========================================
        # GOAL COMPLETION CHECK - Poll Lua Status
        # ===========================================

        # Check if Lua controller says goal is reached
        if current_goal_index is not None:
            goal_reached = sim.callScriptFunction(
                'getGoalStatus', script_handle)

            if goal_reached:
                print(
                    f"[{robot_name}] *** LUA REPORTS GOAL {current_goal_index} REACHED! ***")

                with goals_lock:
                    goal_world = goals_data['positions'][current_goal_index]
                    print(f"[{robot_name}] Goal was at: {goal_world[:2]}")
                    print(f"[{robot_name}] Robot is at: {robot_pos[:2]}")

                    # Mark goal as completed
                    goals_data['completed'][current_goal_index] = True
                    goals_data['assigned_to'][current_goal_index] = None
                    current_goal_index = None
                    print(
                        f"[{robot_name}] Goal marked as completed, will find next goal...")

        # ===========================================
        # GOAL ASSIGNMENT LOGIC
        # ===========================================

        # Assign new goal if needed
        if current_goal_index is None:
            print(
                f"[{robot_name}] No current goal, finding nearest uncompleted goal...")
            with goals_lock:
                # Find nearest uncompleted AND unassigned goal
                min_distance = float('inf')
                nearest_goal_idx = None

                for idx in range(len(goals_data['positions'])):
                    # Skip if completed OR already assigned to another robot
                    if goals_data['completed'][idx]:
                        print(f"[{robot_name}] Goal {idx}: COMPLETED")
                        continue
                    if goals_data['assigned_to'][idx] is not None and goals_data['assigned_to'][idx] != robot_name:
                        print(
                            f"[{robot_name}] Goal {idx}: Already assigned to {goals_data['assigned_to'][idx]}")
                        continue

                    goal_pos = goals_data['positions'][idx]
                    dist = math.sqrt(
                        (robot_pos[0] - goal_pos[0])**2 +
                        (robot_pos[1] - goal_pos[1])**2
                    )
                    print(
                        f"[{robot_name}] Goal {idx}: distance = {dist:.3f}m, available")
                    if dist < min_distance:
                        min_distance = dist
                        nearest_goal_idx = idx

                if nearest_goal_idx is not None:
                    current_goal_index = nearest_goal_idx
                    goals_data['assigned_to'][current_goal_index] = robot_name

                    # Mark goal cell as floor (not obstacle) to ensure pathfinding works
                    goal_pos = goals_data['positions'][current_goal_index]
                    goal_map_coord = Func.Convert_world_to_map(
                        goal_pos[0], goal_pos[1], R, Resolution)
                    gi, gj = goal_map_coord
                    if 0 <= gi < Resolution and 0 <= gj < Resolution:
                        with map_lock:
                            worldmap[gi][gj].setTerrainNum(
                                Func.TerrainType.floor)

                    print(
                        f"[{robot_name}] *** ASSIGNED TO GOAL {current_goal_index} at {goals_data['positions'][current_goal_index][:2]} ***")
                else:
                    print(
                        f"[{robot_name}] *** ALL GOALS COMPLETED! No more goals available! ***")
                    time.sleep(scan_interval)
                    continue

        # Get current goal position
        with goals_lock:
            goal_world = goals_data['positions'][current_goal_index]
            goal_map = Func.Convert_world_to_map(
                goal_world[0], goal_world[1], R, Resolution)

            # IMPORTANT: Always ensure goal cell is floor (not obstacle) on every scan
            gi, gj = goal_map
            if 0 <= gi < Resolution and 0 <= gj < Resolution:
                with map_lock:
                    worldmap[gi][gj].setTerrainNum(Func.TerrainType.floor)

        print(
            f"[{robot_name}] Current Goal {current_goal_index}: World {goal_world[:2]}, Map {goal_map}")

        # 1. Scan terrain using LiDAR (threshold from config)
        print(f"[{robot_name}] Scanning with LiDAR...")
        segmented_points = Func.process_Lidar_depth(
            sim, f'{robot_name}/fastHokuyo_0', cfg.sensor.lidar_threshold)

        # Get LiDAR sensor matrix for coordinate transformation
        sensor_matrix = sim.getObjectMatrix(lidar_handle, sim.handle_world)

        # 2. Process LiDAR segments - find centroids
        centroids_sensor = []
        for seg in segmented_points:
            if seg.shape[0] == 0:
                continue
            seg_mid = len(seg)//2
            centroid = seg[seg_mid]
            centroids_sensor.append(centroid)

        # 3. Transform centroids to world coordinates
        centroids_world = [Func.transform_point(
            sensor_matrix, c) for c in centroids_sensor]
        print(f"[{robot_name}] Detected {len(centroids_world)} obstacles")

        # 4. Vision sensor processing
        print(f"[{robot_name}] Processing vision sensor...")
        visionSensor_matrix = sim.getObjectMatrix(
            vision_handle, sim.handle_world)

        # Get RGB image and depth map
        imgL, res = Func.process_vision_Sensor_RBG(
            sim, f"{robot_name[1:]}/visionSensor")
        depth_map, _ = Func.process_vision_sensor_depth(
            sim, f"{robot_name[1:]}/visionSensor")

        # 5. Detect terrain types by color (FOV from config)
        fov_deg = cfg.sensor.vision_fov_deg
        f = fov_deg
        terrain_detections = []

        # Red terrain detection
        mask_red = Func.mask_color(imgL, "red")
        if np.any(mask_red):
            cent_red = Func.centroid_from_mask(mask_red).astype(int)
            u, v = cent_red[0], cent_red[1]
            depth_Z = depth_map[v, u]
            pixel_uv = (u, v)
            camera_frame_coord = Func.compute_pos_from_pix(
                pixel_uv, res, f, depth_Z)
            red_world_coord = Func.transform_point(
                visionSensor_matrix, camera_frame_coord)
            terrain_detections.append(
                ("red", red_world_coord, Func.TerrainType.obstacle))

        # Green terrain detection
        mask_green = Func.mask_color(imgL, "green")
        if np.any(mask_green):
            cent_green = Func.centroid_from_mask(mask_green).astype(int)
            u, v = cent_green[0], cent_green[1]
            depth_Z = depth_map[v, u]
            pixel_uv = (u, v)
            camera_frame_coord = Func.compute_pos_from_pix(
                pixel_uv, res, f, depth_Z)
            green_world_coord = Func.transform_point(
                visionSensor_matrix, camera_frame_coord)
            terrain_detections.append(
                ("green", green_world_coord, Func.TerrainType.grass))

        # Blue terrain detection
        mask_blue = Func.mask_color(imgL, "blue")
        if np.any(mask_blue):
            cent_blue = Func.centroid_from_mask(mask_blue).astype(int)
            u, v = cent_blue[0], cent_blue[1]
            depth_Z = depth_map[v, u]
            pixel_uv = (u, v)
            camera_frame_coord = Func.compute_pos_from_pix(
                pixel_uv, res, f, depth_Z)
            blue_world_coord = Func.transform_point(
                visionSensor_matrix, camera_frame_coord)
            terrain_detections.append(
                ("blue", blue_world_coord, Func.TerrainType.water))

        # 6. Build terrain array
        terrain_array = []

        for color, world_coord, terrain_type in terrain_detections:
            terrain_width = 0.5
            terrain_obj = Func.terrain(
                width=terrain_width,
                Coordinate=[world_coord[0], world_coord[1]],
                terrain=terrain_type,
                resolution=Resolution
            )
            terrain_array.append(terrain_obj)

        for obstacle_coord in centroids_world:
            terrain_width = 1.0
            terrain_obj = Func.terrain(
                width=terrain_width,
                Coordinate=[obstacle_coord[0], obstacle_coord[1]],
                terrain=Func.TerrainType.obstacle,
                resolution=Resolution
            )
            terrain_array.append(terrain_obj)

        # 7. Update the shared map with thread-safe lock
        if terrain_array:
            print(f"[{robot_name}] Acquiring map lock to update...")
            with map_lock:
                # Filter out terrain objects that are out of bounds
                valid_terrain = []
                for terrain_obj in terrain_array:
                    coord = terrain_obj.getCoordinateArray()
                    map_coord = Func.Convert_world_to_map(
                        coord[0], coord[1], R, Resolution)
                    i, j = map_coord

                    # Check if within bounds
                    if 0 <= i < Resolution and 0 <= j < Resolution:
                        valid_terrain.append(terrain_obj)
                    else:
                        print(
                            f"[{robot_name}] Warning: Terrain at ({i}, {j}) is out of bounds, skipping")

                if valid_terrain:
                    Func.Update_map(worldmap, valid_terrain,
                                    Resolution, cfg.map.world_size)
                    print(
                        f"[{robot_name}] Map updated with {len(valid_terrain)} terrain objects")

                    # After updating map, re-protect all active goal cells
                    with goals_lock:
                        for idx in range(len(goals_data['positions'])):
                            if not goals_data['completed'][idx]:
                                g_pos = goals_data['positions'][idx]
                                g_map = Func.Convert_world_to_map(
                                    g_pos[0], g_pos[1], R, Resolution)
                                gi, gj = g_map
                                if 0 <= gi < Resolution and 0 <= gj < Resolution:
                                    worldmap[gi][gj].setTerrainNum(
                                        Func.TerrainType.floor)
                else:
                    print(f"[{robot_name}] No valid terrain to update")
            print(f"[{robot_name}] Map lock released")

        # 8. Run A* pathfinding with thread-safe map access
        print(f"[{robot_name}] Running A* pathfinding...")
        print(f"[{robot_name}] Acquiring map lock for pathfinding...")
        with map_lock:
            group_path = Func.astar(
                worldmap, robot_pos, goal_world, R, Resolution)
        print(f"[{robot_name}] Map lock released after pathfinding")

        if group_path is not None:
            print(f"[{robot_name}] Path found! Length: {len(group_path)} waypoints")

            # 9. Convert path to world coordinates
            world_path = []
            z_height = robot_pos[2]

            for k in range(1, len(group_path)):
                i = group_path[k][0]
                j = group_path[k][1]
                x, y = Func.Convert_map_to_world(i, j, R, Resolution)
                world_path.append((x, y))

            # Replace last waypoint with exact goal
            if len(world_path) > 0:
                world_path[-1] = (goal_world[0], goal_world[1])
            else:
                world_path.append((goal_world[0], goal_world[1]))

            # 10. Send path to robot
            sim.callScriptFunction('setPath', script_handle, world_path)
            print(f"[{robot_name}] Path sent to robot!")
            print(f"[{robot_name}] First 3 waypoints: {world_path[:3]}")
        else:
            # Error recovery: No path found - wait and retry on next scan
            print(
                f"[{robot_name}] ERROR: No path found to goal {current_goal_index}!")
            print(f"[{robot_name}] Will retry pathfinding on next scan cycle...")
            # Robot continues moving with previous path (if any) or stays idle
            # Next scan will update the map and attempt pathfinding again

        print(f"[{robot_name}] Scan complete. Waiting {scan_interval}s...")
        time.sleep(scan_interval)


if __name__ == "__main__":
    # ========================================
    # SCENE SETUP
    # ========================================

    # 1. Establish the ZMQ connection
    print("Establishing connection to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    print("Connection established!")

    # 2. Initialize the map with resolution (from config)
    Resolution = cfg.map.resolution  # the resolution of the map
    R = cfg.map.cell_size  # width of each discrete square
    print(f"\nInitializing map with resolution: {Resolution}x{Resolution}")
    print(f"Cell size (R): {R}")
    print(f"World size: {cfg.map.world_size}m")
    worldmap = Func.createMap_withResolution(Resolution, cfg.map.world_size)
    print("Map initialized!")

    # 3. Get goal positions and initialize tracking (from config)
    print("\n--- Initializing Goals ---")
    goal_names = cfg.goals.names

    for goal_name in goal_names:
        try:
            goal_handle = sim.getObjectHandle(goal_name)
            goal_pos = sim.getObjectPosition(goal_handle, sim.handle_world)
            goals_data['positions'].append(goal_pos)
            goals_data['completed'].append(False)
            goals_data['assigned_to'].append(None)
            print(f"Goal '{goal_name}' at position: {goal_pos[:2]}")
        except Exception as e:
            print(f"Warning: Could not find goal '{goal_name}': {e}")

    print(f"Total goals initialized: {len(goals_data['positions'])}")

    if len(goals_data['positions']) == 0:
        print("ERROR: No goals found! Exiting...")
        exit(1)

    # 4. Initialize robots (from config)
    robot_names = cfg.robots.names

    robots = {}

    print("\n--- Initializing Robots ---")
    for name in robot_names:
        print(f"Setting up {name}")

        robot_handle = sim.getObjectHandle(name)
        lidar_handle = sim.getObjectHandle(f"{name}/fastHokuyo_0")
        vision_handle = sim.getObjectHandle(f"{name}/visionSensor")
        script_handle = sim.getScript(sim.scripttype_childscript, name)

        robot_pos = sim.getObjectPosition(robot_handle, sim.handle_world)
        robot_map = Func.Convert_world_to_map(
            robot_pos[0], robot_pos[1], R, Resolution)

        print(f"{name} position world: {robot_pos}")
        print(f"{name} position map:   {robot_map}")

        robots[name] = {
            "handle": robot_handle,
            "lidar": lidar_handle,
            "vision": vision_handle,
            "script": script_handle,
            "pos": robot_pos
        }

    # ========================================
    # START MULTI-THREADED ROBOT CONTROL
    # ========================================

    print("\n===========================================")
    print("Starting multi-threaded robot control...")
    print(f"Launching {len(robots)} robot threads")
    print("===========================================")

    # seconds between scans for each robot
    scan_interval = cfg.navigation.scan_interval

    # Create and start a thread for each robot
    threads = []
    for robot_name in robot_names:
        thread = threading.Thread(
            target=robot_control_thread,
            args=(robot_name, robots[robot_name],
                  worldmap, R, Resolution, scan_interval),
            daemon=True,
            name=f"Thread-{robot_name}"  # Name the thread for easier debugging
        )
        thread.start()
        threads.append(thread)
        print(f"Thread started for {robot_name}")
        time.sleep(1.0)  # Longer delay to ensure clean startup

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
