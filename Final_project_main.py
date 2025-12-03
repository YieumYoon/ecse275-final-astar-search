# -*- coding: utf-8 -*-
"""
Created on Thu Nov 13 20:21:34 2025

@author: halas
"""


import coppeliasim_zmqremoteapi_client as zmq
import FP_funcs as Func
import math
import numpy as np
import time
import matplotlib.pyplot as plt
import threading

# Global lock for thread-safe map access
map_lock = threading.Lock()


def robot_control_thread(robot_name, robot_info, worldmap, goal_handle, R, Resolution, scan_interval):
    """
    Thread function for controlling a single robot.
    Each robot runs its own scanning, detection, and pathfinding loop.
    
    Args:
        robot_name: Name of the robot (e.g., "/Robot_0")
        robot_info: Dictionary containing robot handles
        worldmap: Shared map object (thread-safe with lock)
        goal_handle: Goal object handle
        R: Cell size
        Resolution: Map resolution
        scan_interval: Time between scans
    """
    # Create a separate ZMQ client for this thread (ZMQ is not thread-safe)
    print(f"[{robot_name}] Creating dedicated ZMQ connection...")
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    print(f"[{robot_name}] ZMQ connection established!")
    
    scan_count = 0
    
    while True:
        scan_count += 1
        print(f"\n[{robot_name}] Scan #{scan_count} - Time: {time.time():.2f}")
        
        # Update robot position
        robot_pos = sim.getObjectPosition(robot_info["handle"], sim.handle_world)
        robot_map = Func.Convert_world_to_map(robot_pos[0], robot_pos[1], R, Resolution)
        
        # Update goal position
        goal_world = sim.getObjectPosition(goal_handle, sim.handle_world)
        
        
        print(f"[{robot_name}] Position: World {robot_pos[:2]}, Map {robot_map}")
        
        # 1. Scan terrain using LiDAR
        print(f"[{robot_name}] Scanning with LiDAR...")
        segmented_points = Func.process_Lidar_depth(sim, f'{robot_name}/fastHokuyo_0', 0.2)
        
        # Get LiDAR sensor matrix for coordinate transformation
        sensor_matrix = sim.getObjectMatrix(robot_info["lidar"], sim.handle_world)
        
        # 2. Process LiDAR segments - find centroids
        centroids_sensor = []
        for seg in segmented_points:
            if seg.shape[0] == 0:
                continue
            seg_mid = len(seg)//2
            centroid = seg[seg_mid]
            centroids_sensor.append(centroid)
        
        # 3. Transform centroids to world coordinates
        centroids_world = [Func.transform_point(sensor_matrix, c) for c in centroids_sensor]
        print(f"[{robot_name}] Detected {len(centroids_world)} obstacles")
        
        # 4. Vision sensor processing
        print(f"[{robot_name}] Processing vision sensor...")
        visionSensor_matrix = sim.getObjectMatrix(robot_info["vision"], sim.handle_world)
        
        # Get RGB image and depth map
        imgL, res = Func.process_vision_Sensor_RBG(sim, f"{robot_name[1:]}/visionSensor")
        depth_map, _ = Func.process_vision_sensor_depth(sim, f"{robot_name[1:]}/visionSensor")
        
        # 5. Detect terrain types by color
        fov_deg = 60
        f = fov_deg
        terrain_detections = []
        
        # Red terrain detection
        mask_red = Func.mask_color(imgL, "red")
        if np.any(mask_red):
            cent_red = Func.centroid_from_mask(mask_red).astype(int)
            u, v = cent_red[0], cent_red[1]
            depth_Z = depth_map[v, u]
            pixel_uv = (u, v)
            camera_frame_coord = Func.compute_pos_from_pix(pixel_uv, res, f, depth_Z)
            red_world_coord = Func.transform_point(visionSensor_matrix, camera_frame_coord)
            terrain_detections.append(("red", red_world_coord, Func.TerrainType.obstacle))
        
        # Green terrain detection
        mask_green = Func.mask_color(imgL, "green")
        if np.any(mask_green):
            cent_green = Func.centroid_from_mask(mask_green).astype(int)
            u, v = cent_green[0], cent_green[1]
            depth_Z = depth_map[v, u]
            pixel_uv = (u, v)
            camera_frame_coord = Func.compute_pos_from_pix(pixel_uv, res, f, depth_Z)
            green_world_coord = Func.transform_point(visionSensor_matrix, camera_frame_coord)
            terrain_detections.append(("green", green_world_coord, Func.TerrainType.grass))
        
        # Blue terrain detection
        mask_blue = Func.mask_color(imgL, "blue")
        if np.any(mask_blue):
            cent_blue = Func.centroid_from_mask(mask_blue).astype(int)
            u, v = cent_blue[0], cent_blue[1]
            depth_Z = depth_map[v, u]
            pixel_uv = (u, v)
            camera_frame_coord = Func.compute_pos_from_pix(pixel_uv, res, f, depth_Z)
            blue_world_coord = Func.transform_point(visionSensor_matrix, camera_frame_coord)
            terrain_detections.append(("blue", blue_world_coord, Func.TerrainType.water))
        
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
                Func.Update_map(worldmap, terrain_array, Resolution)
                print(f"[{robot_name}] Map updated with {len(terrain_array)} terrain objects")
            print(f"[{robot_name}] Map lock released")
        
        # 8. Run A* pathfinding with thread-safe map access
        print(f"[{robot_name}] Running A* pathfinding...")
        print(f"[{robot_name}] Acquiring map lock for pathfinding...")
        with map_lock:
            group_path = Func.astar(worldmap, robot_pos, goal_world, R, Resolution)
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
            sim.callScriptFunction('setPath', robot_info["script"], world_path)
            print(f"[{robot_name}] Path sent to robot!")
        else:
            print(f"[{robot_name}] ERROR: No path found!")
        
        print(f"[{robot_name}] Scan complete. Waiting {scan_interval}s...")
        time.sleep(scan_interval)


if __name__ == "__main__" :
    # ========================================
    # SCENE SETUP
    # ========================================
    
    # 1. Establish the ZMQ connection
    print("Establishing connection to CoppeliaSim...")
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    print("Connection established!")
    
    # 2. Initialize the map with resolution
    Resolution = 100  # the resolution of the map
    R = 10/Resolution  # width of each discrete square
    print(f"\nInitializing map with resolution: {Resolution}x{Resolution}")
    print(f"Cell size (R): {R}")
    worldmap = Func.createMap_withResolution(Resolution)
    print("Map initialized!")
    
    # 3. Get goal position and convert to map coordinates
    print("\n--- Goal Position ---")
    goal = sim.getObjectHandle("/goal_point")
    goal_world = sim.getObjectPosition(goal, sim.handle_world)
    goal_map = Func.Convert_world_to_map(goal_world[0], goal_world[1], R, Resolution)
    print(f"Goal (world): {goal_world}")
    print(f"Goal (map): {goal_map}")
    
    # 4. Initialize robots (supports 1, 2, or more robots)
    robot_names = ["/Robot_0", "/Robot_1"]   # add more if needed
    
    robots = {}
    
    print("\n--- Initializing Robots ---")
    for name in robot_names:
        print(f"Setting up {name}")
    
        robot_handle = sim.getObjectHandle(name)
        lidar_handle = sim.getObjectHandle(f"{name}/fastHokuyo_0")
        vision_handle = sim.getObjectHandle(f"{name}/visionSensor")
        script_handle = sim.getScript(sim.scripttype_childscript, name)
    
        robot_pos = sim.getObjectPosition(robot_handle, sim.handle_world)
        robot_map = Func.Convert_world_to_map(robot_pos[0], robot_pos[1], R, Resolution)
    
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
    
    scan_interval = 1  # seconds between scans for each robot
    
    # Create and start a thread for each robot
    threads = []
    for robot_name in robot_names:
        thread = threading.Thread(
            target=robot_control_thread,
            args=(robot_name, robots[robot_name], worldmap, goal, R, Resolution, scan_interval),
            daemon=True
        )
        thread.start()
        threads.append(thread)
        print(f"Thread started for {robot_name}")
        time.sleep(0.5)  # Small delay to stagger thread startup
    
    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")