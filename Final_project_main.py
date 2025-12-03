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
    
    # 4. Get robot position and convert to map coordinates
    print("\n--- Robot Position ---")
    robot = sim.getObjectHandle("/Robot_0")
    robot_pos = sim.getObjectPosition(robot, sim.handle_world)
    robot_map = Func.Convert_world_to_map(robot_pos[0], robot_pos[1], R, Resolution)
    print(f"Robot (world): {robot_pos}")
    print(f"Robot (map): {robot_map}")
    
    # Get sensor handles once before loop
    liDar = sim.getObjectHandle("/Robot_0/fastHokuyo_0")
    visionSensorHandle = sim.getObjectHandle("/Robot_0/visionSensor")
    
    # ========================================
    # CONTINUOUS SCANNING AND PATH PLANNING LOOP
    # ========================================
    
    print("\n===========================================")
    print("Starting continuous scanning loop...")
    print("Scanning every 15 seconds")
    print("===========================================")
    
    scan_interval = 5  # seconds
    scan_count = 0
    
    while True:
        scan_count += 1
        print(f"\n\n{'='*50}")
        print(f"SCAN #{scan_count} - Time: {time.time():.2f}")
        print(f"{'='*50}")
        
        # Update robot position
        robot_pos = sim.getObjectPosition(robot, sim.handle_world)
        robot_map = Func.Convert_world_to_map(robot_pos[0], robot_pos[1], R, Resolution)
        print(f"Robot position: World {robot_pos[:2]}, Map {robot_map}")
        
        # 5. Scan terrain using LiDAR
        print("\n--- Scanning Terrain with LiDAR ---")
        segmented_points = Func.process_Lidar_depth(sim, '/Robot_0/fastHokuyo_0', 0.2)
        print(f"Number of segments detected: {len(segmented_points)}")
        
        # Get LiDAR sensor matrix for coordinate transformation
        sensor_matrix = sim.getObjectMatrix(liDar, sim.handle_world)
        
        # 6. Process LiDAR segments - find centroids in sensor frame
        centroids_sensor = []
        
        # For each LiDAR segment, compute the true centroid
        for seg in segmented_points:
            if seg.shape[0] == 0:
                continue
            # Mean of all points in the segment (x,y,z)
            centroid = np.mean(seg, axis=0)
            centroids_sensor.append(centroid)
        
        print(f"LiDAR segments: {len(segmented_points)}, Centroids found: {len(centroids_sensor)}")
                
        # 7. Transform centroids to world coordinates
        centroids_world = [Func.transform_point(sensor_matrix, c) for c in centroids_sensor]
        print(f"Detected {len(centroids_world)} objects/obstacles")
        
        # Display detected obstacle positions
        for idx, centroid in enumerate(centroids_world):
            centroid_map = Func.Convert_world_to_map(centroid[0], centroid[1], R, Resolution)
            print(f"  Obstacle {idx}: World {centroid[:2]}, Map {centroid_map}")
        
        # 8. Vision sensor setup and processing
        print("\n--- Processing Vision Sensor ---")
        visionSensor_matrix = sim.getObjectMatrix(visionSensorHandle, sim.handle_world)
        
        # Get RGB image and depth map
        imgL, res = Func.process_vision_Sensor_RBG(sim, "Robot_0/visionSensor")
        depth_map, _ = Func.process_vision_sensor_depth(sim, "Robot_0/visionSensor")
        
        print(f"Vision sensor resolution: {res}")
        
        # 9. Detect terrain types by color
        print("\n--- Detecting Terrain Types ---")
        
        # Set camera parameters
        fov_deg = 60
        width_px = res[0]
        height_px = res[1]
        f = fov_deg  # field of view
        
        # Detect different terrain colors
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
            print(f"Red terrain detected at world: {red_world_coord}")
        
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
            print(f"Green terrain detected at world: {green_world_coord}")
        
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
            print(f"Blue terrain detected at world: {blue_world_coord}")
        
        # 10. Update the map with detected terrains
        print("\n--- Updating Map with Detected Terrains ---")
        terrain_array = []
        
        for color, world_coord, terrain_type in terrain_detections:
            # Create terrain object with estimated width (adjust as needed)
            terrain_width = 1.0  # You can adjust this based on your scene
            terrain_obj = Func.terrain(
                width=terrain_width,
                Coordinate=[world_coord[0], world_coord[1]],
                terrain=terrain_type,
                resolution=Resolution
            )
            terrain_array.append(terrain_obj)
            
            terrain_map = Func.Convert_world_to_map(world_coord[0], world_coord[1], R, Resolution)
            print(f"  {color.capitalize()} terrain ({terrain_type.name}): World {world_coord[:2]}, Map {terrain_map}")
        
        # Update the map with all detected terrains
        if terrain_array:
            Func.Update_map(worldmap, terrain_array, Resolution)
            print(f"Map updated with {len(terrain_array)} terrain objects")
        
        # Debug: check what's in the grid
        print("\n--- Map Status ---")
        obstacle_count = 0
        terrain_count = 0
        for i in range(Resolution):
            for j in range(Resolution):
                if worldmap[i, j].isObstacle():
                    obstacle_count += 1
                elif not worldmap[i, j].isFloor():
                    terrain_count += 1
        
        print(f"Obstacles in map: {obstacle_count}")
        print(f"Special terrain cells: {terrain_count}")
        print(f"Floor cells: {Resolution*Resolution - obstacle_count - terrain_count}")
        
        # 11. Run A* pathfinding
        print("\n--- Running A* Pathfinding ---")
        print(f"Finding path from robot to goal...")
        
        group_path = Func.astar(worldmap, robot_pos, goal_world, R, Resolution)
        
        if group_path is not None:
            print(f"Path found! Length: {len(group_path)} waypoints")
            print("Path (map coordinates):")
            for idx, waypoint in enumerate(group_path[:5]):
                print(f"  {idx}: {waypoint}")
            if len(group_path) > 5:
                print(f"  ... ({len(group_path) - 5} more waypoints)")
            
            # 12. Convert path to world coordinates and send to robot
            print("\n--- Sending Path to Robot ---")
            world_path = []
            z_height = robot_pos[2]  # Keep the same z height as the robot
            
            # Convert each map coordinate to world coordinate
            for k in range(1, len(group_path)):
                i = group_path[k][0]
                j = group_path[k][1]
                x, y = Func.Convert_map_to_world(i, j, R, Resolution)
                world_path.append((x, y))
            
            # Replace the last waypoint with the exact goal position
            if len(world_path) > 0:
                world_path[-1] = (goal_world[0], goal_world[1])
            else:
                world_path.append((goal_world[0], goal_world[1]))
            
            print(f"World path waypoints: {len(world_path)}")
            print("Path preview (first 5 waypoints):")
            for idx, waypoint in enumerate(world_path[:5]):
                print(f"  {idx}: {waypoint}")
            
            # Send path to robot
            scriptHandle = sim.getScript(sim.scripttype_childscript, '/Robot_0')
            sim.callScriptFunction('setPath', scriptHandle, world_path)
            print("Path sent to robot successfully!")
        else:
            print("ERROR: No path found!")
        
        print(f"\n--- Scan #{scan_count} Complete ---")
        print(f"Waiting {scan_interval} seconds until next scan...")
        
        # Wait for next scan
        time.sleep(scan_interval)