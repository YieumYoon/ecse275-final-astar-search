# -*- coding: utf-8 -*-
"""
Main Control Script for Robot Navigation System.
"""

import coppeliasim_zmqremoteapi_client as zmq
import FP_funcs as Func
import numpy as np
import matplotlib.pyplot as plt
import config
import sys

def main():
    # Establish the ZMQ connection
    print("Connecting to CoppeliaSim...")
    try:
        client = zmq.RemoteAPIClient()
        sim = client.getObject('sim')
    except Exception as e:
        print(f"Failed to connect to CoppeliaSim: {e}")
        sys.exit(1)
        
    print("Connected!")

    # Get handles
    try:
        goal = sim.getObjectHandle(config.GOAL_NAME)
        goal_world = sim.getObjectPosition(goal, sim.handle_world)
        
        robot = sim.getObjectHandle(config.ROBOT_NAME)
        robot_pos = sim.getObjectPosition(robot, sim.handle_world)
        
        lidar = sim.getObjectHandle(config.LIDAR_NAME)
        vision_sensor_handle = sim.getObjectHandle(f"{config.ROBOT_NAME}/{config.VISION_SENSOR_NAME}")
    except Exception as e:
        print(f"Error getting object handles: {e}")
        sys.exit(1)

    # Initialize map
    print("Initializing map...")
    world_map = Func.create_map_with_resolution(config.RESOLUTION)

    # Process LiDAR
    print("Processing LiDAR data...")
    segmented_points = Func.process_lidar_depth(sim, config.LIDAR_NAME, config.LIDAR_THRESHOLD)
    
    sensor_matrix = sim.getObjectMatrix(lidar, sim.handle_world)
    centroids_sensor = []
    
    for segment in segmented_points:
        centroid = np.mean(segment, axis=0)  # [x, y, z] in sensor frame
        centroids_sensor.append(centroid)
        
    centroids_world = [Func.transform_point(sensor_matrix, c) for c in centroids_sensor]
    print(f"Found {len(centroids_world)} obstacles.")

    # Process Vision
    print("Processing vision data...")
    vision_sensor_matrix = sim.getObjectMatrix(vision_sensor_handle, sim.handle_world)
    
    img_l, res = Func.process_vision_sensor_rgb(sim, f"{config.ROBOT_NAME}/{config.VISION_SENSOR_NAME}")
    depth_map, _ = Func.process_vision_sensor_depth(sim, f"{config.ROBOT_NAME}/{config.VISION_SENSOR_NAME}")
    
    plt.imshow(img_l)
    plt.axis('off')
    plt.title("Robot Vision")
    plt.show() 
    
    mask = Func.mask_color(img_l, "red")
    
    cent = Func.centroid_from_mask(mask)
    if cent is not None:
        cent = cent.astype(int)
        u, v = cent[0], cent[1]
        
        depth_z = depth_map[v, u]
        pixel_uv = (u, v)
        
        camera_frame_coord = Func.compute_pos_from_pix(pixel_uv, res, config.VISION_FOV_DEG, depth_z)
        camera_world_coordinate = Func.transform_point(vision_sensor_matrix, camera_frame_coord)
        
        print(f"Detected Red Object at World Coordinates: {camera_world_coordinate}")
    else:
        print("No red object detected.")

if __name__ == "__main__":
    main()