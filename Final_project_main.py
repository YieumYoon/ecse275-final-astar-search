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
    # Establish the ZMQ connection
    SENSOR_NAME = 'visionSensor' 
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
        
    goal = sim.getObjectHandle("/goal_point")
    goal_world = sim.getObjectPosition(goal,sim.handle_world) # query the goal xyz coords in the world frame
    
    robot = sim.getObjectHandle("/Robot_0")
    robot_pos = sim.getObjectPosition(robot,sim.handle_world)
    
    Resolution = 100 #the resolution of the map
    R = 10/Resolution
    worldmap = Func.createMap_withResolution(Resolution)

    liDar = sim.getObjectHandle("/Robot_0/fastHokuyo_0")
    segmented_points = Func.process_Lidar_depth(sim,'/Robot_0/fastHokuyo_0', 0.2)
    #print(sim.callScriptFunction('getMeasuredData',lidarScriptHandle))
   
    sensor_matrix = sim.getObjectMatrix(liDar, sim.handle_world)
    # objects: list of arrays, each (M_i, 3)
    centroids_sensor = []
    
    for segment in segmented_points:
        centroid = np.mean(segment, axis=0)  # [x_mean, y_mean, z_mean] in sensor frame
        centroids_sensor.append(centroid)
        
    centroids_world = [Func.transform_point(sensor_matrix, c) for c in centroids_sensor]



    visionSensorHandle = sim.getObjectHandle("/Robot_0/visionSensor")
    visionSensor_matrix = sim.getObjectMatrix(visionSensorHandle, sim.handle_world)
    
    imgL , res = Func.process_vision_Sensor_RBG(sim, "Robot_0/visionSensor")
    depth_map,_ = Func.process_vision_sensor_depth(sim, "Robot_0/visionSensor")
    
    plt.imshow(imgL)       # display the image
    plt.axis('off')       # turn off axis numbers
    plt.show() 
    
    mask = Func.mask_color(imgL, "red")
    
    # Set parameters
    fov_deg = 60
    width_px = 256
    height_px = 256
    
    # Focal length in pixels
    f = fov_deg#(width_px/2) / np.tan(fov_deg / 2)
    
    
    cent = Func.centroid_from_mask(mask).astype(int)
    
    u = cent[0]   # column
    v = cent[1]   # row
    
    depth_Z = depth_map[v, u]
    pixel_uv = (u, v)
    
    camera_frame_coord = Func.compute_pos_from_pix(pixel_uv, res, f, depth_Z)
    
    camera_word_coordinate = Func.transform_point(visionSensor_matrix, camera_frame_coord)
    
    print(camera_word_coordinate)
    
    
    
    
    
    
    


        