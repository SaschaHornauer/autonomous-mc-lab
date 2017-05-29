'''
Created on May 8, 2017

@author: picard
'''
from trajectory_generator.trajectory_tools import *
import cPickle as pickle
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
from collections import defaultdict
import sys
from timeit import default_timer as timer

distance = 8 # m
fov_angle = 110.0 # deg

def get_fov_triangle(xy, heading, fov_angle, distance):
    '''
    returns the field of view as triangle, based on a sequence of
    coordinates. It will always regard the last coordinates in that list
    '''    
    limit_left = heading - np.deg2rad(fov_angle / 2.0)
    limit_right = heading + np.deg2rad(fov_angle / 2.0)
    
    a = xy  # Point a is the latest known position
    b = cv2.polarToCart(distance, limit_left)  # Point b is distance in front and half of fov to the left
    c = cv2.polarToCart(distance, limit_right)  # Point c is distance in front and half of fov to the right
    
    # Needed extraction of the values from the polarToCart method
    # and adding of a as the origin 
    b = b[0][0][0]+a[0],b[1][0][0]+a[1]
    c = c[0][0][0]+a[0],c[1][0][0]+a[1]
    
    return Triangle(to_point(a), to_point(b), to_point(c))



def get_encounter_xy(own_car_name, own_timestamps_n_trajectories, trajectories_dict):
    
    global distance
    global fov_angle
    
    
    encounter_xy = defaultdict(lambda: defaultdict(dict))
    
    # The whole method worked though very slow so a few 
    # tweaks are made to increase the speed
    
    own_xy_values = []
    timestamps = []
    timestamped_other_trajs = {}
    
    
    # First create a quickly accessible list of own positions
    for (timestamp,coord) in own_timestamps_n_trajectories:
        own_xy_values.append(coord)
        timestamps.append(timestamp)
    
    # There is a bug here
    for car_name in trajectories_dict:
        for run_name in trajectories_dict[car_name]:
            timestamped_other_trajs[car_name] = get_timestamped_trajectories(car_name, run_name, trajectories_dict)
    
    for i in range(len(own_timestamps_n_trajectories)):
        
        smooth_diff = 10
        own_xy = own_timestamps_n_trajectories[i][1]
        
        if smooth_diff+i > len(own_timestamps_n_trajectories):
            return None
        
        heading = get_heading(own_xy_values[i:i+smooth_diff])
        fov_triangle = get_fov_triangle(own_xy, heading, fov_angle, distance)
        
        for car_name in timestamped_other_trajs:
            
            if car_name != own_car_name:
                for (timestamp,other_xy) in timestamped_other_trajs[car_name]:
                    
                    if fov_triangle.isInside(Point(other_xy[0],other_xy[1])):
                        encounter_xy[car_name] = {'timestamp':timestamp,'pos_xy':other_xy}
                        print "At " + str(timestamp)
                else:
                    continue
    

def get_timestamped_trajectories(car_name,run_name,traj_dictionary):
    
    left_x = trajectories_dict[car_name][run_name]['self_trajectory']['left']['x']
    left_y = trajectories_dict[car_name][run_name]['self_trajectory']['left']['y']
    
    right_x = trajectories_dict[car_name][run_name]['self_trajectory']['right']['x']
    right_y = trajectories_dict[car_name][run_name]['self_trajectory']['right']['y']
    
    mid_xy = (((right_x+left_x)/2.),((left_y + right_y)/2.))
    timestamps = trajectories_dict[car_name][run_name]['self_trajectory']['ts']
    
    return zip(timestamps,zip(mid_xy[0],mid_xy[1]))
    
if __name__ == '__main__':
    
    trajectories_path = sys.argv[1]
    trajectories_dict = pickle.load(open(trajectories_path, "rb"))

    #print trajectories_dict['Mr_Black']['/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black']['self_trajectory']['left']['x']

    show_only_encounters = False
    plt.figure('top',figsize=(6,6))
    
    # Timestamps from the last car are taken. In the future it would be good to check
    # if the timestamps differ for different cars
    
    encounters_cars_timesteps_xy = defaultdict(lambda: defaultdict(dict))

    for own_car_name in trajectories_dict:
        for run_name in trajectories_dict[own_car_name]:
        
            # Right now there is only one runname and this loop is used to get its name
                       
            own_timestamps_n_trajectories = get_timestamped_trajectories(own_car_name, run_name, trajectories_dict) 
    
            # Returns a dict which maps carnames to timestamps and positions if those positions
            # are in sight of the own car
            encounters_cars_timesteps_xy[own_car_name][run_name] = get_encounter_xy(own_car_name, own_timestamps_n_trajectories, trajectories_dict)

    
