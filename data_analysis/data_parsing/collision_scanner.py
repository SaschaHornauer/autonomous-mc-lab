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


def get_fov_one_camera(xy, heading, fov_angle, distance):
    '''
    returns the field of view as triangle, based on a sequence of
    coordinates. It will always regard the last coordinates in that list
    '''    
    limit_left = heading - np.deg2rad(fov_angle / 2.0)
    limit_right = heading + np.deg2rad(fov_angle / 2.0)
    
    a = xy[len(xy)-1]  # Point a is the latest known position
    b = cv2.polarToCart(distance, limit_left)  # Point b is distance in front and half of fov to the left
    c = cv2.polarToCart(distance, limit_right)  # Point c is distance in front and half of fov to the right
    
    # Needed extraction of the values from the polarToCart method
    # and adding of a as the origin 
    b = b[0][0][0]+a[0],b[1][0][0]+a[1]
    c = c[0][0][0]+a[0],c[1][0][0]+a[1]
    
    return Triangle(to_point(a), to_point(b), to_point(c))


def get_encounter_xy(timestamps_n_trajectories):
    
    
    
    
    
    pass


if __name__ == '__main__':
    
    trajectories_path = sys.argv[1]
    trajectories_dict = pickle.load(open(trajectories_path, "rb"))

    #print trajectories_dict['Mr_Black']['/home/picard/2ndDisk/carData/run_28apr/direct_rewrite_test_28Apr17_17h23m15s_Mr_Black']['self_trajectory']['left']['x']

    show_only_encounters = False
    plt.figure('top',figsize=(6,6))
    
    # Timestamps from the last car are taken. In the future it would be good to check
    # if the timestamps differ for different cars
    
    encounters_cars_timesteps_xy = {}

    for carname in trajectories_dict:
        for run_name in trajectories_dict[carname]:
            print trajectories_dict[carname][run_name]['self_trajectory']['left']['x']
        
            # Right now there is only one runname and this loop is used to get its name
            
            left_x = trajectories_dict[carname][run_name]['self_trajectory']['left']['x']
            left_y = trajectories_dict[carname][run_name]['self_trajectory']['left']['y']
            
            right_x = trajectories_dict[carname][run_name]['self_trajectory']['right']['x']
            right_y = trajectories_dict[carname][run_name]['self_trajectory']['right']['y']
            
            mid_xy = (((right_x+left_x)/2.),((left_y + right_y)/2.))
            timestamps = trajectories_dict[carname][run_name]['self_trajectory']['ts']
            
            own_timestamps_n_trajectories = dict(zip(timestamps,zip(mid_xy[0],mid_xy[1])))
    
            encounters_cars_timesteps_xy[carname][run_name] = get_encounter_xy(own_timestamps_n_trajectories, trajectories_dict)

    
