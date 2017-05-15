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
import sys

def get_close_encounters(own_xys, other_xys, list_of_timestamps):
    '''
    own and other xy coordinates are expected in the following format:
    
    own_data = 'pos_xy':[2]
    other_data = 'other_xy':[2] 
    list_of_timestamps = list of list_of_timestamps 
    '''
    # Get triangle in front of the vehicle ( THIS IS A SIMPLIFICATION ) 
        
    encounter_list = []
    triangle_points = {}

    smooth_over_list_of_timestamps = 3
    own_xys = np.transpose(own_xys)
    other_xys = np.transpose(other_xys)
    
    for t in range(smooth_over_list_of_timestamps,len(list_of_timestamps)):
        
        current_own_xys = own_xys[t-smooth_over_list_of_timestamps:t]
        
        triangle_fov = get_fov_one_camera(current_own_xys,get_heading(current_own_xys),66.,1.5)
                
        point_xy = Point(other_xys[t][0], other_xys[t][1])
            
        if(triangle_fov.isInside(point_xy)):
            encounter_list.append(list_of_timestamps[t])
            triangle_points[str(list_of_timestamps[t])]=triangle_fov
            print(t)
        
    return encounter_list, triangle_points

def get_close_encounters_in_list(own_xys,other_xys,start_timestep, end_timestep):
    # Get triangle in front of the vehicle ( THIS IS A SIMPLIFICATION ) 
        
    encounter_timestep = []
    other_positions = []
    triangle_points = {}
    smooth_over_list_of_timestamps = 3
   
    for t in range(start_timestep+smooth_over_list_of_timestamps,end_timestep):
        
        current_own_xys = own_xys[t-smooth_over_list_of_timestamps:t]
        
        # distance chosen is 1.5 m, field of view of one of the stereo cameras is 66
        triangle_fov = get_fov_one_camera(current_own_xys,get_heading(current_own_xys),66.,1.5)
        
        closest_xy = None

        for other_car_id in range(0,len(other_xys)):
            # For all other car positions at that point in time t
            # This has to be a Point of a certain datastructure to make easy computation
            # of the triangle approach possible
            other_xy = Point(other_xys[other_car_id][t][0], other_xys[other_car_id][t][1])
            
            # Get the closest of that positions
            if closest_xy == None:
                closest_xy = other_xy
            else:
                new_dist = np.hypot(other_xy.x-np.mean([i[0] for i in current_own_xys]),other_xy.y-np.mean([i[1] for i in current_own_xys]))
                old_dist = np.hypot(closest_xy.x-np.mean([i[0] for i in current_own_xys]),closest_xy.y-np.mean([i[1] for i in current_own_xys]))
                if new_dist < old_dist:
                    closest_xy = other_xy
        # If that closest other xy point lies in the FOV add it to the list
        if(triangle_fov.isInside(closest_xy)):
            encounter_timestep.append(t)
            other_positions.append(closest_xy)
            triangle_points[str(t)]=triangle_fov
            
    return encounter_timestep, other_positions, triangle_points

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

if __name__ == '__main__':
#     home = os.path.expanduser("~")
#     pickle_file = pickle.load(open(home + '/kzpy3/teg9/trajectories.pkl', "rb"))
# 
#     t1 = 1493425694.71+5
#     t2 = 1493425899.676476 - 100
#     timestamps = np.arange(t1,t2,1/30.)
#     
#     show_only_encounters = False
#     
#     xy_black = [pickle_file['Mr_Black']['left'][0](timestamps),pickle_file['Mr_Black']['left'][1](timestamps)]
#     xy_blue  = [pickle_file['Mr_Blue']['left'][0](timestamps),pickle_file['Mr_Blue']['left'][1](timestamps)]
#     
#     encounters, triangle_fov = get_close_encounters(xy_black, xy_blue, timestamps)
#     
#     plt.figure('top',figsize=(6,6)) 
# 
#     if show_only_encounters:
#         timestamps = encounters
#     
#     print ("Encounters at " + str(timestamps))
#     color = 'black'
#     for car in ['Mr_Black','Mr_Blue']:
#         for side in ['left','right']:
#             x = pickle_file[car][side][0](timestamps)
#             y = pickle_file[car][side][1](timestamps)
#             plt.plot(x,y,'.',color=color)
#         color = 'blue'
# 
#     # Draw triangles 
#     for triangle_key in triangle_fov:
#          
#         triangle = triangle_fov[triangle_key]
#         
#         a = triangle.a
#         b = triangle.b
#         c = triangle.c
#          
#         plt.plot((a.x,b.x,c.x,a.x), (a.y,b.y,c.y,a.y), 'r')
#          
#         plt.axes().set_aspect('equal', 'datalim')
#         
#     plt.show()
    pass
